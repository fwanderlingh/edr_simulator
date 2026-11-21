"""Numerical checks and physical outcomes for the sensor activities in the UI."""

import csv
import math

import pytest

from robotics_sim import Pose
from robotics_sim.session import ActivitySession
from robotics_sim.sim.references.sensors import (
    ReferenceInertial, ReferenceObstacleSupervisor, ReferenceOdometry, ReferenceSectors,
)


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def test_encoder_arc_and_zero_increment():
    model = ReferenceOdometry(radius=0.1, width=0.5)
    model.update(1, 2, 0.2)
    angle, distance = 0.2, 0.15
    radius = distance / angle
    assert model.x == pytest.approx(radius * math.sin(angle))
    assert model.y == pytest.approx(radius * (1 - math.cos(angle)))
    previous = model.x, model.y, model.yaw
    model.update(0, 0, 0.2)
    assert (model.x, model.y, model.yaw) == previous
    assert model.speed == model.yaw_rate == 0


def test_inertial_constant_acceleration_and_rotated_frame():
    imu = ReferenceInertial()
    for _ in range(20):
        imu.update(0, 0.1 * 9.80665, 0, 0.1, truth_yaw=math.pi / 2)
    assert imu.x == pytest.approx(0, abs=1e-12)
    assert imu.vy == pytest.approx(1.96133)
    assert imu.y == pytest.approx(1.96133)


def test_curved_estimation_with_truth_assisted_orientation(tmp_path):
    duration = 2.0
    path = tmp_path / "assisted-curve.csv"
    session = ActivitySession("4", "sensor_estimation_curve", duration=duration,
                              dt=1 / 120, exercise_model="reference")
    try:
        session.laboratory.truth_orientation = True
        session.start()
        while session.state == "Running":
            session.step()
        assert session.outcome == "COMPLETED"
        session.export_csv(path)
        rows = read_rows(path)
        assert float(rows[0]["time_s"]) == pytest.approx(session.dt)
        assert float(rows[-1]["time_s"]) == pytest.approx(duration)
        assert float(rows[-1]["enc_position_error_m"]) < 0.005
        assert float(rows[-1]["imu_position_error_m"]) < 0.03
    finally:
        session.close()


def test_sector_misses_and_hits():
    summary = ReferenceSectors()
    angles = [-math.pi / 2, 0, math.pi / 2]
    assert summary.summarize(angles, [4, 0.5, 4], [False, True, False], 4) == {
        "front": 0.5, "left": 4, "right": 4,
    }
    assert summary.summarize(angles, [4, 4, 4], [False] * 3, 4) == {
        "front": 4, "left": 4, "right": 4,
    }
    assert summary.summarize(angles, [0.7, 4, 0.9], [True, False, True], 4) == {
        "front": 4, "left": 0.9, "right": 0.7,
    }


def test_sector_boundaries_and_minimum_of_multiple_returns():
    summary = ReferenceSectors()
    angles = [-math.pi / 2, -math.pi / 6, 0, math.pi / 6, math.pi / 2]
    ranges = [0.9, 0.4, 0.7, 0.3, 1.1]
    assert summary.summarize(angles, ranges, [True] * 5, 4) == {
        "front": 0.3, "left": 1.1, "right": 0.9,
    }


@pytest.mark.parametrize("left,right,turn_direction", [(4, 0.7, 1), (0.7, 4, -1), (4, 4, 1)])
def test_supervisor_stops_then_turns_toward_the_clearer_side(left, right, turn_direction):
    controller = ReferenceObstacleSupervisor()
    sectors = {"front": 0.5, "left": left, "right": right}
    assert controller.command(Pose(), (3, 0, 0), sectors) == (0, 0)
    assert controller.phase == "STOP"
    v, omega = controller.command(Pose(), (3, 0, 0), sectors)
    assert v == 0 and omega * turn_direction > 0
    assert controller.phase == "DETOUR"
    assert controller.nominal.controller.phase == "TURN"
    assert controller.detour_goal == pytest.approx((0, turn_direction, turn_direction * math.pi / 2))


def test_supervisor_remains_stopped_when_neither_side_has_enough_space():
    controller = ReferenceObstacleSupervisor()
    sectors = {"front": 0.5, "left": 0.6, "right": 0.7}
    assert controller.command(Pose(), (3, 0, 0), sectors) == (0, 0)
    assert controller.command(Pose(), (3, 0, 0), sectors) == (0, 0)
    assert controller.phase == "STOP"


def test_detour_keeps_one_world_goal_and_reuses_the_same_pose_controller():
    from unittest.mock import patch

    controller = ReferenceObstacleSupervisor()
    nominal = controller.nominal.controller
    goal = (6, 0, 0)
    start = Pose(x=2, y=-1, yaw=math.pi / 2)
    clear = {"front": 4, "left": 4, "right": 4}
    blocked = {"front": .5, "left": 4, "right": .7}
    with patch.object(nominal, "set_goal", wraps=nominal.set_goal) as select:
        controller.command(start, goal, clear)
        controller.command(start, goal, blocked)
        controller.command(start, goal, blocked)
        intermediate = controller.detour_goal
        assert intermediate == pytest.approx((1, -1, -math.pi))
        # Changed measurements and pose must not make the intermediate target move.
        controller.command(Pose(x=1.8, y=-1, yaw=2.8), goal, {"front": 4, "left": .7, "right": 4})
        assert controller.detour_goal is intermediate
        assert controller.nominal.controller is nominal
        assert select.call_count == 2  # Final target, then intermediate target.
        # Position alone is insufficient: the same controller finishes alignment.
        controller.command(Pose(x=1, y=-1, yaw=2.9), goal, clear)
        assert controller.phase == "DETOUR" and nominal.phase == "ALIGN"
        controller.command(Pose(x=1, y=-1, yaw=-math.pi), goal, clear)
        assert controller.phase == "GO" and controller.detour_goal is None
        assert nominal.goal == goal and select.call_count == 3
        assert [call.args[0] for call in select.call_args_list] == [goal, intermediate, goal]


def test_detour_allows_rotation_and_gates_advance_with_hysteresis():
    controller = ReferenceObstacleSupervisor()
    goal = (3, 0, 0)
    sectors = {"front": .5, "left": 4, "right": 4}
    controller.command(Pose(), goal, sectors)
    v, omega = controller.command(Pose(), goal, sectors)
    assert v == 0 and omega > 0  # The front obstacle must not prevent the detour turn.
    intermediate = controller.detour_goal
    pose = Pose(yaw=math.pi / 2)
    for distance, allowed in ((.7, False), (.8, True), (.7, True), (.5, False), (.7, False), (.8, True)):
        sectors["front"] = distance
        command = controller.command(pose, goal, sectors)
        assert controller.advance_allowed is allowed
        assert (command[0] > 0) is allowed
        if not allowed:
            assert command == (0, 0)
        assert controller.phase == "DETOUR" and controller.detour_goal is intermediate
