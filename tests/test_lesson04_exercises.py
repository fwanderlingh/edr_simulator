"""Numerical checks and physical outcomes for the standalone sensor exercises."""

import argparse
import csv
import math

import pytest

from robotics_sim import Pose
from robotics_sim.exercises.lesson04 import avoidance, estimation
from robotics_sim.exercises.lesson04.reference import (
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


@pytest.mark.parametrize("task,duration", [("curve", 2.0), ("blocked", 8.0)])
def test_estimation_same_stream_and_blocked_chassis(task, duration, tmp_path):
    path = tmp_path / f"{task}.csv"
    estimation.run(argparse.Namespace(model="reference", task=task, mode="ideal",
                                      orientation="truth", duration=duration, dt=1 / 120,
                                      csv=path, gui=False))
    rows = read_rows(path)
    assert float(rows[0]["time_s"]) == 0
    assert float(rows[-1]["time_s"]) == pytest.approx(duration)
    if task == "curve":
        assert float(rows[-1]["enc_position_error_m"]) < 0.005
        assert float(rows[-1]["imu_position_error_m"]) < 0.03
    else:
        assert float(rows[-1]["enc_position_error_m"]) > 0.3
        assert float(rows[-1]["imu_position_error_m"]) < 0.05


def test_sector_misses_and_invalid_scan():
    summary = ReferenceSectors()
    angles = [-math.pi / 2, 0, math.pi / 2]
    assert summary.summarize(angles, [4, 0.5, 4], [False, True, False], 4) == {
        "front": 0.5, "left": 4, "right": 4,
    }
    assert summary.summarize(angles, [4, float("nan"), 4], [False] * 3, 4) is None
    assert summary.summarize(angles, [4, 4], [False] * 3, 4) is None


def test_supervisor_stale_data_stop_before_resuming():
    controller = ReferenceObstacleSupervisor()
    sectors = {"front": 4, "left": 4, "right": 4}
    assert controller.command(Pose(), (3, 0), sectors, 0.3) == (0, 0)
    assert controller.phase == "STALE"
    v, _ = controller.command(Pose(), (3, 0), sectors, 0.0)
    assert v > 0


def test_reference_bypasses_isolated_obstacle(tmp_path):
    path = tmp_path / "avoidance.csv"
    status = avoidance.run(argparse.Namespace(model="reference", mode="ideal", gui=False,
                                             duration=35, csv=path))
    rows = read_rows(path)
    assert status == 0
    assert rows[-1]["phase"] == "DONE"
    assert float(rows[-1]["goal_error_m"]) <= 0.05
    assert all(max(abs(float(r["left_rad_s"])), abs(float(r["right_rad_s"])))
               <= 5 + 1e-12 for r in rows)
    assert float(rows[-1]["left_rad_s"]) == float(rows[-1]["right_rad_s"]) == 0
