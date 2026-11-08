"""Check arrival, limits and timeout independently of the student solution."""

import csv
import math

import pytest

from robotics_sim import Pose
from robotics_sim.exercises.lesson03.activity import WAYPOINT_TASKS
from robotics_sim.session import ActivitySession
from robotics_sim.sim.references.waypoint_control import ReferenceWaypointController
from robotics_sim.robots.differential_drive import ReferenceDifferentialDrive


def test_waypoint_at_initial_pose_completes_without_motion(tmp_path, monkeypatch):
    goals = ((0.0, 0.0, 0.0),)
    monkeypatch.setitem(WAYPOINT_TASKS, "drive_waypoints", goals)
    session = ActivitySession("3", "drive_waypoints", duration=60)
    try:
        session.start()
        while session.state == "Running":
            session.step()
        assert session.outcome == "DONE"
        assert session.sim.steps == 1
        assert len(session.arrivals) == len(goals)
        assert session.robot.wheel_velocities == (0.0, 0.0)
        for arrival in session.arrivals:
            assert arrival["position_error_m"] <= 0.03
            assert abs(arrival["angle_error_rad"]) <= 0.03
        path = tmp_path / "run.csv"
        session.export_csv(path)
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        assert all(max(abs(float(row["left_rad_s"])), abs(float(row["right_rad_s"])))
                   <= 5.0 + 1e-12 for row in rows)
    finally:
        session.close()


@pytest.mark.parametrize("goal,phase,v,omega", [
    ((0, 1, 0), "TURN", 0.0, math.pi),
    ((1, 0, 0), "DRIVE", 0.8, 0.0),
    ((0, 0, math.pi / 2), "ALIGN", 0.0, 1.5 * math.pi / 2),
])
def test_reference_wheel_commands_include_turning_and_robot_geometry(goal, phase, v, omega):
    ctrl = ReferenceWaypointController((goal,), wheel_max=100)
    model = ReferenceDifferentialDrive(wheel_radius=.09, track_width=.6)
    wheels = ctrl.wheel_command(Pose(), model.wheel_radius, model.track_width)
    assert ctrl.phase == phase
    assert wheels == pytest.approx(model.inverse(v, omega))
