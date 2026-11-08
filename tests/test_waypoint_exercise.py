"""Check arrival, limits and timeout independently of the student solution."""

import csv
import math

import pytest

from robotics_sim import Pose
from robotics_sim.exercises.lesson03.waypoint import StudentWaypointController
from robotics_sim.exercises.lesson03._control_runtime import main, run
from robotics_sim.sim.references.waypoint_control import ReferenceWaypointController
from robotics_sim.robots.differential_drive import ReferenceDifferentialDrive


@pytest.mark.parametrize("goals", [
    ((1.0, 0.5, math.pi / 2), (1.0, 1.5, 0.0)),
    ((-1.0, 0.0, 0.0),),
    ((0.0, 0.0, 0.0),),
])
def test_reference_arrival_and_wheel_limits(goals, tmp_path):
    path = tmp_path / "run.csv"
    result = run(task="waypoints", model="reference", goals=goals, csv_path=path)
    assert result["status"] == "DONE"
    assert len(result["arrivals"]) == len(goals)
    assert result["final_wheels"] == (0.0, 0.0)
    for arrival in result["arrivals"]:
        assert arrival["position_error_m"] <= 0.03
        assert abs(arrival["angle_error_rad"]) <= 0.03
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    assert all(max(abs(float(r["w_left_rad_s"])), abs(float(r["w_right_rad_s"])))
               <= 5.0 + 1e-12 for r in rows)


def test_timeout_is_distinct_and_stops(tmp_path):
    result = run(task="waypoints", model="reference", duration=0.02, csv_path=tmp_path / "timeout.csv")
    assert result["status"] == "TIMEOUT"
    assert result["completed"] == 0
    assert result["final_wheels"] == (0.0, 0.0)


def test_target_at_center_needs_only_final_rotation():
    ctrl = ReferenceWaypointController(((0, 0, math.pi / 2),))
    r, L = .12, .42
    left, right = ctrl.wheel_command(Pose(), r, L)
    v, omega = r * (right + left) / 2, r * (right - left) / L
    assert ctrl.phase == "ALIGN" and v == 0 and omega > 0


@pytest.mark.parametrize("goal,phase,v,omega", [
    ((0, 1, 0), "TURN", 0.0, math.pi),
    ((1, 0, 0), "DRIVE", 0.8, 0.0),
    ((0, 0, math.pi / 2), "ALIGN", 0.0, 1.5 * math.pi / 2),
])
def test_reference_wheel_commands_include_turning_and_robot_geometry(goal, phase, v, omega):
    ctrl = ReferenceWaypointController((goal,))
    model = ReferenceDifferentialDrive(wheel_radius=.09, track_width=.6)
    wheels = ctrl.wheel_command(Pose(), model.wheel_radius, model.track_width)
    assert ctrl.phase == phase
    assert wheels == pytest.approx(model.inverse(v, omega))


def test_incomplete_student_entry_point_is_reported(monkeypatch, capsys, tmp_path):
    def incomplete(self, pose, r, L):
        raise NotImplementedError("Complete wheel_command")
    monkeypatch.setattr(StudentWaypointController, "wheel_command", incomplete)
    assert main(["--csv", str(tmp_path / "incomplete.csv")], task="waypoints") == 2
    assert "NOT IMPLEMENTED: Complete wheel_command" in capsys.readouterr().out
