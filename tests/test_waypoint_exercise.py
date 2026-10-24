"""Check arrival, limits and timeout independently of the student solution."""

import csv
import math

import pytest

from robotics_sim import Pose
from robotics_sim.exercises.lesson03.control import StudentWaypointController, main, run
from robotics_sim.exercises.lesson03.control_reference import ReferenceWaypointController


@pytest.mark.parametrize("goals", [
    ((1.0, 0.5, math.pi / 2), (1.0, 1.5, 0.0)),
    ((-1.0, 0.0, 0.0),),
    ((0.0, 0.0, 0.0),),
])
def test_reference_arrival_and_wheel_limits(goals, tmp_path):
    path = tmp_path / "run.csv"
    result = run(model="reference", goals=goals, csv_path=path)
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
    result = run(model="reference", duration=0.02, csv_path=tmp_path / "timeout.csv")
    assert result["status"] == "TIMEOUT"
    assert result["completed"] == 0
    assert result["final_wheels"] == (0.0, 0.0)


def test_target_at_center_needs_only_final_rotation():
    ctrl = ReferenceWaypointController(((0, 0, math.pi / 2),))
    v, omega = ctrl.nominal_command(Pose())
    assert ctrl.phase == "ALIGN" and v == 0 and omega > 0


def test_incomplete_student_entry_point_is_reported(monkeypatch, capsys, tmp_path):
    def incomplete(self, pose):
        raise NotImplementedError("Complete nominal_command")
    monkeypatch.setattr(StudentWaypointController, "nominal_command", incomplete)
    assert main(["--csv", str(tmp_path / "incomplete.csv")]) == 2
    assert "NOT IMPLEMENTED: Complete nominal_command" in capsys.readouterr().out
