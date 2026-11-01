"""Goal-control exercises use the same lifecycle as the desktop activities."""

import csv
from unittest.mock import Mock

import pytest

from robotics_sim.exercises.lesson03.control import StudentWaypointController
from robotics_sim.exercises.lesson03 import control
from robotics_sim.robots.differential_drive import ReferenceDifferentialDrive
from robotics_sim.session import ActivitySession, ACTIVITY_FILES, TASKS


@pytest.mark.parametrize("task,count", [
    ("drive_goal", 1), ("drive_waypoints", 2), ("drive_goal_behind", 1),
])
def test_reference_goals_arrive_early_and_export_correct_targets(task, count, tmp_path):
    session = ActivitySession("3", task, duration=30, dt=1/120)
    try:
        assert task in TASKS["3"]
        assert ACTIVITY_FILES[task][1].endswith("lesson03/control.py")
        assert session.controller.index == 0
        assert session.state == "Ready"
        assert session.robot.wheel_velocities == (0, 0)
        session.start()
        while session.state == "Running":
            session.step()
        assert session.state == "Completed"
        assert session.outcome == "DONE"
        assert session.controller.index == count
        assert session.sim.time < 30
        assert session.robot.wheel_velocities == (0, 0)
        assert len(session.arrivals) == count
        for arrival in session.arrivals:
            assert arrival["position_error_m"] <= session.controller.eps_pos
            assert abs(arrival["angle_error_rad"]) <= session.controller.eps_angle
        path = tmp_path / "goals.csv"
        session.export_csv(path)
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        assert rows[-1]["outcome"] == "DONE"
        assert int(rows[-1]["completed_waypoints"]) == count
        assert int(rows[-1]["goal_index"]) == count - 1
        assert all(None not in row for row in rows)
        assert all(max(abs(float(row["left_rad_s"])), abs(float(row["right_rad_s"])))
                   <= session.controller.wheel_max + 1e-12 for row in rows)
    finally:
        session.close()


def test_timeout_stops_and_new_session_resets_waypoint_state():
    for _ in range(2):
        session = ActivitySession("3", "drive_waypoints", duration=0.02, dt=0.01)
        try:
            assert session.controller.index == 0 and session.arrivals == []
            assert session.samples == [] and session.outcome == "RUNNING"
            session.start()
            session.step()
            session.pause()
            session.step()
            assert session.sim.time == 0.01
            session.step(single=True)
            assert session.state == "Completed" and session.outcome == "TIMEOUT"
            assert session.robot.wheel_velocities == (0, 0)
            assert session.world.physics.connected
        finally:
            session.close()


def test_student_controller_is_inspectable_before_missing_step(monkeypatch):
    def unfinished(self, pose, r, L):
        raise NotImplementedError("Complete wheel_command")
    monkeypatch.setattr(StudentWaypointController, "wheel_command", unfinished)
    session = ActivitySession("3", "drive_goal", kinematics="student")
    try:
        assert isinstance(session.controller, StudentWaypointController)
        assert type(session.robot.motor_model) is ReferenceDifferentialDrive
        assert session.state == "Ready" and session.sim.steps == 0
        assert session.robot.wheel_velocities == (0, 0)
        with pytest.raises(NotImplementedError, match="wheel_command"):
            session.step(single=True)
        assert session.sim.steps == 0
        assert session.samples == [] and session.controller.index == 0
    finally:
        session.close()


@pytest.mark.parametrize("r,L", [(0.12, 0.42), (0.09, 0.60)])
def test_student_computes_wheel_references_without_automatic_inverse(monkeypatch, r, L):
    command = Mock(return_value=(1.0, 3.0))
    monkeypatch.setattr(StudentWaypointController, "wheel_command", command)
    session = ActivitySession("3", "drive_goal", kinematics="student", dt=.01)
    try:
        session.robot.motor_model = ReferenceDifferentialDrive(r, L)
        session.robot.motor_model.inverse = Mock(side_effect=AssertionError("Student computes inverse kinematics"))
        session.step(single=True)
        command.assert_called_once()
        assert command.call_args.args[1:] == (r, L)
        assert session.robot.wheel_velocities == (1.0, 3.0)
        assert session.robot.command.vx == pytest.approx(2 * r)
        assert session.robot.command.omega == pytest.approx(2 * r / L)
        session.robot.motor_model.inverse.assert_not_called()
    finally:
        session.close()


@pytest.mark.parametrize("model", ["student", "reference"])
def test_exercise_settings_reach_both_controller_models(monkeypatch, model):
    monkeypatch.setattr(control, "GAIN", .25)
    monkeypatch.setattr(control, "EPS_POS", .05)
    monkeypatch.setattr(control, "GAIN_BEARING", 3.0)
    monkeypatch.setattr(control, "GAIN_HEADING", 1.2)
    monkeypatch.setattr(control, "EPS_ANGLE", .02)
    session = ActivitySession("3", "drive_goal", kinematics=model)
    try:
        ctrl = session.controller
        assert (ctrl.gain, ctrl.eps_pos) == (.25, .05)
        assert (ctrl.gain_bearing, ctrl.gain_heading, ctrl.eps_angle) == (3.0, 1.2, .02)
        assert session.sim.steps == 0
    finally:
        session.close()
