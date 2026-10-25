"""The introductory controller shares the app lifecycle but returns wheel speeds."""

import csv
from unittest.mock import Mock

import pytest

from robotics_sim.exercises.lesson03.proportional import StudentProportionalController
from robotics_sim.exercises.lesson03.proportional_reference import ReferenceProportionalController
from robotics_sim.robots.differential_drive import ReferenceDifferentialDrive
from robotics_sim.session import ACTIVITY_FILES, ActivitySession, TASKS


@pytest.mark.parametrize("gain", [0.4, 0.2])
def test_reference_reaches_front_goal_and_exports_results(gain, tmp_path):
    session = ActivitySession("3", "drive_proportional", duration=20, dt=1 / 120)
    try:
        assert "drive_proportional" in TASKS["3"]
        assert ACTIVITY_FILES["drive_proportional"][1].endswith("lesson03/proportional.py")
        assert isinstance(session.controller, ReferenceProportionalController)
        assert session.state == "Ready" and session.robot.wheel_velocities == (0, 0)
        session.controller.gain = gain
        session.step(single=True)
        assert session.robot.wheel_velocities == pytest.approx((gain / .12, gain / .12))
        session.start()
        while session.state == "Running":
            session.step()
        pose = session.robot.pose()
        assert session.outcome == "DONE" and session.sim.time < 20
        assert abs(1 - pose.x) <= session.controller.eps_p
        assert (pose.y, pose.yaw) == pytest.approx((0, 0), abs=1e-9)
        assert session.controller.phase == "DONE" and session.controller.index == 1
        assert session.robot.wheel_velocities == (0, 0)
        assert len(session.arrivals) == 1
        path = tmp_path / "proportional.csv"
        session.export_csv(path)
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        assert rows[-1]["outcome"] == "DONE"
        assert rows[-1]["completed_waypoints"] == "1"
        assert rows[-1]["left_rad_s"] == rows[-1]["right_rad_s"] == "0.0"
        assert all(None not in row for row in rows)
    finally:
        session.close()


def test_student_stays_inspectable_until_first_command(monkeypatch):
    def unfinished(self, pose, wheel_radius):
        raise NotImplementedError("Complete wheel_command")
    monkeypatch.setattr(StudentProportionalController, "wheel_command", unfinished)
    session = ActivitySession("3", "drive_proportional", kinematics="student")
    try:
        assert isinstance(session.controller, StudentProportionalController)
        assert type(session.robot.motor_model) is ReferenceDifferentialDrive
        assert session.controller.goals == ((1, 0, 0),)
        assert session.state == "Ready" and session.samples == []
        with pytest.raises(NotImplementedError, match="wheel_command"):
            session.step(single=True)
        assert session.sim.steps == 0 and session.samples == []
        assert session.robot.wheel_velocities == (0, 0)
    finally:
        session.close()


def test_student_values_are_applied_as_wheel_references(monkeypatch):
    command = Mock(return_value=(1.25, 1.25))
    monkeypatch.setattr(StudentProportionalController, "wheel_command", command)
    session = ActivitySession("3", "drive_proportional", kinematics="student", dt=.01)
    try:
        session.robot.motor_model.inverse = Mock(side_effect=AssertionError("Do not convert twice"))
        session.step(single=True)
        command.assert_called_once()
        assert command.call_args.args[1] == .12
        assert session.robot.wheel_velocities == (1.25, 1.25)
        assert session.robot.command.vx == pytest.approx(.15)
        assert session.robot.pose().x == pytest.approx(.0015)
        session.robot.motor_model.inverse.assert_not_called()
    finally:
        session.close()


def test_common_wheel_limit_and_timeout_stop_the_robot():
    session = ActivitySession("3", "drive_proportional", duration=.02, dt=.01)
    try:
        session.controller.gain = 4
        session.step(single=True)
        assert session.robot.wheel_velocities == pytest.approx((5, 5))
        assert session.controller.scale == pytest.approx(.15)
        session.step(single=True)
        assert session.state == "Completed" and session.outcome == "TIMEOUT"
        assert session.controller.index == 0
        assert session.robot.wheel_velocities == (0, 0)
    finally:
        session.close()


def test_proportional_goal_stays_on_the_exercise_axis():
    with pytest.raises(ValueError, match="positive x"):
        ActivitySession("3", "drive_proportional", goal=(1, .5, 0))
