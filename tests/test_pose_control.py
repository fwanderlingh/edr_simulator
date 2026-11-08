"""Pose feedback precedes, and is reused by, waypoint sequencing."""

import csv
import math

import pytest

from robotics_sim import Pose
from robotics_sim.exercises.lesson03 import control
from robotics_sim.exercises.lesson03._control_runtime import main, run
from robotics_sim.exercises.lesson03.control import StudentPoseController
from robotics_sim.exercises.lesson03.waypoint import StudentWaypointController
from robotics_sim.sim.references.control import ReferencePoseController
from robotics_sim.sim.references.waypoint_control import ReferenceWaypointController
from robotics_sim.session import ActivitySession


@pytest.mark.parametrize("phase,alpha,expected", [
    ("TURN", .07, "DRIVE"), ("TURN", .2, "TURN"),
    ("DRIVE", .2, "DRIVE"), ("DRIVE", .61, "TURN"),
    ("ALIGN", .2, "TURN"),
])
def test_pose_hysteresis_uses_updated_phase_for_current_command(phase, alpha, expected):
    controller = ReferencePoseController(((1, 0, 0),))
    controller.phase = phase
    left, right = controller.wheel_command(Pose(yaw=-alpha), .12, .42)
    assert controller.phase == expected
    assert .12 * (left + right) / 2 == pytest.approx(.8 if expected == "DRIVE" else 0)
    assert .12 * (right - left) / .42 == pytest.approx(2 * alpha)


def test_position_check_precedes_bearing_and_heading_error_is_wrapped(monkeypatch):
    def undefined_bearing(*args):
        raise AssertionError("Do not compute a bearing at the goal")
    monkeypatch.setattr(math, "atan2", undefined_bearing)
    controller = ReferencePoseController(((0, 0, -math.pi + .1),))
    left, right = controller.wheel_command(Pose(yaw=math.pi - .1), .12, .42)
    assert controller.phase == "ALIGN"
    assert left == pytest.approx(-right)
    assert .12 * (right - left) / .42 == pytest.approx(1.5 * .2)
    assert controller.wheel_command(Pose(yaw=-math.pi + .1), .12, .42) == (0, 0)
    assert controller.done and controller.phase == "DONE"
    assert controller.wheel_command(Pose(x=10), .12, .42) == (0, 0)


def test_waypoint_arrival_waits_for_heading_and_stops_for_one_sample():
    controller = ReferenceWaypointController(((0, 0, math.pi / 2), (1, 0, 0)))
    controller.wheel_command(Pose(), .12, .42)
    assert controller.index == 0 and controller.phase == "ALIGN"
    assert controller.wheel_command(Pose(yaw=math.pi / 2), .12, .42) == (0, 0)
    assert controller.index == 1 and controller.phase == "TURN"
    assert controller.wheel_command(Pose(), .12, .42) == pytest.approx((.8 / .12,) * 2)
    assert controller.phase == "DRIVE"


def test_student_waypoint_sequencing_reuses_the_student_pose_exercise(monkeypatch):
    # Supply only the sequencer first: an unfinished pose exercise must surface.
    def unfinished(self, pose, r, L):
        raise NotImplementedError("Complete StudentPoseController.wheel_command in control.py")
    monkeypatch.setattr(StudentPoseController, "wheel_command", unfinished)
    monkeypatch.setattr(StudentWaypointController, "wheel_command",
                        ReferenceWaypointController.wheel_command)
    controller = StudentWaypointController()
    assert all(type(pose) is StudentPoseController for pose in controller.controllers)
    with pytest.raises(NotImplementedError, match="StudentPoseController.*control.py"):
        controller.wheel_command(Pose(), .12, .42)
    monkeypatch.setattr(StudentPoseController, "wheel_command",
                        ReferencePoseController.wheel_command)
    session = ActivitySession("3", "drive_waypoints", kinematics="student", duration=30, dt=1/120)
    try:
        session.start()
        while session.state == "Running":
            session.step()
        assert session.outcome == "DONE" and len(session.arrivals) == 2
        assert session.robot.wheel_velocities == (0, 0)
    finally:
        session.close()


@pytest.mark.parametrize("task", ["pose", "waypoints"])
def test_standalone_runner_uses_editable_wheel_limit_and_cli_override(monkeypatch, tmp_path, task):
    monkeypatch.setattr(control, "WHEEL_MAX", .75)
    for override, expected in ((None, .75), (1.25, 1.25)):
        path = tmp_path / f"{task}-{expected}.csv"
        result = run(task=task, model="reference", duration=.02,
                     wheel_max=override, csv_path=path)
        assert result["status"] == "TIMEOUT" and result["final_wheels"] == (0, 0)
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        assert max(abs(float(rows[0][column])) for column in
                   ("w_left_rad_s", "w_right_rad_s")) == pytest.approx(expected)


def test_pose_cli_reports_its_own_unfinished_exercise(monkeypatch, capsys, tmp_path):
    def unfinished(self, pose, r, L):
        raise NotImplementedError("Complete StudentPoseController.wheel_command in control.py")
    monkeypatch.setattr(StudentPoseController, "wheel_command", unfinished)
    assert main(["--csv", str(tmp_path / "pose.csv")]) == 2
    assert "StudentPoseController.wheel_command" in capsys.readouterr().out


def test_single_pose_target_fields_apply_to_the_pose_controller():
    session = ActivitySession("3", "drive_goal", goal=(0, 0, math.pi / 2))
    try:
        session.step(single=True)
        assert session.controller.goal == (0, 0, math.pi / 2)
        assert session.controller.phase == "ALIGN"
        assert session.robot.command.vx == 0 and session.robot.command.omega > 0
    finally:
        session.close()
