"""Verify independent stages and actual reuse of the preceding lesson's code."""

import math

import pytest

from robotics_sim import Pose
from robotics_sim.session import ActivitySession
from robotics_sim.exercise_checks import ExerciseChecks
from robotics_sim.exercises.lesson03 import control
from robotics_sim.exercises.lesson04 import avoidance, estimation
from robotics_sim.exercises.lesson04.lidar import StudentLidar
from robotics_sim.sim.references.control import ReferencePoseController
from robotics_sim.sim.references.sensors import (
    ReferenceInertial, ReferenceObstacleSupervisor, ReferenceOdometry, ReferenceSectors,
)


def unfinished(*args, **kwargs):
    raise NotImplementedError("This later component must not be called yet")


def finish(session):
    session.start()
    while session.state == "Running":
        session.step()


@pytest.mark.parametrize("stage", ["encoder", "imu"])
def test_estimator_can_be_verified_before_the_other_is_implemented(monkeypatch, stage):
    monkeypatch.setattr(estimation, "ESTIMATION_STAGE", stage)
    monkeypatch.setattr(estimation.StudentOdometry, "update",
                        ReferenceOdometry.update if stage == "encoder" else unfinished)
    monkeypatch.setattr(estimation.StudentInertial, "update",
                        ReferenceInertial.update if stage == "imu" else unfinished)
    # ReferenceOdometry supports two documented algorithms; this student chooses arc.
    if stage == "encoder":
        monkeypatch.setattr(estimation.StudentOdometry, "integration", "arc", raising=False)
    session = ActivitySession("4", "sensor_estimation_curve", duration=.3, exercise_model="student")
    try:
        finish(session)
        assert all(ok for _, ok in ExerciseChecks(session).evaluate())
        assert session.laboratory.odom_model == ("student" if stage == "encoder" else "reference")
        assert session.laboratory.imu_model == ("student" if stage == "imu" else "reference")
        assert f"Stage: {stage}" in session.laboratory.readout
    finally:
        session.close()


def test_sectors_run_without_supervisor_or_previous_controller(monkeypatch):
    monkeypatch.setattr(avoidance, "AVOIDANCE_STAGE", "sectors")
    monkeypatch.setattr(avoidance.StudentSectors, "summarize", ReferenceSectors.summarize)
    monkeypatch.setattr(avoidance.StudentObstacleSupervisor, "command", unfinished)
    monkeypatch.setattr(control.StudentPoseController, "wheel_command", unfinished)
    session = ActivitySession("4", "sensor_avoidance", duration=.3, exercise_model="student")
    try:
        finish(session)
        assert session.outcome == "COMPLETED"
        assert session.laboratory.controller.phase == "SECTORS"
        assert all(ok for _, ok in ExerciseChecks(session).evaluate())
    finally:
        session.close()


def test_stop_stage_records_stop_without_claiming_goal_arrival(monkeypatch):
    monkeypatch.setattr(avoidance, "AVOIDANCE_STAGE", "stop")
    session = ActivitySession("4", "sensor_avoidance", duration=8)
    try:
        finish(session)
        assert session.outcome == "COMPLETED"
        assert session.laboratory.controller.phase == "STOP"
        assert session.robot.pose().x < 1
        assert all(ok for _, ok in ExerciseChecks(session).evaluate())
    finally:
        session.close()


def test_nominal_motion_calls_student_lesson3_and_keeps_its_phase(monkeypatch):
    calls = []

    def completed_pose_exercise(self, pose, r, L):
        calls.append((self, r, L, self.gain_pos))
        self.phase = "DRIVE"
        return 1.0, 2.0

    monkeypatch.setattr(control.StudentPoseController, "wheel_command", completed_pose_exercise)
    monkeypatch.setattr(control, "GAIN_POS", .37)
    monkeypatch.setattr(control, "WHEEL_MAX", .75)
    monkeypatch.setattr(avoidance.StudentSectors, "summarize", ReferenceSectors.summarize)
    monkeypatch.setattr(avoidance.StudentObstacleSupervisor, "command", ReferenceObstacleSupervisor.command)
    session = ActivitySession("4", "sensor_avoidance", exercise_model="student")
    try:
        lab = session.laboratory
        assert type(lab.pose_controller) is control.StudentPoseController
        sectors = {"front": 4, "left": 4, "right": 4}
        command = lab.controller.command(Pose(), (3, 0), sectors, 0)
        assert command == pytest.approx((.18, .12 / .42))
        lab.controller.command(Pose(), (3, 0), sectors, 0)
        assert len(calls) == 2 and calls[0][0] is calls[1][0] is lab.pose_controller
        assert calls[0][3] == .37 and lab.wheel_max == .75
        assert lab.pose_controller.phase == "DRIVE"
        # New body commands from the supervisor use the SAME editable wheel limit.
        monkeypatch.setattr(lab.controller, "command", lambda *args: (10, 3))
        session.step(single=True)
        assert max(map(abs, session.robot.wheel_velocities)) == pytest.approx(.75)
    finally:
        session.close()


def test_adapter_resets_only_on_new_goal_or_after_detour():
    from robotics_sim.exercises.lesson04._runtime import PoseCommandAdapter
    controller = ReferencePoseController()
    adapter = PoseCommandAdapter(controller)
    pose = Pose()
    adapter.command(pose, (3, 0))
    assert controller.phase == "DRIVE"
    controller.phase = "ALIGN"  # Deliberate state marker to detect an unwanted set_goal.
    adapter.command(pose, (3, 0))
    assert controller.phase == "DRIVE"  # ALIGN exits naturally when outside tolerance.
    controller.index = 7
    adapter.command(pose, (3, 0))
    assert controller.index == 7  # set_goal would have reset the index.
    adapter.reset()
    adapter.command(pose, (3, 0))
    assert controller.index == 0


def test_midpoint_solution_passes_but_wrong_state_is_still_rejected(monkeypatch):
    def midpoint(self, left, right, dt):
        ds = self.radius * (right + left) / 2
        da = self.radius * (right - left) / self.width
        self.x += ds * math.cos(self.yaw + da / 2)
        self.y += ds * math.sin(self.yaw + da / 2)
        self.yaw += da
        self.speed, self.yaw_rate = ds / dt, da / dt

    monkeypatch.setattr(estimation.StudentOdometry, "update", midpoint)
    monkeypatch.setattr(estimation.StudentInertial, "update", ReferenceInertial.update)
    session = ActivitySession("4", "sensor_estimation_curve", duration=2, exercise_model="student")
    try:
        checks = ExerciseChecks(session)
        finish(session)
        assert all(ok for _, ok in checks.evaluate())
        session.laboratory.odom.x += .1
        assert [ok for _, ok in checks.evaluate()] == [True, False, True]
    finally:
        session.close()


def test_student_lidar_is_reused_independently_of_noise(monkeypatch):
    def directions(self, yaw):
        return [(math.cos(yaw + a), math.sin(yaw + a), 0) for a in self.angles]

    monkeypatch.setattr(StudentLidar, "directions", directions)
    monkeypatch.setattr(avoidance, "LIDAR_MODEL", "student")
    monkeypatch.setattr(avoidance, "AVOIDANCE_STAGE", "sectors")
    session = ActivitySession("4", "sensor_avoidance", sensor_mode="noisy", duration=.2)
    try:
        assert type(session.sensors.lidar) is StudentLidar
        assert session.sensors.lidar.noisy
        finish(session)
        assert all(ok for _, ok in ExerciseChecks(session).evaluate())
    finally:
        session.close()
