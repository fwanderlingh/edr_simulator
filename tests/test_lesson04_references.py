"""Instructor solutions match exercise interfaces and run on student classes."""

import inspect
import math

import numpy as np
import pytest

from robotics_sim import Pose
from robotics_sim.exercise_checks import ExerciseChecks
from robotics_sim.exercises.lesson03 import control
from robotics_sim.exercises.lesson04 import avoidance, estimation, lidar, rangefinder
from robotics_sim.session import ActivitySession
from robotics_sim.sim.references.avoidance import ReferenceObstacleSupervisor, ReferenceSectors
from robotics_sim.sim.references.control import ReferencePoseController
from robotics_sim.sim.references.estimation import ReferenceInertial, ReferenceOdometry
from robotics_sim.sim.references.lidar import ReferenceLidar
from robotics_sim.sim.references.rangefinder import ReferenceRangeStop


@pytest.mark.parametrize("student,reference,method", [
    (lidar.StudentLidar, ReferenceLidar, "directions"),
    (rangefinder.StudentRangeStop, ReferenceRangeStop, "command"),
    (avoidance.StudentSectors, ReferenceSectors, "summarize"),
    (avoidance.StudentObstacleSupervisor, ReferenceObstacleSupervisor, "command"),
    (estimation.StudentOdometry, ReferenceOdometry, "update"),
    (estimation.StudentInertial, ReferenceInertial, "update"),
])
def test_reference_method_matches_student_signature(student, reference, method):
    assert inspect.signature(getattr(student, method)) == inspect.signature(getattr(reference, method))


def test_reference_lidar_known_rotations_and_unit_directions():
    sensor = ReferenceLidar(None, angles=[-math.pi / 2, 0, math.pi / 2])
    assert np.asarray(sensor.directions(math.pi / 2)) == pytest.approx(np.array([
        [1, 0, 0], [0, 1, 0], [-1, 0, 0],
    ]))
    # A general yaw must keep every ray horizontal and unit length.
    pose = Pose(x=3, y=-2, yaw=math.pi / 4)
    sensor = ReferenceLidar(None)
    rays = np.asarray(sensor.directions(pose.yaw))
    assert rays.shape == (37, 3)
    assert np.linalg.norm(rays, axis=1) == pytest.approx(np.ones(37))
    assert rays[:, 2] == pytest.approx(np.zeros(37))
    assert rays[18] == pytest.approx((math.sqrt(0.5), math.sqrt(0.5), 0))


def test_runtime_reference_mode_uses_the_named_solution_files():
    for task, controller in (
        ("sensor_lidar", None),
        ("sensor_front_stop", ReferenceRangeStop),
        ("sensor_avoidance", ReferenceObstacleSupervisor),
    ):
        session = ActivitySession("4", task)
        try:
            assert type(session.sensors.lidar) is ReferenceLidar
            if controller is not None:
                assert type(session.laboratory.controller) is controller
        finally:
            session.close()


def test_reference_methods_complete_the_student_obstacle_route(monkeypatch):
    monkeypatch.setattr(control.StudentPoseController, "wheel_command", ReferencePoseController.wheel_command)
    monkeypatch.setattr(lidar.StudentLidar, "directions", ReferenceLidar.directions)
    monkeypatch.setattr(avoidance.StudentSectors, "summarize", ReferenceSectors.summarize)
    monkeypatch.setattr(avoidance.StudentObstacleSupervisor, "command", ReferenceObstacleSupervisor.command)
    monkeypatch.setattr(avoidance, "LIDAR_MODEL", "student")
    monkeypatch.setattr(avoidance, "AVOIDANCE_STAGE", "avoid")
    session = ActivitySession("4", "sensor_avoidance", exercise_model="student", duration=35)
    try:
        session.start()
        while session.state == "Running":
            session.step()
        assert type(session.sensors.lidar) is lidar.StudentLidar
        assert session.outcome == "DONE"
        assert math.hypot(3 - session.robot.pose().x, session.robot.pose().y) <= control.EPS_POS
        assert session.robot.wheel_velocities == (0, 0)
        assert all(ok for _, ok in ExerciseChecks(session).evaluate())
    finally:
        session.close()
