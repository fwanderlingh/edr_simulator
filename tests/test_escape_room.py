"""The final set is inspectable, stationary and physically enclosed."""

import numpy as np
import pybullet as bullet
import pytest

from robotics_sim import Pose, Robot
from robotics_sim.exercises.lesson08 import activity
from robotics_sim.exercise_checks import ExerciseChecks
from robotics_sim.session import ActivitySession, TASKS
from robotics_sim.sim.camera import Camera
from robotics_sim.sim.scene_renderer import BoxScene


def test_preview_opens_without_exercise_implementations_and_stays_stationary():
    session = ActivitySession("8", exercise_model="student", kinematics="student",
                              sensor_mode="student", duration=1, dt=.02)
    try:
        assert session.task in TASKS["8"]
        assert session.state == "Ready" and session.sim.time == 0
        assert session.controller is session.laboratory is session.sensors is None
        assert not ExerciseChecks(session).evaluate()
        initial = session.robot.pose()
        client = session.world.physics.client
        before = [bullet.getBasePositionAndOrientation(i, physicsClientId=client)
                  for i in range(bullet.getNumBodies(physicsClientId=client))]
        session.robot.set_twist(.4, .5)  # Preview must cancel any held command.
        session.start()
        while session.state == "Running":
            session.step()
        assert session.state == "Completed" and session.world.physics.connected
        assert session.robot.pose() == initial
        assert session.robot.wheel_velocities == (0, 0)
        after = [bullet.getBasePositionAndOrientation(i, physicsClientId=client)
                 for i in range(bullet.getNumBodies(physicsClientId=client))]
        assert after == before
    finally:
        session.close()


def test_dense_perimeter_scan_finds_only_the_exit_opening():
    session = ActivitySession("8")
    try:
        origins, ends, expected_open = [], [], []
        w, h = activity.HALF_WIDTH, activity.HALF_DEPTH
        # Sample cell centres, avoiding the exact shared wall/jamb surfaces.
        for x in np.linspace(-w + .025, w - .025, 176):
            for sign in (-1, 1):
                origins.append((x, sign * (h - .12), .2))
                ends.append((x, sign * (h + .35), .2))
                expected_open.append(sign == 1 and activity.EXIT_X[0] < x < activity.EXIT_X[1])
        for y in np.linspace(-h, h, 141):
            for sign in (-1, 1):
                origins.append((sign * (w - .12), y, .2))
                ends.append((sign * (w + .35), y, .2))
                expected_open.append(False)
        for (body, _), is_open in zip(session.world.raycast(origins, ends), expected_open):
            assert (body == -1) == is_open
    finally:
        session.close()


@pytest.mark.parametrize("x,y,vx,vy,axis,sign,limit", [
    (-4.0, -1.5, -.4, 0, "x", -1, 4.4),
    (4.0, -1.5, .4, 0, "x", 1, 4.4),
    (0, -3.0, 0, -.4, "y", -1, 3.5),
    (-.5, 3.0, 0, .4, "y", 1, 3.5),
])
def test_perimeter_stops_a_real_physics_body(x, y, vx, vy, axis, sign, limit):
    session = ActivitySession("8", dt=.01)
    try:
        probe = Robot(session.world, Pose(x=x, y=y, z=.17))
        probe.set_velocity(vx=vx, vy=vy)
        for _ in range(200):
            probe.apply_command()
            session.world.physics.step()
        # The centre cannot cross a wall, even with commands held against it.
        assert sign * getattr(probe.pose(), axis) < limit - .1
    finally:
        session.close()


def test_exit_has_physical_clearance_below_the_parked_door():
    session = ActivitySession("8", dt=.01)
    try:
        probe = Robot(session.world, Pose(x=sum(activity.EXIT_X) / 2, y=3.0, z=.17))
        probe.set_velocity(vy=.4)
        for _ in range(200):
            probe.apply_command()
            session.world.physics.step()
        assert probe.pose().y > activity.HALF_DEPTH + .25
    finally:
        session.close()


def test_preview_uses_cached_primitive_geometry_and_overview_camera():
    session = ActivitySession("8")
    try:
        camera = Camera()
        for key, value in session.world.preview_camera.items():
            setattr(camera, key, value)
        camera.prepare(1200, 850)
        scene = BoxScene(session.world)
        assert len(scene.shapes) < 120
        first = scene.faces(camera)
        second = scene.faces(camera)
        assert len(first) < 1200
        assert all(a is b for a, b in zip(first, second) if a.key[0] >= 0)
        # All room corners, including the exterior exit landing, fit the overview.
        for x in (-4.6, 4.6):
            for y in (-3.7, 4.7):
                px, py = camera.project((x, y, 0))
                assert 0 < px < camera.width and 0 < py < camera.height
    finally:
        session.close()
