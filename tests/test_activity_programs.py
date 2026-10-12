"""Lifecycle and shared-world stepping for explicit robot examples."""
from unittest.mock import Mock

import numpy as np
import pytest

from robotics_sim import Pose, Robot, Simulator, World
from robotics_sim.exercises.lesson01 import circle, square, straight
from robotics_sim.session import ActivitySession


@pytest.mark.parametrize("example", [straight, circle, square])
def test_setup_creates_one_robot_and_update_receives_it(example):
    with World(gui=False, dt=0.01) as world:
        robot = example.setup(world)
        update = Mock(wraps=example.update)
        sim = Simulator(world, robot, update=update)
        sim.step()
        sim.step()
        assert update.call_count == 2
        assert update.call_args_list[0].args == (robot, 0.0, 0.01)
        assert update.call_args_list[1].args == (robot, 0.01, 0.01)
        assert robot.pose().x > 0


def test_named_robots_share_one_physics_step_and_stop_together():
    with World(gui=False, dt=0.1) as world:
        first = Robot(world, Pose(x=-2, y=-2))
        second = Robot(world, Pose(x=2, y=2))
        observed = []
        def move_first(robot, time, dt):
            observed.append(("first", time, second.pose().y))
            robot.set_velocity(vx=0.5)
        def move_second(robot, time, dt):
            observed.append(("second", time, first.pose().x))
            robot.set_velocity(vy=-0.3)
        sim = Simulator(world, first, update=move_first)
        sim.register_robot("partner", second, update=move_second)
        world.physics.step = Mock(wraps=world.physics.step)
        pose = sim.step()
        world.physics.step.assert_called_once()
        assert observed == [("first", 0, 2), ("second", 0, -2)]
        np.testing.assert_allclose((pose.x, pose.y), (-1.95, -2), atol=1e-6)
        np.testing.assert_allclose((second.pose().x, second.pose().y), (2, 1.97), atol=1e-6)
        assert sim.time == 0.1
        sim.stop()
        assert first.command.vx == second.command.vy == 0
        with pytest.raises(ValueError, match="before the first"):
            sim.register_robot("late", second)


def test_registry_rejects_duplicates_and_foreign_worlds():
    with World(gui=False) as world, World(gui=False) as other_world:
        first = Robot(world)
        second = Robot(world, Pose(x=2))
        sim = Simulator(world, first)
        with pytest.raises(ValueError, match="unique"):
            sim.register_robot("robot", second)
        with pytest.raises(ValueError, match="already registered"):
            sim.register_robot("duplicate", first)
        with pytest.raises(ValueError, match="world"):
            sim.register_robot("foreign", Robot(other_world))
        with pytest.raises(TypeError):
            sim.robots["unvalidated"] = second
        with pytest.raises(ValueError, match="either"):
            Simulator(world, first, update=straight.update, program=object())


def test_session_uses_setup_on_each_new_activity_and_stops_all_robots(monkeypatch):
    setup = Mock(wraps=straight.setup)
    monkeypatch.setattr(straight, "setup", setup)
    for _ in range(2):
        session = ActivitySession(lesson="1", task="straight", duration=0.02, dt=0.01)
        try:
            partner = Robot(session.world, Pose(x=-2, y=-2))
            session.sim.register_robot("partner", partner, update=straight.update)
            assert session.sim.time == 0
            session.start()
            session.step()
            session.pause()
            session.step()
            assert session.sim.time == 0.01
            session.step(single=True)
            assert session.state == "Completed"
            assert all(robot.command.vx == 0 for robot in session.sim.robots.values())
        finally:
            session.close()
    assert setup.call_count == 2


@pytest.mark.parametrize("lesson,task,vx,omega", [
    ("2", "planar", 0, 0), ("2", "static", 0, 0), ("2", "frames", 0, 0.3),
    ("3", "drive_line", 0.24, 0), ("3", "drive_circle", 0.24, 0.4),
    ("3", "drive_spin", 0, 0.24 / 0.42), ("3", "drive_inverse", 0.24, 0.4),
    ("4", "sensor_encoders", 0.15, 0), ("4", "sensor_lidar", 0, 0.3),
    ("4", "sensor_imu", 0.2, 0.4),
])
def test_lesson_callbacks_preserve_motion_and_reset(lesson, task, vx, omega):
    for _ in range(2):
        session = ActivitySession(lesson, task, duration=0.03, dt=0.01)
        try:
            initial = session.robot.pose()
            session.step(single=True)
            assert session.robot.command.vx == pytest.approx(vx)
            assert session.robot.command.omega == pytest.approx(omega)
            pose = session.robot.pose()
            assert pose.x - initial.x == pytest.approx(vx * 0.01, abs=1e-6)
            assert pose.yaw - initial.yaw == pytest.approx(omega * 0.01, abs=1e-6)
            if lesson == "4":
                assert session.sensors.robot is session.robot
            session.start()
            session.step()
            session.step()
            assert session.state == "Completed"
            assert session.robot.command.vx == session.robot.command.omega == 0
            assert session.sim.steps == 3
        finally:
            session.close()


def test_planar_attachment_and_exercise_follow_moving_robot(monkeypatch):
    from robotics_sim import app
    from robotics_sim.exercises.lesson02 import activity

    def move(robot, time, dt, *, task):
        robot.set_velocity(vx=0.2, omega=0.4, frame="world")

    monkeypatch.setattr(activity, "update", move)
    solver = Mock(return_value=(np.zeros(2), np.zeros(2)))
    monkeypatch.setattr(app, "solve_planar_task", solver)
    for _ in range(2):
        session = ActivitySession("2", "planar", duration=1, dt=0.1)
        try:
            scene = session.planar_scene
            assert scene.robot is session.robot
            np.testing.assert_allclose(scene.sensor_frame()[:2, 3], [2, 1.5])
            session.step(single=True)
            angle = np.pi / 2 + 0.04
            expected_robot = np.array([
                [np.cos(angle), -np.sin(angle), 2.02],
                [np.sin(angle), np.cos(angle), 1], [0, 0, 1]])
            np.testing.assert_allclose(scene.world_from_robot(), expected_robot, atol=1e-6)
            sensor = scene.sensor_frame()
            np.testing.assert_allclose(sensor[:2, 3],
                [2.02 + 0.5 * np.cos(angle), 1 + 0.5 * np.sin(angle)], atol=1e-6)
            np.testing.assert_allclose(sensor[:2, :2], expected_robot[:2, :2], atol=1e-6)
            # Exercise inputs come from the same live attachment used for drawing.
            viewer = app.SimulatorApp.__new__(app.SimulatorApp)
            viewer.session = session
            viewer._planar_output()
            world_robot, robot_sensor, point, direction = solver.call_args.args
            np.testing.assert_allclose(world_robot, expected_robot, atol=1e-6)
            np.testing.assert_allclose(robot_sensor, scene.robot_from_sensor)
            np.testing.assert_allclose(point, [1, 0])
            np.testing.assert_allclose(direction, [1, 0])
            # Per-session attachment edits must not affect future resets.
            scene.robot_from_sensor[0, 2] = 10
        finally:
            session.close()
