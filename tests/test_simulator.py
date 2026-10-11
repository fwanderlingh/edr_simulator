import math

import pytest

from robotics_sim import Pose, Robot, Simulator, Velocity, World
from robotics_sim.exercises.lesson01.program import SquareProgram


def test_body_frame_command():
    with World(gui=False, dt=0.01) as world:
        robot = Robot(world, Pose(yaw=math.pi / 2))
        robot.set_velocity(vx=1, frame="body")
        sim = Simulator(world, robot)
        for _ in range(100):
            pose = sim.step()
        assert pose.x == pytest.approx(0, abs=1e-6)
        assert pose.y == pytest.approx(1, abs=1e-6)


def test_square_program_returns_to_start():
    with World(gui=False) as world:
        robot = Robot(world)
        sim = Simulator(world, robot, program=SquareProgram())
        for _ in range(3840):
            pose = sim.step()
        assert pose.x == pytest.approx(0, abs=1e-6)
        assert pose.y == pytest.approx(0, abs=1e-6)
        assert sim.step().x == pytest.approx(0, abs=1e-6)


def test_obstacle_blocks_prescribed_motion():
    with World(gui=False) as world:
        world.add_box(Pose(x=1, z=0.3), size=(0.2, 2, 0.6))
        robot = Robot(world)
        robot.set_velocity(vx=0.5)
        sim = Simulator(world, robot)
        for _ in range(960):
            pose = sim.step()
        assert 0.6 < pose.x < 0.72


def test_worlds_are_isolated_and_close_is_idempotent():
    with World(gui=False) as first, World(gui=False) as second:
        a, b = Robot(first), Robot(second)
        a.set_velocity(vx=1)
        Simulator(first, a).step()
        assert b.pose().x == 0
        with pytest.raises(ValueError):
            Simulator(first, b)
        first.close()
        first.close()
        with pytest.raises(RuntimeError, match="closed"):
            a.pose()
        assert second.physics.connected


def test_student_program_receives_simulation_time():
    class StudentProgram:
        def __init__(self):
            self.calls = []

        def update(self, time, dt):
            self.calls.append((time, dt))
            return Velocity(vx=0.3)

    program = StudentProgram()
    with World(gui=False, dt=0.1) as world:
        sim = Simulator(world, Robot(world), program=program)
        sim.step()
        pose = sim.step()
        assert program.calls == [(0.0, 0.1), (0.1, 0.1)]
        assert pose.x == pytest.approx(0.06, abs=1e-6)
