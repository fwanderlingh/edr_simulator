"""Fixed-step loop; wall-clock pacing never changes the physics timestep."""

from typing import Protocol

from ..types import Velocity


class VelocityProgram(Protocol):
    """Replaceable open-loop command source for the first lesson."""

    def update(self, time: float, dt: float) -> Velocity: ...


class Simulator:
    def __init__(self, world, robot, *, program: VelocityProgram | None = None):
        if robot.world is not world:
            raise ValueError("The robot must belong to the simulator's world.")
        self.world = world
        self.robot = robot
        self.program = program
        self.steps = 0

    @property
    def dt(self):
        return self.world.physics.dt

    @property
    def time(self):
        return self.steps * self.dt

    def running(self):
        return self.world.physics.connected

    def step(self):
        self.world.physics.require_connection()
        if self.program is not None:
            command = self.program.update(self.time, self.dt)
            self.robot.set_velocity(command.vx, command.vy, command.omega, frame=command.frame)
        self.robot.apply_command()
        self.world.physics.step()
        if hasattr(self.robot, "after_step"):
            self.robot.after_step()
        self.steps += 1
        return self.robot.pose()
