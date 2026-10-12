"""Fixed-step loop: command all robots, then advance their shared world once."""

from types import MappingProxyType
from typing import Callable, Protocol

from ..types import Velocity


class VelocityProgram(Protocol):
    """Legacy single-robot command source, retained for existing API callers."""

    def update(self, time: float, dt: float) -> Velocity: ...


RobotUpdate = Callable[[object, float, float], None]


class Simulator:
    def __init__(self, world, robot, *, update: RobotUpdate | None = None,
                 program: VelocityProgram | None = None):
        if update is not None and program is not None:
            raise ValueError("Use either update or the legacy program, not both.")
        self.world = world
        self.robot = robot  # Primary robot for the current UI and CSV format.
        self.program = program
        self._robots = {}
        self._updates = {}
        self.steps = 0
        self.register_robot("robot", robot, update=update)

    @property
    def robots(self):
        """Read-only named registry; use register_robot to add a participant."""
        return MappingProxyType(self._robots)

    def register_robot(self, name, robot, *, update: RobotUpdate | None = None):
        """Register a robot in the same world before advancing the simulation."""
        if self.steps:
            raise ValueError("Register robots before the first simulation step.")
        if not isinstance(name, str) or not name.strip() or name in self._robots:
            raise ValueError("Each robot needs a unique, nonempty name.")
        if robot.world is not self.world:
            raise ValueError("The robot must belong to the simulator's world.")
        if any(existing.body_id == robot.body_id for existing in self._robots.values()):
            raise ValueError("This robot body is already registered.")
        if update is not None and not callable(update):
            raise TypeError("The update callback must be callable.")
        self._robots[name] = robot
        self._updates[name] = update

    @property
    def dt(self):
        return self.world.physics.dt

    @property
    def time(self):
        return self.steps * self.dt

    def running(self):
        return self.world.physics.connected

    def stop(self):
        """Clear held commands for every registered robot."""
        for robot in self._robots.values():
            robot.set_velocity()

    def step(self):
        self.world.physics.require_connection()
        if self.program is not None:
            command = self.program.update(self.time, self.dt)
            self.robot.set_velocity(command.vx, command.vy, command.omega, frame=command.frame)
        for name, robot in self._robots.items():
            update = self._updates[name]
            if update is not None:
                update(robot, self.time, self.dt)
        for robot in self._robots.values():
            robot.apply_command()
        self.world.physics.step()
        for robot in self._robots.values():
            if hasattr(robot, "after_step"):
                robot.after_step()
        self.steps += 1
        return self.robot.pose()
