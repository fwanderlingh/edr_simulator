"""Provided simulator adapter for the introductory proportional exercise."""

import math

from ._control_runtime import WheelController


class ProportionalController(WheelController):
    """Lifecycle, validation and actuator limits; students edit proportional.py."""

    def __init__(self, goals=((1.0, 0.0, 0.0),), *, gain=0.4, eps_pos=0.03,
                 wheel_max=5.0):
        super().__init__(goals, gain=gain, eps_pos=eps_pos, wheel_max=wheel_max)
        if len(self.goals) != 1 or self.goals[0][0] <= 0 or self.goals[0][1:] != (0, 0):
            raise ValueError("The proportional exercise needs one goal on positive x with y=heading=0.")
        self.gain = gain
        self.phase = "DRIVE"

    @property
    def goal(self):
        return self.goals[0]

    def wheel_command(self, pose, r, L):
        raise NotImplementedError("Choose the student or reference proportional controller.")

    def update(self, robot, time, dt):
        """Apply raw wheel references, with no automatic inverse conversion."""
        if self.done:
            robot.set_wheel_velocities(0.0, 0.0)
            return
        pose = robot.pose()
        references = tuple(self.wheel_command(
            pose, robot.motor_model.wheel_radius, robot.motor_model.track_width))
        if len(references) != 2 or not all(math.isfinite(value) for value in references):
            raise ValueError("wheel_command must return two finite wheel speeds in rad/s.")
        self.nominal = robot.motor_model.forward(*references)
        peak = max(abs(value) for value in references)
        self.scale = min(1.0, self.wheel_max / peak) if peak else 1.0
        gx, gy, _ = self.goal
        if math.hypot(gx - pose.x, gy - pose.y) <= self.eps_pos:
            self.index, self.phase = 1, "DONE"
            robot.set_wheel_velocities(0.0, 0.0)
        else:
            robot.set_wheel_velocities(*(self.scale * value for value in references))
