"""Supplied controller state and wheel limits for the Lesson 3 UI exercises."""

import math


def controller_parameters():
    """Read the editable pose-control settings for student and reference models."""
    from . import control
    return dict(gain=control.GAIN, gain_bearing=control.GAIN_BEARING,
                gain_heading=control.GAIN_HEADING, eps_pos=control.EPS_POS,
                eps_angle=control.EPS_ANGLE, angle_in=control.ANGLE_IN,
                angle_out=control.ANGLE_OUT, wheel_max=control.WHEEL_MAX)


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class WheelController:
    """Supplied state and actuator limits; the exercise computes wheel references."""

    def __init__(self, goals=((1.0, 0.5, math.pi / 2), (1.0, 1.5, 0.0)), *,
                 gain=0.8, gain_bearing=2.0, gain_heading=1.5,
                 eps_pos=0.03, eps_angle=0.03, angle_in=0.08, angle_out=0.6,
                 wheel_max=5.0):
        self.goals = tuple(tuple(goal) for goal in goals)
        if not self.goals or any(len(g) != 3 or not all(math.isfinite(x) for x in g)
                                 for g in self.goals):
            raise ValueError("Supply at least one finite (x_m, y_m, yaw_rad) goal.")
        positive = (gain, gain_bearing, gain_heading, eps_pos, eps_angle, wheel_max)
        if not all(math.isfinite(x) and x > 0 for x in positive):
            raise ValueError("Gains, tolerances and wheel limit must be positive and finite.")
        if not 0 < angle_in < angle_out < math.pi / 2:
            raise ValueError("Require 0 < angle_in < angle_out < pi/2.")
        self.gain, self.gain_bearing, self.gain_heading = gain, gain_bearing, gain_heading
        self.eps_pos, self.eps_angle = eps_pos, eps_angle
        self.angle_in, self.angle_out = angle_in, angle_out
        self.wheel_max = wheel_max
        self.index, self.phase = 0, "TURN"
        self.nominal = (0.0, 0.0)
        self.scale = 1.0

    @property
    def done(self):
        return self.index == len(self.goals)

    def wheel_command(self, pose, r, L):
        raise NotImplementedError("Choose the student or reference controller.")

    def update(self, robot, time, dt):
        """Apply the exercise's wheel references, with a common actuator limit."""
        if self.done:
            references = (0.0, 0.0)
        else:
            references = tuple(self.wheel_command(
                robot.pose(), robot.motor_model.wheel_radius, robot.motor_model.track_width))
        if len(references) != 2 or not all(math.isfinite(x) for x in references):
            raise ValueError("wheel_command must return two finite wheel speeds in rad/s.")
        if self.done:
            self.index = len(self.goals)
        self.nominal = robot.motor_model.forward(*references)
        wl, wr = references
        peak = max(abs(wl), abs(wr))
        self.scale = min(1.0, self.wheel_max / peak) if peak else 1.0
        robot.set_wheel_velocities(self.scale * wl, self.scale * wr)


class PoseController(WheelController):
    """Supplied adapter for one goal; the student chooses TURN/DRIVE/ALIGN/DONE."""

    def __init__(self, goals=((1.0, 0.5, math.pi / 2),), **options):
        super().__init__(goals, **options)
        if len(self.goals) != 1:
            raise ValueError("The pose-control exercise needs exactly one goal.")

    @property
    def goal(self):
        return self.goals[0]

    @property
    def done(self):
        return self.phase == "DONE"


class WaypointController(WheelController):
    """Prepare one pose controller per target; students implement the sequencing."""

    def __init__(self, *args, **options):
        super().__init__(*args, **options)
        parameters = {name: getattr(self, name) for name in (
            "gain", "gain_bearing", "gain_heading", "eps_pos", "eps_angle",
            "angle_in", "angle_out", "wheel_max")}
        # self is the student or reference subclass instance, so Python finds
        # pose_controller_class on that subclass. Calling the class creates an
        # object; keep one per goal so its phase persists across updates.
        # (goal,) is a tuple containing one goal; **parameters passes settings
        # as keyword arguments to each pose controller's constructor.
        self.controllers = [self.pose_controller_class((goal,), **parameters)
                            for goal in self.goals]
