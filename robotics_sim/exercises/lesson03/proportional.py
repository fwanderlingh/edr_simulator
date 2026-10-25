"""First feedback exercise: reach a goal straight ahead using wheel references.

Complete StudentProportionalController.wheel_command, save and restart the app.
Choose Lesson 3 > Reach a goal: proportional control > student.
"""

import math

from .control import WaypointController


class ProportionalController(WaypointController):
    """Supplied lifecycle and actuator limits for the one-dimensional exercise."""

    def __init__(self, goals=((1.0, 0.0, 0.0),), *, gain=0.4, eps_p=0.03,
                 wheel_max=5.0):
        super().__init__(goals, kp=gain, eps_p=eps_p, wheel_max=wheel_max)
        if len(self.goals) != 1 or self.goals[0][0] <= 0 or self.goals[0][1:] != (0, 0):
            raise ValueError("The proportional exercise needs one goal on positive x with y=heading=0.")
        self.gain = gain
        self.phase = "DRIVE"

    def wheel_command(self, pose, wheel_radius):
        raise NotImplementedError("Choose the student or reference proportional controller.")

    def update(self, robot, time, dt):
        """Apply the exercise's wheel references; never convert a body command here."""
        if self.done:
            robot.set_wheel_velocities(0.0, 0.0)
            return
        pose = robot.pose()
        references = tuple(self.wheel_command(pose, robot.motor_model.wheel_radius))
        if len(references) != 2 or not all(math.isfinite(value) for value in references):
            raise ValueError("wheel_command must return two finite wheel speeds in rad/s.")
        self.nominal = robot.motor_model.forward(*references)
        peak = max(abs(value) for value in references)
        self.scale = min(1.0, self.wheel_max / peak) if peak else 1.0
        if math.hypot(self.goals[0][0] - pose.x, pose.y) <= self.eps_p:
            self.index, self.phase = 1, "DONE"
            robot.set_wheel_velocities(0.0, 0.0)
        else:
            robot.set_wheel_velocities(*(self.scale * value for value in references))


class StudentProportionalController(ProportionalController):
    def wheel_command(self, pose, wheel_radius):
        """Return (w_left, w_right) in rad/s for the goal straight ahead.

        Available data: self.goals[0][0], self.gain, self.eps_p and pose.x.
        1. Compute the signed longitudinal error from the current position.
        2. Use a proportional forward speed, or zero within position tolerance.
        3. With zero yaw rate, calculate both wheel references yourself.
        4. Return the two values; the supplied loop applies them to the robot.
        """
        # TODO: position error -> forward speed -> left/right wheel references.
        raise NotImplementedError(
            "Complete StudentProportionalController.wheel_command in "
            "robotics_sim/exercises/lesson03/proportional.py."
        )
