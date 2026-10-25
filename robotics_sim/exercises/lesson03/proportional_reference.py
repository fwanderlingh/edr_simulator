"""Worked comparison for the first proportional-control exercise."""

from .proportional import ProportionalController


class ReferenceProportionalController(ProportionalController):
    def wheel_command(self, pose, wheel_radius):
        error = self.goals[0][0] - pose.x
        speed = self.gain * error if abs(error) > self.eps_p else 0.0
        return speed / wheel_radius, speed / wheel_radius
