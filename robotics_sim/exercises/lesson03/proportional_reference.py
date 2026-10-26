"""Worked comparison for the first proportional-control exercise."""

import math

from ._proportional_runtime import ProportionalController


class ReferenceProportionalController(ProportionalController):
    def wheel_command(self, pose, wheel_radius):
        gx, gy, _ = self.goal
        ex, ey = gx - pose.x, gy - pose.y
        rho = math.hypot(ex, ey)
        speed = self.gain * rho if rho > self.eps_p else 0.0
        return speed / wheel_radius, speed / wheel_radius
