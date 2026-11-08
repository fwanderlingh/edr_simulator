"""Worked comparison for the first proportional-control exercise."""

import math

from robotics_sim.exercises.lesson03._proportional_runtime import ProportionalController


class ReferenceProportionalController(ProportionalController):
    def wheel_command(self, pose, r, L):
        gx, gy, _ = self.goal
        ex, ey = gx - pose.x, gy - pose.y
        rho = math.hypot(ex, ey)  # Distance sqrt(ex² + ey²), in metres.
        v = self.gain_pos * rho if rho > self.eps_pos else 0.0
        omega = 0.0
        # Convert (v, omega) using the complete inverse-kinematics equations.
        w_left = (v - L * omega / 2) / r
        w_right = (v + L * omega / 2) / r
        return w_left, w_right
