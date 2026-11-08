"""Solution for exercises/lesson03/control.py: reach one target pose."""

import math

from robotics_sim.exercises.lesson03._control_runtime import PoseController, wrap


class ReferencePoseController(PoseController):
    def wheel_command(self, pose, r, L):
        if self.done:
            return 0.0, 0.0
        gx, gy, gt = self.goal
        rho = math.hypot(gx - pose.x, gy - pose.y)
        if rho <= self.eps_pos:
            self.phase = "ALIGN"
        elif self.phase == "ALIGN":
            self.phase = "TURN"
        if self.phase == "ALIGN":
            align_error = wrap(gt - pose.yaw)
            if abs(align_error) <= self.eps_angle:
                self.phase = "DONE"
                return 0.0, 0.0
            v, omega = 0.0, self.gain_heading * align_error
        else:
            alpha = wrap(math.atan2(gy - pose.y, gx - pose.x) - pose.yaw)
            if self.phase == "TURN" and abs(alpha) <= self.angle_in:
                self.phase = "DRIVE"
            elif self.phase == "DRIVE" and abs(alpha) > self.angle_out:
                self.phase = "TURN"
            v = self.gain_pos * rho if self.phase == "DRIVE" else 0.0
            omega = self.gain_bearing * alpha
        w_left = (v - L * omega / 2) / r
        w_right = (v + L * omega / 2) / r
        return w_left, w_right
