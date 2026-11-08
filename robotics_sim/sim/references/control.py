"""Solution for exercises/lesson03/control.py: reach one target pose."""

import math

from robotics_sim.exercises.lesson03._control_runtime import PoseController, wrap


class ReferencePoseController(PoseController):
    def wheel_command(self, pose, r, L):
        self.scale = 1.0
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
            goal_angle = math.atan2(gy - pose.y, gx - pose.x)
            alpha = wrap(goal_angle - pose.yaw)
            if self.phase == "TURN" and abs(alpha) <= self.angle_in:
                self.phase = "DRIVE"
            elif self.phase == "DRIVE" and abs(alpha) > self.angle_out:
                self.phase = "TURN"
            if self.phase == "DRIVE":
                v = self.gain_pos * rho
            else:
                v = 0.0
            omega = self.gain_bearing * alpha
        w_left = (v - L * omega / 2) / r
        w_right = (v + L * omega / 2) / r
        # Scale both wheels together to preserve the commanded curvature.
        peak = max(abs(w_left), abs(w_right))
        self.scale = min(1.0, self.wheel_max / peak) if peak else 1.0
        return self.scale * w_left, self.scale * w_right
