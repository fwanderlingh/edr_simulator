"""Instructor comparison for the Lesson 3 controller. Attempt the exercise first."""

import math

from .control import WaypointController, wrap


class ReferenceWaypointController(WaypointController):
    def nominal_command(self, pose):
        if self.done:
            return 0.0, 0.0
        gx, gy, gt = self.goals[self.index]
        rho = math.hypot(gx - pose.x, gy - pose.y)
        if rho <= self.eps_p:
            self.phase = "ALIGN"
        elif self.phase == "ALIGN":
            self.phase = "TURN"
        if self.phase == "ALIGN":
            error = wrap(gt - pose.yaw)
            if abs(error) <= self.eps_a:
                self.index += 1
                self.phase = "DONE" if self.done else "TURN"
                return 0.0, 0.0
            return 0.0, self.kt * error
        alpha = wrap(math.atan2(gy - pose.y, gx - pose.x) - pose.yaw)
        if self.phase == "TURN" and abs(alpha) <= self.angle_in:
            self.phase = "DRIVE"
        elif self.phase == "DRIVE" and abs(alpha) > self.angle_out:
            self.phase = "TURN"
        return (self.kp * rho if self.phase == "DRIVE" else 0.0), self.ka * alpha
