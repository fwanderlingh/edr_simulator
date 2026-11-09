"""Reach one target pose: turn, drive, then align the final heading.

Complete StudentPoseController.wheel_command, save and click Reload exercise.
Choose Lesson 3 > Reach a goal: pose control > student.
Reference: robotics_sim/sim/references/control.py (ReferencePoseController).
The next exercise, waypoint.py, sequences this controller over multiple goals.
"""

import math

from ._control_runtime import PoseController, wrap


GOAL = (1.0, 0.5, math.pi / 2)  # x [m], y [m], heading [rad]
GAIN_POS = 0.8              # k_rho [1/s]; forward speed from position error
GAIN_BEARING = 2.0          # 1/s; turn toward the goal
GAIN_HEADING = 1.5          # 1/s; align the final heading
EPS_POS = 0.03              # m
EPS_ANGLE = 0.03            # rad
ANGLE_IN = 0.08             # rad; enter DRIVE
ANGLE_OUT = 0.6             # rad; return to TURN
WHEEL_MAX = 5.0             # rad/s; apply this limit in wheel_command


class StudentPoseController(PoseController):
    def wheel_command(self, pose, r, L):
        """Return (w_left, w_right) in rad/s for self.goal (x, y, heading).

        The robot supplies pose (x, y, yaw), wheel radius r and track width L.
        Read self.gain_pos, self.gain_bearing, self.gain_heading, self.eps_pos,
        self.eps_angle, self.angle_in and self.angle_out. Initially phase is TURN.

        self.done is an inherited, read-only @property: Python evaluates
        self.phase == "DONE" whenever you read it, without parentheses.
        It is not a separate flag to update. Set self.phase = "DONE" when
        both pose tolerances are met; self.done then becomes True automatically.

        Apply saturation before returning the wheel references: compute
        peak = max(abs(w_left), abs(w_right)), then a common factor
        min(1.0, self.wheel_max / peak), or 1.0 when peak is zero.
        Record the factor in self.scale (also 1.0 when returning two zeros),
        and return both wheel speeds multiplied by it. Do not clip each wheel
        separately. The simulator applies your returned speeds unchanged;
        the wheel-speed check fails if your saturation is missing or incorrect.
        """
        if self.phase == "DONE":
            return 0.0, 0.0
        gx, gy, gt = self.goal
        rho = math.hypot(gx - pose.x, gy - pose.y)
        if rho <= self.eps_pos:
            self.phase = "ALIGN"
        elif self.phase == "ALIGN":
            self.phase = "TURN"
        if self.phase == "ALIGN":
            align_error = wrap(gt - pose.yaw)
            if (align_error) <= self.eps_angle:
                self.phase = "DONE"
                return 0.0, 0.0
            v, omega = 0.0, self.gain_heading * align_error
        else:
            goal_angle = math.atan2(gy - pose.y, gx -pose.x)
            alpha = wrap(goal_angle - pose.yaw)
            if self.phase == "TURN" and abs(alpha <= self.angle_in):
                self.phase = "DRIVE"
            elif self.phase == "DRIVE" and abs(alpha > self.angle_out):
                self.phase = "TURN"
            if self.phase == "DRIVE":
                v = self.gain_pos * rho
            else:
                v = 0.0;
            omega = self.gain_bearing * alpha
                
        w_left = (v - L * omega/2) / r     
        w_right = (v + L * omega/2) / r        
        
        
        w_appl_L = w_left
        w_appl_R = w_right
        return w_appl_L, w_appl_R
