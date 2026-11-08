"""Reach one target pose: turn, drive, then align the final heading.

Complete StudentPoseController.wheel_command, save and click Reload exercise.
Choose Lesson 3 > Reach a goal: pose control > student.
Reference: robotics_sim/sim/references/control.py (ReferencePoseController).
The next exercise, waypoint.py, sequences this controller over multiple goals.
"""

import math

from ._control_runtime import PoseController, wrap


GOAL = (1.0, 0.5, math.pi / 2)  # x [m], y [m], heading [rad]
GAIN = 0.8                  # 1/s; forward speed from position error
GAIN_BEARING = 2.0          # 1/s; turn toward the goal
GAIN_HEADING = 1.5          # 1/s; align the final heading
EPS_POS = 0.03              # m
EPS_ANGLE = 0.03            # rad
ANGLE_IN = 0.08             # rad; enter DRIVE
ANGLE_OUT = 0.6             # rad; return to TURN
WHEEL_MAX = 5.0             # rad/s; supplied common wheel-speed limit


class StudentPoseController(PoseController):
    def wheel_command(self, pose, r, L):
        """Return (w_left, w_right) in rad/s for self.goal (x, y, heading).

        The robot supplies pose (x, y, yaw), wheel radius r and track width L.
        Read self.gain, self.gain_bearing, self.gain_heading, self.eps_pos,
        self.eps_angle, self.angle_in and self.angle_out. Initially phase is TURN.

        1. If self.done, return two zeros. Otherwise compute ex, ey and rho.
        2. If rho <= eps_pos, choose ALIGN before computing any bearing.
           Use v = 0 and omega = gain_heading * wrap(goal_heading - pose.yaw).
           Within eps_angle, set self.phase = "DONE" and return two zeros.
        3. Otherwise, if leaving ALIGN, reset self.phase to TURN. Compute
           alpha = wrap(math.atan2(ey, ex) - pose.yaw).
           TURN -> DRIVE when abs(alpha) <= angle_in;
           DRIVE -> TURN when abs(alpha) > angle_out; otherwise keep the phase.
        4. Use the updated phase: v = gain * rho in DRIVE, v = 0 in TURN.
           Both use omega = gain_bearing * alpha.
        5. Convert (v, omega) with both complete inverse-kinematics equations
           using r and L, as in proportional.py. Return the wheel references;
           the supplied loop applies the common speed limit.
        """
        
        # TODO: errors -> phase -> (v, omega) -> left/right wheel references.
        raise NotImplementedError(
            "Complete StudentPoseController.wheel_command in "
            "robotics_sim/exercises/lesson03/control.py."
        )
