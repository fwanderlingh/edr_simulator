"""First feedback exercise: reach a goal straight ahead using wheel references.

Complete StudentProportionalController.wheel_command, save and click Reload exercise.
Choose Lesson 3 > Reach a goal: proportional control > student.
"""

import math

from ._proportional_runtime import ProportionalController


GOAL = (1.0, 0.0, 0.0)       # x [m], y [m], heading [rad]; goal straight ahead
GAIN = 0.4                   # 1/s
EPS_POS = 0.03               # m


class StudentProportionalController(ProportionalController):
    def wheel_command(self, pose, r, L):
        """Return (w_left, w_right) in rad/s for the goal straight ahead.

        Available data: self.goal (x,y,θ), self.gain, self.eps_pos and pose (x,y,θ).
        The robot supplies r (wheel radius, m) and L (track width, m).

        1. Compute ex, ey and the full position error rho = math.hypot(ex, ey).
        2. Use gain * rho as forward speed, or zero within position tolerance.
        3. Set omega = 0.0 and calculate both wheel references with the complete
           inverse-kinematics equations using r and L; do not simplify to v/r.
        4. Return the two values; the supplied loop applies them to the robot.
        """
        # TODO: position error -> forward speed -> left/right wheel references.
        raise NotImplementedError(
            "Complete StudentProportionalController.wheel_command in "
            "robotics_sim/exercises/lesson03/proportional.py."
        )
