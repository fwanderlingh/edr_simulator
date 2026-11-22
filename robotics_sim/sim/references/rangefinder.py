"""Solution for exercises/lesson04/rangefinder.py: GO/STOP/DONE supervisor.

Use the same command signature and supplied state as StudentRangeStop.
Measurements are assumed valid; nominal motion reuses Lesson 3.
"""

from math import hypot

from robotics_sim.exercises.lesson04._runtime import ObstacleSupervisor
from robotics_sim.exercises.lesson03._control_runtime import wrap


class ReferenceRangeStop(ObstacleSupervisor):
    """Front distance interlock around the existing Lesson 3 controller."""

    def command(self, pose, goal, distance, hit, max_range):
        # A miss represents the observed range limit, not a surface at that limit.
        d = distance if hit else max_range
        gx, gy, gt = goal
        rho = hypot(gx - pose.x, gy - pose.y)
        heading_error = wrap(gt - pose.yaw)
        # Goal completion has priority and DONE remains terminal.
        if rho <= self.eps_pos and abs(heading_error) <= self.eps_angle:
            self.phase = "DONE"

        if self.phase == "DONE":
            return 0.0, 0.0
        # First select the state, then calculate its command (EDR.04 FSM).
        if self.phase == "GO" and d < self.d_in:
            self.phase = "STOP"
        elif self.phase == "STOP" and d >= self.d_out:
            self.reset_goal_controller()
            self.phase = "GO"

        if self.phase == "STOP":
            return 0.0, 0.0
        return self.goal_command(pose, goal)
