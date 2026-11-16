"""Solution for exercises/lesson04/rangefinder.py: GO/STOP/DONE supervisor.

Use the same command signature and supplied state as StudentRangeStop.
Measurements are assumed valid; nominal motion reuses Lesson 3.
"""

from math import hypot

from robotics_sim.exercises.lesson04._runtime import ObstacleSupervisor


class ReferenceRangeStop(ObstacleSupervisor):
    """Front distance interlock around the existing Lesson 3 controller."""

    def command(self, pose, goal, distance, hit, max_range):
        # A miss represents the observed range limit, not a surface at that limit.
        d = distance if hit else max_range
        rho = hypot(goal[0] - pose.x, goal[1] - pose.y)
        # Goal completion has priority and DONE remains terminal.
        if rho <= self.eps_pos:
            self.phase = "DONE"

        if self.phase == "DONE":
            return 0.0, 0.0
        # First select the state, then calculate its command (EDR.04 FSM).
        if self.phase == "GO" and d < self.enter:
            self.phase = "STOP"
        elif self.phase == "STOP" and d >= self.exit:
            self.reset_goal_controller()
            self.phase = "GO"

        if self.phase == "STOP":
            return 0.0, 0.0
        return self.goal_command(pose, goal)
