"""Solutions for exercises/lesson04/avoidance.py: sectors and a local detour.

These methods use the same signatures and supplied state as the student methods.
Measurements are valid and all sectors are covered, as assumed in EDR.04.
"""

from math import hypot, pi

from robotics_sim.exercises.lesson04._runtime import ObstacleSupervisor, SectorSummary


def wrap(angle):
    return (angle + pi) % (2 * pi) - pi


class ReferenceSectors(SectorSummary):
    def summarize(self, angles, ranges, hits, max_range):
        # The lesson assumes valid measurements and coverage of all sectors.
        sectors = {"front": [], "left": [], "right": []}
        for angle, distance, hit in zip(angles, ranges, hits):
            key = "front" if abs(angle) <= pi / 6 + 1e-10 else ("left" if angle > 0 else "right")
            sectors[key].append(min(distance, max_range) if hit else max_range)
        return {key: min(values) for key, values in sectors.items()}


class ReferenceObstacleSupervisor(ObstacleSupervisor):
    """A small detour exercise for one isolated box, not a complete planner."""

    def command(self, pose, goal, sectors):
        dx, dy = goal[0] - pose.x, goal[1] - pose.y
        rho = hypot(dx, dy)
        if rho <= self.eps_pos:
            self.phase = "DONE"
            return 0.0, 0.0
        if self.phase == "DONE":
            return 0.0, 0.0
        # GO: use the nominal controller until the front becomes obstructed.
        if self.phase == "GO" and sectors["front"] < self.enter:
            self.phase = "STOP"
            return 0.0, 0.0
        if self.phase == "STOP":
            # The stop-only activity uses ReferenceRangeStop from rangefinder.py.
            # This method implements the avoid stage, just as students are asked.
            side = 1 if sectors["left"] >= sectors["right"] else -1
            if max(sectors["left"], sectors["right"]) < self.exit:
                return 0.0, 0.0
            self.turn_yaw = pose.yaw + side * self.turn_angle
            self.phase = "TURN"
        if self.phase == "TURN":
            # Keep the chosen target yaw until the rotation is complete.
            error = wrap(self.turn_yaw - pose.yaw)
            if abs(error) > self.turn_tolerance:
                return 0.0, self.turn_gain * error
            if sectors["front"] < self.exit:
                self.phase = "STOP"
                return 0.0, 0.0
            self.pass_start = pose.x, pose.y
            self.phase = "PASS"
        if self.phase == "PASS":
            # Measure progress from the start of the bypass, while checking front.
            if sectors["front"] < self.enter:
                self.phase = "STOP"
                return 0.0, 0.0
            if hypot(pose.x - self.pass_start[0], pose.y - self.pass_start[1]) < self.bypass_distance:
                return self.bypass_speed, self.turn_gain * wrap(self.turn_yaw - pose.yaw)
            self.phase = "GO"
            self.reset_goal_controller()
        return self.goal_command(pose, goal)
