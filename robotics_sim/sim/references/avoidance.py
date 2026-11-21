"""Solutions for exercises/lesson04/avoidance.py: sectors and a local detour.

These methods use the same signatures and supplied state as the student methods.
Measurements are valid and all sectors are covered, as assumed in EDR.04.
"""

from math import cos, hypot, pi, sin

from robotics_sim.exercises.lesson04._runtime import ObstacleSupervisor, SectorSummary


def wrap(angle):
    return (angle + pi) % (2 * pi) - pi


class ReferenceSectors(SectorSummary):
    def summarize(self, angles, ranges, hits, max_range):
        # Start with no obstacle detected within the sensor range.
        front_distance = max_range
        left_distance = max_range
        right_distance = max_range

        front_angle_limit = pi / 6  # 30 degrees, expressed in radians.
        angle_tolerance = 1e-10    # Include boundary rays despite rounding.

        # Examine one ray at a time, using the same index in each array.
        for i in range(len(angles)):
            angle = angles[i]
            if hits[i]:
                distance = ranges[i]
            else:
                distance = max_range

            # Assign the ray to a sector and update that sector's minimum.
            if abs(angle) <= front_angle_limit + angle_tolerance:
                if distance < front_distance:
                    front_distance = distance
            elif angle > 0:
                if distance < left_distance:
                    left_distance = distance
            else: # angle < 0
                if distance < right_distance:
                    right_distance = distance

        result = {}
        result["front"] = front_distance
        result["left"] = left_distance
        result["right"] = right_distance
        return result


class ReferenceObstacleSupervisor(ObstacleSupervisor):
    """A small detour exercise for one isolated box, not a complete planner."""

    def command(self, pose, goal, sectors):
        gx, gy, gt = goal
        dx = gx - pose.x
        dy = gy - pose.y
        rho = hypot(dx, dy)
        heading_error = wrap(gt - pose.yaw)
        if rho <= self.eps_pos and abs(heading_error) <= self.eps_angle:
            self.phase = "DONE"
        if self.phase == "DONE":
            return 0.0, 0.0

        front_distance = sectors["front"]
        left_distance = sectors["left"]
        right_distance = sectors["right"]

        # GO: use the nominal controller until the front becomes obstructed.
        if self.phase == "GO" and front_distance < self.enter:
            self.phase = "STOP"
            return 0.0, 0.0
        if self.phase == "STOP":
            # The stop-only activity uses ReferenceRangeStop from rangefinder.py.
            # This method implements the avoid stage, just as students are asked.
            if left_distance >= right_distance:
                side = 1
                side_distance = left_distance
            else:
                side = -1
                side_distance = right_distance

            if side_distance < self.exit:
                return 0.0, 0.0
            # One fixed pose, expressed in the world, for the existing controller.
            heading = wrap(pose.yaw + side * self.turn_angle)
            self.detour_goal = (pose.x + self.bypass_distance * cos(heading),
                                pose.y + self.bypass_distance * sin(heading), heading)
            self.advance_allowed = False
            self.phase = "DETOUR"
        if self.phase == "DETOUR":
            # Reuse TURN/DRIVE/ALIGN and both arrival tolerances from Lesson 3.
            command = self.goal_command(pose, self.detour_goal)
            if self.goal_controller_done:
                self.detour_goal = None
                self.phase = "GO"
                if front_distance < self.enter:
                    self.phase = "STOP"
                    return 0.0, 0.0
                # The different target automatically restarts the pose controller.
                return self.goal_command(pose, goal)

            # Hysteresis gates forward motion. Allow the initial turn even while
            # the original obstacle remains in front; the side was checked above.
            if front_distance < self.enter:
                self.advance_allowed = False
            elif front_distance >= self.exit:
                self.advance_allowed = True
            if command[0] > 0 and not self.advance_allowed:
                return 0.0, 0.0
            return command
        return self.goal_command(pose, goal)
