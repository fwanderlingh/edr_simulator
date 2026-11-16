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
            self.turn_yaw = pose.yaw + side * self.turn_angle
            self.phase = "TURN"
        if self.phase == "TURN":
            # Keep the chosen target yaw until the rotation is complete.
            error = wrap(self.turn_yaw - pose.yaw)
            if abs(error) > self.turn_tolerance:
                omega = self.turn_gain * error
                return 0.0, omega
            if front_distance < self.exit:
                self.phase = "STOP"
                return 0.0, 0.0
            self.pass_start = (pose.x, pose.y)
            self.phase = "PASS"
        if self.phase == "PASS":
            # Measure progress from the start of the bypass, while checking front.
            if front_distance < self.enter:
                self.phase = "STOP"
                return 0.0, 0.0

            dx = pose.x - self.pass_start[0]
            dy = pose.y - self.pass_start[1]
            distance_from_start = hypot(dx, dy)
            if distance_from_start < self.bypass_distance:
                error = wrap(self.turn_yaw - pose.yaw)
                v = self.bypass_speed
                omega = self.turn_gain * error
                return v, omega

            self.phase = "GO"
            self.reset_goal_controller()
        return self.goal_command(pose, goal)
