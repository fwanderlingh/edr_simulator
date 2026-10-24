"""Instructor solutions for the separate Lesson 4 estimation/control exercises."""

from math import atan2, cos, hypot, isfinite, pi, sin


def wrap(angle):
    return (angle + pi) % (2 * pi) - pi


class ReferenceOdometry:
    def __init__(self, radius=0.12, width=0.42):
        self.radius, self.width = radius, width
        self.x = self.y = self.yaw = 0.0
        self.speed = self.yaw_rate = 0.0

    def update(self, left_delta, right_delta, dt):
        ds = self.radius * (right_delta + left_delta) / 2
        da = self.radius * (right_delta - left_delta) / self.width
        factor = sin(da / 2) / (da / 2) if abs(da) > 1e-12 else 1.0
        self.x += ds * factor * cos(self.yaw + da / 2)
        self.y += ds * factor * sin(self.yaw + da / 2)
        self.yaw += da
        self.speed, self.yaw_rate = ds / dt, da / dt


class ReferenceInertial:
    def __init__(self):
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = 0.0

    def update(self, gyro, ax_body, ay_body, dt, *, truth_yaw=None):
        self.yaw = self.yaw + gyro * dt if truth_yaw is None else truth_yaw
        ax = cos(self.yaw) * ax_body - sin(self.yaw) * ay_body
        ay = sin(self.yaw) * ax_body + cos(self.yaw) * ay_body
        self.x += self.vx * dt + ax * dt * dt / 2
        self.y += self.vy * dt + ay * dt * dt / 2
        self.vx += ax * dt
        self.vy += ay * dt


class ReferenceSectors:
    def summarize(self, angles, ranges, hits, max_range):
        if (not isfinite(max_range) or max_range <= 0 or not len(angles)
                or len(angles) != len(ranges) or len(ranges) != len(hits)):
            return None
        sectors = {"front": [], "left": [], "right": []}
        for angle, distance, hit in zip(angles, ranges, hits):
            if (not isfinite(angle) or abs(angle) > pi / 2 + 1e-10
                    or not isfinite(distance) or not 0 <= distance <= max_range):
                return None
            key = "front" if abs(angle) <= pi / 6 + 1e-10 else ("left" if angle > 0 else "right")
            sectors[key].append(min(distance, max_range) if hit else max_range)
        if any(not values for values in sectors.values()):
            return None
        return {key: min(values) for key, values in sectors.items()}


class ReferenceObstacleSupervisor:
    """A small detour exercise for one isolated box, not a complete planner."""

    def __init__(self):
        self.phase = "GO"
        self.turn_yaw = 0.0
        self.pass_start = (0.0, 0.0)
        self.enter, self.exit = 0.65, 0.8

    def command(self, pose, goal, sectors, age):
        if sectors is None or not isfinite(age) or age < 0 or age > 0.25:
            self.phase = "STALE"
            return 0.0, 0.0
        if self.phase == "STALE":
            # New valid data permit reevaluation; never reuse a stale turn.
            self.phase = "GO"
        dx, dy = goal[0] - pose.x, goal[1] - pose.y
        rho = hypot(dx, dy)
        if rho <= 0.05:
            self.phase = "DONE"
            return 0.0, 0.0
        if self.phase == "DONE":
            return 0.0, 0.0
        if self.phase == "GO" and sectors["front"] < self.enter:
            self.phase = "STOP"
            return 0.0, 0.0
        if self.phase == "STOP":
            side = 1 if sectors["left"] >= sectors["right"] else -1
            if max(sectors["left"], sectors["right"]) < self.exit:
                return 0.0, 0.0
            self.turn_yaw = pose.yaw + side * pi / 2
            self.phase = "TURN"
        if self.phase == "TURN":
            error = wrap(self.turn_yaw - pose.yaw)
            if abs(error) > 0.04:
                return 0.0, 1.5 * error
            if sectors["front"] < self.exit:
                self.phase = "STOP"
                return 0.0, 0.0
            self.pass_start = pose.x, pose.y
            self.phase = "PASS"
        if self.phase == "PASS":
            if sectors["front"] < self.enter:
                self.phase = "STOP"
                return 0.0, 0.0
            if hypot(pose.x - self.pass_start[0], pose.y - self.pass_start[1]) < 1.0:
                return 0.25, 1.5 * wrap(self.turn_yaw - pose.yaw)
            self.phase = "GO"
        error = wrap(atan2(dy, dx) - pose.yaw)
        # Rotate before advancing when the target lies far from the forward axis.
        return min(0.3, 0.8 * rho) * max(0.0, cos(error)), 2.0 * error
