"""Instructor comparison models for Lesson 4 estimation and supervision."""

from math import cos, hypot, isfinite, pi, sin

import math

from robotics_sim.exercises.lesson04._runtime import (
    InertialState, ObstacleSupervisor, OdometryState, SectorSummary,
)


def wrap(angle):
    return (angle + pi) % (2 * pi) - pi


class ReferenceOdometry(OdometryState):
    def __init__(self, radius=0.12, width=0.42, *, integration="arc"):
        super().__init__(radius, width)
        if integration not in ("arc", "midpoint"):
            raise ValueError("Choose arc or midpoint odometry.")
        self.integration = integration

    def update(self, left_delta, right_delta, dt):
        ds = self.radius * (right_delta + left_delta) / 2
        da = self.radius * (right_delta - left_delta) / self.width
        factor = sin(da / 2) / (da / 2) if self.integration == "arc" and abs(da) > 1e-12 else 1.0
        self.x += ds * factor * cos(self.yaw + da / 2)
        self.y += ds * factor * sin(self.yaw + da / 2)
        self.yaw += da
        self.speed, self.yaw_rate = ds / dt, da / dt


class ReferenceInertial(InertialState):
    def update(self, gyro, ax_body, ay_body, dt, *, truth_yaw=None):
        self.yaw = self.yaw + gyro * dt if truth_yaw is None else truth_yaw
        ax = cos(self.yaw) * ax_body - sin(self.yaw) * ay_body
        ay = sin(self.yaw) * ax_body + cos(self.yaw) * ay_body
        self.x += self.vx * dt + ax * dt * dt / 2
        self.y += self.vy * dt + ay * dt * dt / 2
        self.vx += ax * dt
        self.vy += ay * dt


class ReferenceSectors(SectorSummary):
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


class ReferenceRangeStop(ObstacleSupervisor):
    """Front distance interlock around the existing Lesson 3 controller."""

    def command(self, pose, goal, distance, hit, max_range, age):
        if (not all(isfinite(x) for x in (distance, max_range, age))
                or max_range <= 0 or not 0 <= distance <= max_range
                or age < 0 or age > self.max_scan_age):
            self.phase = "STALE"
            self.reset_goal_controller()
            return 0.0, 0.0
        front = distance if hit else max_range
        if hypot(goal[0] - pose.x, goal[1] - pose.y) <= self.eps_pos:
            self.phase = "DONE"
            return 0.0, 0.0
        if self.phase == "DONE":
            return 0.0, 0.0
        if front < self.enter or self.phase == "STOP" and front < self.exit:
            self.phase = "STOP"
            return 0.0, 0.0
        if self.phase in ("STOP", "STALE"):
            self.reset_goal_controller()
        self.phase = "GO"
        return self.goal_command(pose, goal)


class ReferenceObstacleSupervisor(ObstacleSupervisor):
    """A small detour exercise for one isolated box, not a complete planner."""

    def command(self, pose, goal, sectors, age):
        if sectors is None or not isfinite(age) or age < 0 or age > self.max_scan_age:
            self.phase = "STALE"
            self.reset_goal_controller()
            return 0.0, 0.0
        if self.phase == "STALE":
            # New valid data permit reevaluation; never reuse a stale turn.
            self.phase = "GO"
        dx, dy = goal[0] - pose.x, goal[1] - pose.y
        rho = hypot(dx, dy)
        if rho <= self.eps_pos:
            self.phase = "DONE"
            return 0.0, 0.0
        if self.phase == "DONE":
            return 0.0, 0.0
        if self.phase == "GO" and sectors["front"] < self.enter:
            self.phase = "STOP"
            return 0.0, 0.0
        if self.phase == "STOP":
            if self.stage == "stop":
                if sectors["front"] < self.exit:
                    return 0.0, 0.0
                self.phase = "GO"
                self.reset_goal_controller()
                return self.goal_command(pose, goal)
            side = 1 if sectors["left"] >= sectors["right"] else -1
            if max(sectors["left"], sectors["right"]) < self.exit:
                return 0.0, 0.0
            self.turn_yaw = pose.yaw + side * self.turn_angle
            self.phase = "TURN"
        if self.phase == "TURN":
            error = wrap(self.turn_yaw - pose.yaw)
            if abs(error) > self.turn_tolerance:
                return 0.0, self.turn_gain * error
            if sectors["front"] < self.exit:
                self.phase = "STOP"
                return 0.0, 0.0
            self.pass_start = pose.x, pose.y
            self.phase = "PASS"
        if self.phase == "PASS":
            if sectors["front"] < self.enter:
                self.phase = "STOP"
                return 0.0, 0.0
            if hypot(pose.x - self.pass_start[0], pose.y - self.pass_start[1]) < self.bypass_distance:
                return self.bypass_speed, self.turn_gain * wrap(self.turn_yaw - pose.yaw)
            self.phase = "GO"
            self.reset_goal_controller()
        return self.goal_command(pose, goal)
