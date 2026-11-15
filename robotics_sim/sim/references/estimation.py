"""Solutions for exercises/lesson04/estimation.py: odometry and inertial integration."""

from math import cos, sin

from robotics_sim.exercises.lesson04._runtime import InertialState, OdometryState


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
