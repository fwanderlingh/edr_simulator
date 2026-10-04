"""Ideal differential drive: x forward, y left, positive yaw counterclockwise."""

import math

import pybullet as bullet

from .mobile_base import Robot
from ..types import Pose


class ReferenceDifferentialDrive:
    def __init__(self, wheel_radius=0.12, track_width=0.42):
        if not all(math.isfinite(v) and v > 0 for v in (wheel_radius, track_width)):
            raise ValueError("Wheel radius and track width must be finite and positive.")
        self.wheel_radius, self.track_width = wheel_radius, track_width

    def forward(self, w_left, w_right):
        """Wheel angular speeds (rad/s) -> forward speed (m/s), yaw rate (rad/s)."""
        return (self.wheel_radius * (w_right + w_left) / 2,
                self.wheel_radius * (w_right - w_left) / self.track_width)

    def inverse(self, v, omega):
        """Body twist -> left and right wheel angular speeds (rad/s)."""
        return ((v - omega * self.track_width / 2) / self.wheel_radius,
                (v + omega * self.track_width / 2) / self.wheel_radius)


class DifferentialDriveRobot(Robot):
    def __init__(self, world, pose=Pose(), *, motor_model=None):
        super().__init__(world, pose)
        self.motor_model = motor_model or ReferenceDifferentialDrive()
        self.wheel_velocities = (0.0, 0.0)
        client = world.physics.client
        # Decorative wheels follow the chassis; contact dynamics are intentionally
        # absent in this ideal kinematics lesson. Cylinder axes are local z.
        self.wheels = []
        for color in ((0.15, 0.22, 0.32, 1), (0.3, 0.35, 0.42, 1)):
            visual = bullet.createVisualShape(bullet.GEOM_CYLINDER,
                radius=self.motor_model.wheel_radius, length=0.06,
                rgbaColor=color, physicsClientId=client)
            self.wheels.append(bullet.createMultiBody(baseMass=0,
                baseVisualShapeIndex=visual, baseCollisionShapeIndex=-1,
                physicsClientId=client))
        self.after_step()

    def set_wheel_velocities(self, w_left, w_right):
        if not all(math.isfinite(v) for v in (w_left, w_right)):
            raise ValueError("Wheel speeds must be finite.")
        v, omega = self.motor_model.forward(w_left, w_right)
        if not all(math.isfinite(value) for value in (v, omega)):
            raise ValueError("Kinematics must return finite velocities.")
        self.wheel_velocities = (w_left, w_right)
        super().set_velocity(vx=v, omega=omega, frame="body")

    def set_twist(self, v, omega):
        self.set_wheel_velocities(*self.motor_model.inverse(v, omega))

    def set_velocity(self, vx=0.0, vy=0.0, omega=0.0, *, frame="body"):
        if vy != 0 or (frame != "body" and (vx != 0 or omega != 0)):
            raise ValueError("Differential drive accepts forward body velocity and yaw rate only.")
        self.set_twist(vx, omega)

    def after_step(self):
        pose = self.pose()
        client = self.world.physics.client
        orientation = bullet.getQuaternionFromEuler((math.pi / 2, 0, pose.yaw))
        for body, side in zip(self.wheels, (1, -1)):
            offset = side * self.motor_model.track_width / 2
            bullet.resetBasePositionAndOrientation(body,
                (pose.x - math.sin(pose.yaw) * offset,
                 pose.y + math.cos(pose.yaw) * offset, pose.z),
                orientation, physicsClientId=client)
