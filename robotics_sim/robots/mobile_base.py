"""Lesson 1 ideal velocity-driven body, before differential-drive kinematics."""

from math import cos, sin

import pybullet as bullet

from ..geometry.rotations import matrix_to_rpy, quaternion_to_matrix
from ..types import Pose, Velocity


class Robot:
    def __init__(self, world, pose=Pose()):
        self.world = world
        self.body_id = world.add_box(pose, size=(0.4, 0.3, 0.3), color=(0.05, 0.5, 0.5, 1), mass=1)
        self.command = Velocity()
        bullet.changeDynamics(self.body_id, -1, linearDamping=0, angularDamping=0,
                              lateralFriction=0, physicsClientId=world.physics.client)

    def set_velocity(self, vx=0.0, vy=0.0, omega=0.0, *, frame="world"):
        """Hold a command until changed; apply it at each simulation step."""
        self.command = Velocity(vx, vy, omega, frame)

    def pose(self):
        """Read physics ground truth, never an internally integrated estimate."""
        self.world.physics.require_connection()
        position, quaternion = bullet.getBasePositionAndOrientation(
            self.body_id, physicsClientId=self.world.physics.client)
        roll, pitch, yaw = matrix_to_rpy(quaternion_to_matrix(quaternion))
        return Pose(*position, roll, pitch, yaw)

    def apply_command(self):
        self.world.physics.require_connection()
        vx, vy = self.command.vx, self.command.vy
        if self.command.frame == "body":
            yaw = self.pose().yaw
            vx, vy = cos(yaw)*vx - sin(yaw)*vy, sin(yaw)*vx + cos(yaw)*vy
        bullet.resetBaseVelocity(self.body_id, linearVelocity=(vx, vy, 0),
                                 angularVelocity=(0, 0, self.command.omega),
                                 physicsClientId=self.world.physics.client)

    def measured_velocity(self):
        """Ground-truth world linear velocity and angular velocity from physics."""
        self.world.physics.require_connection()
        return bullet.getBaseVelocity(self.body_id, physicsClientId=self.world.physics.client)
