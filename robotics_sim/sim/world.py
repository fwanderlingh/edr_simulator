"""Scene ownership; bodies and debug items belong to this world's connection."""

import numpy as np
import pybullet as bullet

from ..geometry.rotations import matrix_to_quaternion
from ..types import Pose
from .physics import Physics


class World:
    def __init__(self, *, gui=True, dt=1 / 240):
        self.physics = Physics(gui=gui, dt=dt)
        self.add_box(Pose(z=-0.05), size=(10, 10, 0.1), color=(0.86, 0.88, 0.9, 1))

    def add_box(self, pose, *, size=(0.5, 0.5, 0.5), color=(0.9, 0.5, 0.15, 1), mass=0):
        self.physics.require_connection()
        dimensions = np.asarray(size, dtype=float)
        if dimensions.shape != (3,) or not np.all(np.isfinite(dimensions)) or np.any(dimensions <= 0):
            raise ValueError("Box dimensions must be three finite positive lengths.")
        if not np.isfinite(mass) or mass < 0:
            raise ValueError("Mass must be finite and nonnegative.")
        client = self.physics.client
        collision = bullet.createCollisionShape(bullet.GEOM_BOX, halfExtents=dimensions / 2, physicsClientId=client)
        visual = bullet.createVisualShape(bullet.GEOM_BOX, halfExtents=dimensions / 2, rgbaColor=color, physicsClientId=client)
        return bullet.createMultiBody(
            baseMass=mass, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual,
            basePosition=pose.position, baseOrientation=matrix_to_quaternion(pose.matrix()[:3, :3]),
            physicsClientId=client,
        )

    def close(self):
        self.physics.close()

    def raycast(self, origins, endpoints):
        """Return (body id, hit fraction) for each world-space ray."""
        self.physics.require_connection()
        return [(hit[0], hit[2]) for hit in bullet.rayTestBatch(
            origins, endpoints, physicsClientId=self.physics.client)]

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
