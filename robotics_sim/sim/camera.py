"""Cached camera projection; optional raster capture for diagnostics."""

import numpy as np
import pybullet as bullet


class Camera:
    def __init__(self):
        self.target = (0.8, 0.7, 0.1)
        self.key = None
        self.reset()

    def reset(self):
        self.yaw, self.pitch, self.distance = 35.0, -55.0, 5.5

    def orbit(self, dx, dy):
        self.yaw += dx * 0.4
        self.pitch = float(np.clip(self.pitch + dy * 0.4, -89, -5))

    def zoom(self, direction):
        self.distance = float(np.clip(self.distance * (0.9 ** direction), 1.5, 14))

    def prepare(self, width, height):
        """Refresh matrices only when the view changes, never rasterize here."""
        key = (tuple(self.target), self.distance, self.yaw, self.pitch, width, height)
        if key == self.key:
            return
        view = bullet.computeViewMatrixFromYawPitchRoll(self.target, self.distance,
                                                        self.yaw, self.pitch, 0, 2)
        projection = bullet.computeProjectionMatrixFOV(50, width / height, 0.05, 50)
        self.view = np.array(view).reshape(4, 4, order="F")
        self.projection = np.array(projection).reshape(4, 4, order="F")
        self.transform = self.projection @ self.view
        self.eye = -self.view[:3, :3].T @ self.view[:3, 3]
        self.width, self.height = width, height
        self.key = key

    def render(self, world, width, height):
        """Explicit image capture, not used by the interactive viewport."""
        self.prepare(width, height)
        _, _, rgba, _, _ = bullet.getCameraImage(
            width, height, viewMatrix=self.view.flatten(order="F"), projectionMatrix=self.projection.flatten(order="F"),
            renderer=bullet.ER_TINY_RENDERER, flags=bullet.ER_NO_SEGMENTATION_MASK,
            physicsClientId=world.physics.client)
        rgb = np.asarray(rgba, dtype=np.uint8).reshape(height, width, 4)[:, :, :3]
        return f"P6\n{width} {height}\n255\n".encode("ascii") + rgb.tobytes()

    def project(self, point):
        clip = self.transform @ np.append(point, 1)
        if clip[3] <= 0:
            return None
        x, y, z = clip[:3] / clip[3]
        if not -1 <= z <= 1:
            return None
        return (x + 1) * self.width / 2, (1 - y) * self.height / 2
