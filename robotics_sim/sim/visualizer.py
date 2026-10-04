"""Teaching annotations. Named items are replaced instead of accumulating."""

import numpy as np
import pybullet as bullet


class Visualizer:
    def __init__(self, world):
        self.physics = world.physics
        self._items = {}
        if self.physics.gui:
            bullet.resetDebugVisualizerCamera(5.5, 35, -55, (0.8, 0.5, 0), physicsClientId=self.physics.client)

    def draw_vector(self, origin, vector, *, name, color=(0.1, 0.5, 0.5)):
        if not self.physics.gui:
            return
        self._items[name] = bullet.addUserDebugLine(
            origin, np.asarray(origin) + np.asarray(vector), lineColorRGB=color, lineWidth=2,
            replaceItemUniqueId=self._items.get(name, -1), physicsClientId=self.physics.client)

    def draw_text(self, position, text, *, name, color=(0.15, 0.15, 0.15)):
        if not self.physics.gui:
            return
        self._items[name] = bullet.addUserDebugText(
            text, position, textColorRGB=color, textSize=1.2,
            replaceItemUniqueId=self._items.get(name, -1), physicsClientId=self.physics.client)

    def draw_frame(self, transform, *, name, scale=0.5):
        t = np.asarray(transform)
        for axis, color in enumerate(((0.9, 0.15, 0.15), (0.1, 0.7, 0.2), (0.15, 0.3, 0.95))):
            self.draw_vector(t[:3, 3], t[:3, axis] * scale, name=f"{name}:axis:{axis}", color=color)
        self.draw_text(t[:3, 3] + (0, 0, scale + 0.05), name, name=f"{name}:label")

    def draw_point(self, position, *, name, color=(0.6, 0.2, 0.8), size=0.06):
        for axis in range(3):
            offset = np.eye(3)[axis] * size
            self.draw_vector(np.asarray(position) - offset, 2 * offset, name=f"{name}:{axis}", color=color)
        self.draw_text(np.asarray(position) + (0, 0, 0.1), name, name=f"{name}:label", color=color)

    def draw_path(self, points, *, name, color=(0.9, 0.5, 0.15)):
        # Remove obsolete segments when a named path becomes shorter.
        prefix = f"path:{name}:"
        for key in list(self._items):
            if key.startswith(prefix):
                bullet.removeUserDebugItem(self._items.pop(key), physicsClientId=self.physics.client)
        for index, (start, end) in enumerate(zip(points[:-1], points[1:])):
            self.draw_vector(start, np.asarray(end) - start, name=f"{prefix}{index}", color=color)
