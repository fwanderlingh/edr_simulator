"""One isolated PyBullet connection per world."""

from math import isfinite

import pybullet as bullet


class Physics:
    def __init__(self, *, gui=True, dt=1 / 240):
        if not isfinite(dt) or dt <= 0:
            raise ValueError("The timestep must be finite and positive.")
        self.dt = dt
        self.gui = gui
        self.client = bullet.connect(bullet.GUI if gui else bullet.DIRECT)
        if self.client < 0:
            raise RuntimeError("Could not connect to PyBullet.")
        self._closed = False
        bullet.setTimeStep(dt, physicsClientId=self.client)
        bullet.setRealTimeSimulation(0, physicsClientId=self.client)
        # Lessons 1-2 isolate prescribed motion from gravity and wheel dynamics.
        bullet.setGravity(0, 0, 0, physicsClientId=self.client)

    @property
    def connected(self):
        return not self._closed and bool(bullet.isConnected(self.client))

    def require_connection(self):
        if not self.connected:
            raise RuntimeError("The simulation world is closed.")

    def step(self):
        self.require_connection()
        bullet.stepSimulation(physicsClientId=self.client)

    def close(self):
        if self.connected:
            bullet.disconnect(physicsClientId=self.client)
        self._closed = True
