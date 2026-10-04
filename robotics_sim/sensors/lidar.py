"""Planar front-mounted scan with explicit misses and clipped noisy ranges."""

import math
import numpy as np


class LidarSensor:
    def __init__(self, world, *, noisy=False, seed=2, angles=None, max_range=4.0):
        self.world = world
        self.noisy = noisy
        self.rng = np.random.default_rng(seed)
        self.angles = np.linspace(-math.pi / 2, math.pi / 2, 37) if angles is None else np.asarray(angles)
        self.max_range = max_range

    def directions(self, yaw):
        return np.array([(math.cos(yaw + a), math.sin(yaw + a), 0) for a in self.angles])

    def sample(self, pose):
        # Mount above the chassis/wheels so rays do not intersect the robot.
        self.origin = np.array((pose.x, pose.y, pose.z + 0.20))
        directions = np.asarray(self.directions(pose.yaw), dtype=float)
        if directions.shape != (len(self.angles), 3) or not np.all(np.isfinite(directions)):
            raise ValueError("Lidar directions must be a finite N x 3 array.")
        if not np.allclose(np.linalg.norm(directions, axis=1), 1):
            raise ValueError("Lidar directions must be unit vectors.")
        endpoints = self.origin + directions * self.max_range
        results = self.world.raycast([self.origin.tolist()] * len(endpoints), endpoints.tolist())
        self.hits = np.array([body >= 0 for body, _ in results])
        self.truth = np.array([fraction * self.max_range if body >= 0 else self.max_range
                               for body, fraction in results])
        self.ranges = self.truth.copy()
        if self.noisy:
            self.ranges[self.hits] += self.rng.normal(0, 0.02, int(self.hits.sum()))
            self.ranges = np.clip(self.ranges, 0, self.max_range)
        self.endpoints = self.origin + directions * self.ranges[:, None]
        return self.scan()

    def scan(self):
        """Latest ranges in metres; max range plus hit=False represents a miss."""
        return self.ranges.copy()
