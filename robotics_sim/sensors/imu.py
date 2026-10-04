"""Planar gyro and body-frame linear acceleration (gravity disabled)."""

import math
import numpy as np


class ImuSensor:
    def __init__(self, robot, *, noisy=False, seed=3):
        self.robot, self.noisy = robot, noisy
        self.rng = np.random.default_rng(seed)
        self.previous_velocity = np.asarray(robot.measured_velocity()[0])
        self.truth = self.value = (0.0, 0.0, 0.0)

    def sample(self, dt):
        linear, angular = self.robot.measured_velocity()
        velocity = np.asarray(linear)
        acceleration = (velocity - self.previous_velocity) / dt if dt > 0 else np.zeros(3)
        self.previous_velocity = velocity
        yaw = self.robot.pose().yaw
        ax = math.cos(yaw) * acceleration[0] + math.sin(yaw) * acceleration[1]
        ay = -math.sin(yaw) * acceleration[0] + math.cos(yaw) * acceleration[1]
        self.truth = (angular[2], ax, ay)
        measured = np.asarray(self.truth)
        if self.noisy:
            measured += (0.01, 0.02, -0.02) + self.rng.normal(0, (0.005, 0.03, 0.03))
        self.value = tuple(measured)
        return self.read()

    def read(self):
        """Latest yaw rate (rad/s), forward and lateral acceleration (m/s²)."""
        return self.value
