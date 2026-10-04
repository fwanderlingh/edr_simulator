"""Ideal shaft angles and optional quantized encoder measurements."""

import math
import numpy as np


class EncoderSensor:
    def __init__(self, *, noisy=False, seed=1, ticks_per_turn=360):
        self.noisy = noisy
        self.rng = np.random.default_rng(seed)
        self.resolution = 2 * math.pi / ticks_per_turn
        self.angles = np.zeros(2)
        self.previous = np.zeros(2)
        self.previous_true = np.zeros(2)
        self.value = (0.0, 0.0)
        self.truth = (0.0, 0.0)

    def advance(self, wheel_velocities, dt):
        # Lesson 3 has ideal shafts, not physical wheel joints. Encoder angles
        # therefore integrate shaft commands, even when the chassis is blocked.
        self.angles += np.asarray(wheel_velocities) * dt

    def sample(self):
        measured = self.angles.copy()
        if self.noisy:
            measured += self.rng.normal(0, self.resolution / 4, 2)
            measured = np.round(measured / self.resolution) * self.resolution
        self.truth = tuple(self.angles - self.previous_true)
        self.value = tuple(measured - self.previous)
        self.previous_true = self.angles.copy()
        self.previous = measured
        return self.read()

    def read(self):
        """Latest left/right angular increments (rad), without resampling."""
        return self.value
