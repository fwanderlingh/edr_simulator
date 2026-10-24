"""Lesson 5, Part 2: implement Cartesian-error feedback and joint-speed limits."""

import math
import numpy as np


class StudentArmController:
    def __init__(self, model, *, gain=1.5, damping=0.05, max_speed=1.0):
        if not all(math.isfinite(x) and x > 0 for x in (gain, damping, max_speed)):
            raise ValueError("Gain, damping and speed limit must be positive.")
        self.model = model
        self.gain, self.damping, self.max_speed = gain, damping, max_speed

    def velocity(self, q, target):
        """Return two finite joint velocities in rad/s.

        Use model.forward and model.jacobian. Apply the same scale to both
        velocities so neither exceeds max_speed in absolute value.
        """
        # TODO: error -> desired endpoint velocity -> damped inverse -> limit.
        raise NotImplementedError("Complete StudentArmController.velocity in lesson05/control.py")
