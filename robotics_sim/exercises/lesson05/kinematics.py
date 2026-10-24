"""Lesson 5, Parts 1–2: implement the planar-arm model in metres and radians."""

import math
import numpy as np


class StudentPlanarArm:
    def __init__(self, l1=1.0, l2=1.0):
        if not all(math.isfinite(x) and x > 0 for x in (l1, l2)):
            raise ValueError("Link lengths must be finite and positive.")
        self.l1, self.l2 = l1, l2

    def forward(self, q):
        """Return a 2-element endpoint array; q2 is relative to link 1."""
        # TODO: compose the two planar link vectors.
        raise NotImplementedError("Complete StudentPlanarArm.forward in lesson05/kinematics.py")

    def inverse(self, target):
        """Return both 2-element joint solutions, or [] if unreachable.

        Handle the infinitely folded equal-link origin with ValueError.
        """
        # TODO: check reachability, then construct both geometric branches.
        raise NotImplementedError("Complete StudentPlanarArm.inverse in lesson05/kinematics.py")

    def jacobian(self, q):
        """Return a 2 x 2 matrix mapping joint speeds to endpoint speed."""
        # TODO: differentiate each component of forward() with respect to q.
        raise NotImplementedError("Complete StudentPlanarArm.jacobian in lesson05/kinematics.py")
