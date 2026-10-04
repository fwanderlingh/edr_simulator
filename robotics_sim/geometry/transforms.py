"""T_A_B maps coordinates expressed in B into coordinates expressed in A."""

import numpy as np


def homogeneous(rotation, position):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = position
    return transform


def compose(first, second):
    return np.asarray(first) @ np.asarray(second)


def inverse(transform):
    """Inverse of a rigid transform (assumes an orthonormal rotation)."""
    t = np.asarray(transform)
    return homogeneous(t[:3, :3].T, -t[:3, :3].T @ t[:3, 3])


def transform_point(transform, point):
    t = np.asarray(transform)
    return t[:3, :3] @ np.asarray(point) + t[:3, 3]
