"""Right-handed rotations acting on column vectors; quaternions use xyzw."""

import numpy as np


def rot_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rpy_to_matrix(roll, pitch, yaw):
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def matrix_to_rpy(rotation):
    """Return roll, pitch, yaw; at gimbal lock choose roll = 0."""
    r = np.asarray(rotation, dtype=float)
    pitch = np.arctan2(-r[2, 0], np.hypot(r[0, 0], r[1, 0]))
    if np.hypot(r[0, 0], r[1, 0]) < 1e-10:
        return np.array([0.0, pitch, np.arctan2(-r[0, 1], r[1, 1])])
    return np.array([np.arctan2(r[2, 1], r[2, 2]), pitch, np.arctan2(r[1, 0], r[0, 0])])


def quaternion_to_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-15:
        raise ValueError("Expected a finite, nonzero xyzw quaternion.")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def matrix_to_quaternion(rotation):
    """Convert a proper rotation matrix to a unit xyzw quaternion."""
    r = np.asarray(rotation, dtype=float)
    if (r.shape != (3, 3) or not np.all(np.isfinite(r))
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(r), 1.0)):
        raise ValueError("Expected a proper 3x3 rotation matrix.")
    # A symmetric eigenproblem remains well-conditioned at a 180-degree turn.
    k = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
        [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
        [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)],
    ]) / 3
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    return q if q[3] >= 0 else -q
