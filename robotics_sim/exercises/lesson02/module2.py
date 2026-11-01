"""Scene annotations for the three worked Module 2 exercises.

    Geometric frames here are mathematical frames, not body-centre poses:
    Exercises places the base origin at z=0, as in the notes.
"""

import numpy as np
from typing import NamedTuple

from robotics_sim.geometry.rotations import rot_x, rot_z, quaternion_to_matrix, rpy_to_matrix
from robotics_sim.geometry.transforms import homogeneous
from . import spatial, rpy, quaternions

TASKS = ("spatial", "rpy", "quaternions")
WORLD_FROM_BASE = homogeneous(rot_z(np.pi / 2), (1, 2, 0))
BASE_FROM_SENSOR = homogeneous(rot_x(np.pi / 2), (0.2, 0, 0.5))
POINT_IN_SENSOR = np.array([0., 1., 0.])
RPY_ANGLES = (np.pi / 6, -np.pi / 9, 2 * np.pi / 9)
RPY_DIRECTION = np.array([1., 0., 0.])


class Evaluation(NamedTuple):
    frames: list
    arrows: list
    readout: str
    checks: tuple


CHECK_LABELS = {
    "spatial": ("Sensor pose matches the composed transforms", "Point maps into the world frame"),
    "rpy": ("R matches the expected rotation", "R matches Rz × Ry × Rx",
            "Rᵀ R = I (orthogonal)", "det(R) = 1"),
    "quaternions": ("Inverse matches the unit conjugate", "q × q gives a 180° rotation",
                    "Rotated vector matches the expected result", "q and −q have zero angular distance"),
}


def close(actual, expected):
    return np.shape(actual) == np.shape(expected) and bool(np.allclose(actual, expected, atol=1e-6, rtol=0))


def evaluate(task):
    """Return scene annotations and numerical checks; propagate unfinished errors."""
    def fmt(value):
        return np.array2string(np.asarray(value), precision=3, suppress_small=True)

    if task == "spatial":
        frame, point = spatial.solve_spatial_task(
            WORLD_FROM_BASE.copy(), BASE_FROM_SENSOR.copy(), POINT_IN_SENSOR.copy())
        expected = WORLD_FROM_BASE @ BASE_FROM_SENSOR
        return Evaluation([(WORLD_FROM_BASE, "Base (notes)"), (frame, "Sensor")],
                [(frame[:3, 3], point, "Sensor point", "#773baa")],
                f"Sensor origin: {fmt(frame[:3, 3])} m | Point: {fmt(point)} m",
                tuple(zip(CHECK_LABELS[task], (close(frame, expected),
                          close(point, (expected @ np.r_[POINT_IN_SENSOR, 1])[:3])))))
    if task == "rpy":
        rotation, product = rpy.solve_rpy_task(*RPY_ANGLES)
        direction_in_world = rotation @ RPY_DIRECTION
        expected = rpy_to_matrix(*RPY_ANGLES)
        origin = np.array([0, 0, 0.8])
        matrix_text = "; ".join(fmt(row) for row in rotation)
        return Evaluation([(homogeneous(rotation, origin), "Sensor: RPY assignment")],
                [(origin, origin + direction_in_world, "v_w = R v_s", "#773baa")],
                "RPY (rad): (pi/6, -pi/9, 2pi/9) | v_s: (1, 0, 0)"
                f"\nv_w: {np.array2string(direction_in_world, precision=6, suppress_small=True)}"
                f" | expected: {fmt(expected @ RPY_DIRECTION)}"
                f"\nR: {matrix_text}",
                tuple(zip(CHECK_LABELS[task], (close(rotation, expected), close(rotation, product),
                          close(rotation.T @ rotation, np.eye(3)), close(np.linalg.det(rotation), 1)))))
    if task == "quaternions":
        a = np.sqrt(0.5)
        q, vector = np.array([a, 0, 0, a]), np.array([0, 1, 0])
        inverse, square, rotated, distance = quaternions.solve_quaternion_task(q, vector)
        origin = np.array([0, 0, 0.8])
        # Explicit boundary conversion: notes wxyz -> geometry xyzw.
        rotation = quaternion_to_matrix(q[[1, 2, 3, 0]])
        return Evaluation([(homogeneous(rotation, origin), "q: +90 deg about z")],
                [(origin, origin + vector, "v", "#1976a5"),
                 (origin, origin + rotated, "q v q inverse", "#773baa")],
                f"Quaternions (wxyz) | inverse: {fmt(inverse)} | q*q: {fmt(square)}"
                f" | Rv: {fmt(rotated)} | distance(q,-q): {distance:.3f} rad",
                tuple(zip(CHECK_LABELS[task], (close(inverse, [a, 0, 0, -a]),
                          close(square, [0, 0, 0, 1]), close(rotated, [-1, 0, 0]), close(distance, 0)))))
    raise ValueError(f"Unknown Module 2 task: {task}")
