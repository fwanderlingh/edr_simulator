"""Scene annotations for the three worked Module 2 exercises.

    Geometric frames here are mathematical frames, not body-centre poses:
    Exercises places the base origin at z=0, as in the notes.
"""

import numpy as np

from robotics_sim.geometry.rotations import rot_x, rot_z, quaternion_to_matrix
from robotics_sim.geometry.transforms import homogeneous
from . import spatial, rpy, quaternions

TASKS = ("spatial", "rpy", "quaternions")
WORLD_FROM_BASE = homogeneous(rot_z(np.pi / 2), (1, 2, 0))
BASE_FROM_SENSOR = homogeneous(rot_x(np.pi / 2), (0.2, 0, 0.5))
POINT_IN_SENSOR = np.array([0., 1., 0.])


def evaluate(task):
    """Return (frames, arrows, readout); propagate unfinished exercise errors."""
    def fmt(value):
        return np.array2string(np.asarray(value), precision=3, suppress_small=True)

    if task == "spatial":
        frame, point = spatial.solve_spatial_task(
            WORLD_FROM_BASE.copy(), BASE_FROM_SENSOR.copy(), POINT_IN_SENSOR.copy())
        return ([(WORLD_FROM_BASE, "Base (notes)"), (frame, "Sensor")],
                [(frame[:3, 3], point, "Sensor point", "#773baa")],
                f"Sensor origin: {fmt(frame[:3, 3])} m | Point: {fmt(point)} m")
    if task == "rpy":
        rotation, reversed_order = rpy.solve_rpy_task(np.pi / 2, 0, np.pi / 2)
        lock_a, _ = rpy.solve_rpy_task(0, np.pi / 2, 0)
        lock_b, _ = rpy.solve_rpy_task(np.pi / 6, np.pi / 2, np.pi / 6)
        origin, other = np.array([0, 0, 0.8]), np.array([1.5, 0, 0.8])
        return ([(homogeneous(rotation, origin), "Rz Ry Rx"),
                 (homogeneous(reversed_order, other), "Rx Ry Rz")],
                [(origin, origin + rotation[:, 0], "R i", "#773baa"),
                 (other, other + reversed_order[:, 0], "Reverse i", "#1976a5")],
                f"E2.1 | R i: {fmt(rotation[:, 0])} | Reverse i: {fmt(reversed_order[:, 0])}"
                f" | Gimbal lock: same R = {np.allclose(lock_a, lock_b)}")
    if task == "quaternions":
        a = np.sqrt(0.5)
        q, vector = np.array([a, 0, 0, a]), np.array([0, 1, 0])
        inverse, square, rotated, distance = quaternions.solve_quaternion_task(q, vector)
        origin = np.array([0, 0, 0.8])
        # Explicit boundary conversion: notes wxyz -> geometry xyzw.
        rotation = quaternion_to_matrix(q[[1, 2, 3, 0]])
        return ([(homogeneous(rotation, origin), "q: +90 deg about z")],
                [(origin, origin + vector, "v", "#1976a5"),
                 (origin, origin + rotated, "q v q inverse", "#773baa")],
                f"E2.2 (wxyz) | inverse: {fmt(inverse)} | q*q: {fmt(square)}"
                f" | Rv: {fmt(rotated)} | distance(q,-q): {distance:.3f} rad")
    raise ValueError(f"Unknown Module 2 task: {task}")
