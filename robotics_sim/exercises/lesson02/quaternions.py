"""Module 2: Hamilton quaternions in NOTES order (w, x, y, z)."""

import numpy as np


def hamilton(first, second):
    """Compose two wxyz quaternions; the right operand acts first."""
    # TODO: implement the calculation described above.
    raise NotImplementedError("Complete hamilton")


def solve_quaternion_task(quaternion, vector):
    """Return (inverse wxyz, square wxyz, rotated vector, distance(q,-q)).

    Normalize the input; use q*(0,v)*q_conjugate and radians for distance.
    The simulator geometry API uses xyzw, unlike this exercise.
    """
    # TODO: implement the calculation described above.
    raise NotImplementedError("Complete solve_quaternion_task")
