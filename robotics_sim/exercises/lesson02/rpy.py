"""Module 2 assignment: sensor RPY to a rotation matrix; angles in radians."""

import numpy as np
from robotics_sim.geometry.rotations import rot_x, rot_y, rot_z


def solve_rpy_task(roll, pitch, yaw):
    """Return (expanded RPY matrix, Rz @ Ry @ Rx), both 3x3 matrices.

    Implement the nine entries from the notes, then compute the same
    rotation by multiplication for comparison. The scene supplies
    (pi/6, -pi/9, 2*pi/9) and displays R @ [1, 0, 0].
    """
    # TODO: implement the expanded matrix and the product separately.
    raise NotImplementedError("Complete solve_rpy_task")
