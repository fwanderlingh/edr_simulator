"""Module 2, E2.1: fixed-axis RPY, order and gimbal lock; angles in radians."""

from robotics_sim.geometry.rotations import rot_x, rot_y, rot_z


def solve_rpy_task(roll, pitch, yaw):
    """Return (Rz @ Ry @ Rx, Rx @ Ry @ Rz), both 3x3 matrices.

    Compare the image of the x axis for (pi/2, 0, pi/2).
    Then compare (0, pi/2, 0) and (pi/6, pi/2, pi/6).
    """
    # TODO: implement the calculation described above.
    raise NotImplementedError("Complete solve_rpy_task")
