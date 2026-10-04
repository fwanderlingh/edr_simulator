"""Implement the planar frame-composition and inverse-coordinate exercise."""

import numpy as np


PLANAR_WORLD_FROM_ROBOT = np.array([
    [0, -1, 2],
    [1, 0, 1],
    [0, 0, 1],
], dtype=float)
PLANAR_ROBOT_FROM_SENSOR = np.array([
    [1, 0, 0.5],
    [0, 1, 0.2],
    [0, 0, 1],
], dtype=float)
PLANAR_POINT_IN_SENSOR = np.array([1, -1], dtype=float)
PLANAR_DIRECTION_IN_SENSOR = np.array([1, 0], dtype=float)
PLANAR_GOAL_IN_WORLD = np.array([4, 3], dtype=float)


def solve_planar_task(world_from_robot, robot_from_sensor, point_in_sensor,
                      direction_in_sensor, goal_in_world):
    """Return the world point, world direction, and robot-frame goal coordinates."""
    # TODO: compose the poses, map the point (homogeneous 1), rotate the
    # direction (homogeneous 0), and invert the robot pose for the goal.
    raise NotImplementedError("complete solve_planar_task in exercises/lesson02/planar.py")
