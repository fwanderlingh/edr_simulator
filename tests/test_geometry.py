import numpy as np
import pybullet as bullet
import pytest

from robotics_sim.exercises.lesson02.frames import point_in_world
from robotics_sim.exercises.lesson02.planar import solve_planar_task
from robotics_sim.geometry.rotations import matrix_to_quaternion, matrix_to_rpy, quaternion_to_matrix, rpy_to_matrix
from robotics_sim.geometry.transforms import homogeneous, inverse, transform_point


@pytest.mark.parametrize("rpy", [(0.3, -0.5, 1.2), (0.2, np.pi/2, 0.7)])
def test_rotation_conventions_against_bullet(rpy):
    rotation = rpy_to_matrix(*rpy)
    expected = np.array(bullet.getMatrixFromQuaternion(bullet.getQuaternionFromEuler(rpy))).reshape(3, 3)
    np.testing.assert_allclose(rotation, expected, atol=1e-12)
    np.testing.assert_allclose(quaternion_to_matrix(matrix_to_quaternion(rotation)), rotation, atol=1e-12)
    np.testing.assert_allclose(rpy_to_matrix(*matrix_to_rpy(rotation)), rotation, atol=1e-12)


@pytest.mark.solution
def test_sensor_chain_against_bullet_oracle():
    position_a, position_b = (1, -2, 0.5), (0.3, 0.2, 0.4)
    qa = bullet.getQuaternionFromEuler((0.2, -0.4, 1.1))
    qb = bullet.getQuaternionFromEuler((-0.3, 0.2, -0.6))
    a = homogeneous(quaternion_to_matrix(qa), position_a)
    b = homogeneous(quaternion_to_matrix(qb), position_b)
    point = (0.6, -0.1, 0.2)
    sensor_position, sensor_orientation = bullet.multiplyTransforms(position_a, qa, position_b, qb)
    expected, _ = bullet.multiplyTransforms(sensor_position, sensor_orientation, point, (0, 0, 0, 1))
    np.testing.assert_allclose(point_in_world(a, b, point), expected, atol=2e-6)
    np.testing.assert_allclose(inverse(a) @ a, np.eye(4), atol=1e-12)
    np.testing.assert_allclose(transform_point(inverse(a), transform_point(a, point)), point, atol=1e-12)


@pytest.mark.solution
def test_planar_module_composes_points_rotates_directions_and_inverts_goal():
    from robotics_sim.exercises.lesson02.planar import (
        PLANAR_DIRECTION_IN_SENSOR,
        PLANAR_GOAL_IN_WORLD,
        PLANAR_POINT_IN_SENSOR,
        PLANAR_ROBOT_FROM_SENSOR,
        PLANAR_WORLD_FROM_ROBOT,
    )

    point, direction, goal = solve_planar_task(
        PLANAR_WORLD_FROM_ROBOT, PLANAR_ROBOT_FROM_SENSOR,
        PLANAR_POINT_IN_SENSOR, PLANAR_DIRECTION_IN_SENSOR,
        PLANAR_GOAL_IN_WORLD)
    np.testing.assert_allclose(point, [2.8, 2.5], atol=1e-12)
    np.testing.assert_allclose(direction, [0, 1], atol=1e-12)
    np.testing.assert_allclose(goal, [2, -2], atol=1e-12)
