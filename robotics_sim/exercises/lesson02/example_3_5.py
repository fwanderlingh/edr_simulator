"""Run from simulator/: python -m robotics_sim.exercises.lesson02.example_3_5."""

import numpy as np

from .planar import solve_planar_task


def main():
    # T_w_b = world_from_robot; T_b_s = robot_from_sensor.
    T_w_b = np.array([[0, -1, 2], [1, 0, 1], [0, 0, 1]], dtype=float)
    T_b_s = np.array([[1, 0, 0.5], [0, 1, 0], [0, 0, 1]], dtype=float)
    P_s = np.array([1, 0, 1], dtype=float)

    # Sensor -> robot -> world, using homogeneous points.
    P_b = T_b_s @ P_s
    P_w = T_w_b @ P_b
    T_w_s = T_w_b @ T_b_s
    P_w_direct = T_w_s @ P_s
    np.testing.assert_allclose(P_b, [1.5, 0, 1])
    np.testing.assert_allclose(P_w, [2, 2.5, 1])
    np.testing.assert_allclose(P_w_direct, P_w)
    print("P_b =", P_b)
    print("T_w_s =\n", T_w_s)
    print("P_w =", P_w)

    # Same sensor pose, new observation at 30 degrees.
    beta = np.pi / 6
    P_s_lidar = np.array([np.cos(beta), np.sin(beta), 1])
    P_w_lidar = T_w_s @ P_s_lidar
    O_s_w = T_w_s @ np.array([0, 0, 1])
    distance = np.linalg.norm(P_w_lidar[:2] - O_s_w[:2])
    np.testing.assert_allclose(P_w_lidar, [1.5, 1.5 + np.sqrt(3)/2, 1])
    np.testing.assert_allclose(distance, 1)
    print("P_w (lidar) =", P_w_lidar)
    print("Distance from sensor =", distance)

    # Simulator API: points/directions have two Cartesian components.
    point, direction = solve_planar_task(
        T_w_b, T_b_s, P_s[:2], np.array([1, 0])
    )
    np.testing.assert_allclose(point, P_w[:2])
    np.testing.assert_allclose(direction, [0, 1])
    print("Simulator: point, direction =", point, direction)


if __name__ == "__main__":
    main()
