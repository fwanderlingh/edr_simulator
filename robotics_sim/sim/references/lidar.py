"""Solution for exercises/lesson04/lidar.py: rotate sensor ray directions.

Notation follows EDR.02/EDR.04: R_w_b converts base components to world
components; u_s and u_w are directions, not points P or homogeneous points P_bar.
The planar sensor axes coincide with the base axes, so R_b_s is the identity.
"""

import math

import numpy as np

from robotics_sim.exercises.lesson04.sensors.lidar import LidarSensor


class ReferenceLidar(LidarSensor):
    def directions(self, yaw):
        """Return a NumPy array with one world-space unit ray per row.

        yaw is theta in the notes. A direction uses u_w = R_w_b @ u_s.
        Its extra homogeneous coordinate would be 0, so translation contributes
        nothing. The returned third component is physical z, which is also zero
        for a horizontal ray; it is not a homogeneous coordinate.
        """
        R_w_b = np.array([
            [math.cos(yaw), -math.sin(yaw)],
            [math.sin(yaw), math.cos(yaw)],
        ])
        directions = []
        for beta_i in self.angles:
            u_s = np.array([math.cos(beta_i), math.sin(beta_i)])
            u_w = R_w_b @ u_s
            directions.append((u_w[0], u_w[1], 0.0))
        return np.array(directions)
