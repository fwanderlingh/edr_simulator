"""Legacy ray-direction exercise, retained for programmatic experiments.

The Lesson 4 desktop activities use the supplied LiDAR. The course exercises
implement sector summaries and supervision in avoidance.py.
Angles are radians, x is forward, y is left, z is up.
Return a NumPy array with one (ux, uy, uz) row for each ray.
Reference: robotics_sim/sim/references/lidar.py.
"""

import math
import numpy as np

from .sensors.lidar import LidarSensor


class StudentLidar(LidarSensor):
    def directions(self, yaw):
        # TODO: rotate each ray, collect the rows, and return np.array(directions).
        raise NotImplementedError("complete directions() in exercises/lesson04/lidar.py")
