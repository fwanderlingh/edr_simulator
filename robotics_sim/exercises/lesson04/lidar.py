"""Student exercise: transform each lidar bearing into a world-space unit ray.

Implement directions(), select student sensor mode, then click Reload exercise after saving.
The sensor backend handles ray casting and range conversion.
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
