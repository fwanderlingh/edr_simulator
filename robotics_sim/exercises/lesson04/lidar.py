"""Student exercise: transform each lidar bearing into a world-space unit ray.

Implement directions(), select student sensor mode, then restart after editing.
The sensor backend handles ray casting and range conversion.
Angles are radians, x is forward, y is left, z is up.
"""

from ...sensors.lidar import LidarSensor


class StudentLidar(LidarSensor):
    def directions(self, yaw):
        # TODO: rotate each bearing by the robot yaw into a world-space unit ray.
        raise NotImplementedError("complete directions() in exercises/lesson04/lidar.py")
