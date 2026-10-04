"""Sample sensors at 10 Hz of simulation time, independently of rendering."""

import math

from .encoder import EncoderSensor
from .imu import ImuSensor
from .lidar import LidarSensor
from ..exercises.lesson04.lidar import StudentLidar


class SensorSuite:
    def __init__(self, robot, mode, dt, *, rangefinder=False):
        self.robot = robot
        noisy = mode == "noisy"
        self.encoder = EncoderSensor(noisy=noisy)
        self.imu = ImuSensor(robot, noisy=noisy)
        self.lidar = (StudentLidar if mode == "student" else LidarSensor)(
            robot.world, noisy=noisy, angles=[0] if rangefinder else None)
        self.period_steps = max(1, math.ceil(0.1 / dt))
        self.time = 0.0
        self.rows = []
        self.capture(0.0)

    def advance(self, time, step, dt, *, final=False):
        self.encoder.advance(self.robot.wheel_velocities, dt)
        if step % self.period_steps == 0 or final:
            self.capture(time)

    def capture(self, time):
        self.encoder.sample()
        self.imu.sample(time - self.time)
        self.lidar.sample(self.robot.pose())
        self.time = time
        self.rows.append((time, *self.encoder.read(), *self.encoder.truth,
                          *self.imu.read(), *self.imu.truth,
                          *self.lidar.ranges, *self.lidar.truth, *map(int, self.lidar.hits)))

    @property
    def columns(self):
        return ("time_s", "left_delta_rad", "right_delta_rad", "true_left_delta_rad", "true_right_delta_rad",
                "gyro_z_rad_s", "accel_x_m_s2", "accel_y_m_s2",
                "true_gyro_z_rad_s", "true_accel_x_m_s2", "true_accel_y_m_s2",
                *(f"range_{i}_m" for i in range(len(self.lidar.angles))),
                *(f"true_range_{i}_m" for i in range(len(self.lidar.angles))),
                *(f"hit_{i}" for i in range(len(self.lidar.angles))))
