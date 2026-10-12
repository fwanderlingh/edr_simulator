"""Lesson 4 scene, sensor attachment, and robot commands."""

import math
from robotics_sim import Pose
from robotics_sim.robots.differential_drive import DifferentialDriveRobot
from .sensors.suite import SensorSuite


def setup(world, *, task):
    """Declare the robot and obstacles once on load or Reset."""
    robot = DifferentialDriveRobot(world, Pose(x=0, y=0))
    world.add_box(Pose(x=3, z=0.5), size=(0.2, 3, 1))
    world.add_box(Pose(x=-1.5, y=2, z=0.5), size=(0.8, 0.8, 1))
    world.add_box(Pose(x=0.8, y=-2, z=0.4), size=(0.6, 0.6, 0.8))
    return robot


def setup_sensors(robot, *, task, mode, dt):
    """Attach sensors to this explicit robot; the UI selects their mode."""
    return SensorSuite(robot, mode, dt, rangefinder=task == "sensor_encoders")


def update(robot, time, dt, *, task):
    """Command the selected experiment before each shared physics step."""
    if task == "sensor_encoders":
        robot.set_twist(0.15, 0.0)
    elif task == "sensor_lidar":
        robot.set_twist(0.0, 0.3)
    elif task == "sensor_imu":
        robot.set_twist(0.2 + 0.1 * math.sin(time), 0.4)
