"""Lesson 2 scene and commands. Select an example in the UI, then Start."""

from dataclasses import dataclass
import math
import numpy as np
from robotics_sim import Pose, Robot
from robotics_sim.geometry.rotations import rot_z
from robotics_sim.geometry.transforms import homogeneous
PLANAR_WORLD_FROM_ROBOT = np.array([
    [0, -1, 2],
    [1, 0, 1],
    [0, 0, 1],
], dtype=float)
PLANAR_ROBOT_FROM_SENSOR = np.array([
    [1, 0, 0.5],
    [0, 1, 0],
    [0, 0, 1],
], dtype=float)
PLANAR_POINT_IN_SENSOR = np.array([1, 0], dtype=float)
PLANAR_DIRECTION_IN_SENSOR = np.array([1, 0], dtype=float)

# Fixed attachment and point for the optional 3D examples.
ROBOT_FROM_SENSOR = homogeneous(rot_z(math.pi / 4), (0.25, 0, 0.3))
POINT_IN_SENSOR = np.array([0.6, 0.2, 0.1])


def setup(world, *, task):
    """Declare the scene and spawn the robot once on load or Reset."""
    world.add_box(Pose(x=1, y=1, z=0.3), size=(0.6, 0.6, 0.6))
    pose = Pose()
    if task == "planar":
        transform = PLANAR_WORLD_FROM_ROBOT
        pose = Pose(x=transform[0, 2], y=transform[1, 2],
                    yaw=math.atan2(transform[1, 0], transform[0, 0]))
    robot = Robot(world, pose)
    return robot


def update(robot, time, dt, *, task):
    """Only the rotating-frames example moves; the other two stay fixed."""
    robot.set_velocity(omega=0.3 if task == "frames" else 0.0)


@dataclass
class PlanarScene:
    """A sensor frame attached to a specific robot, with local observations."""

    robot: Robot
    robot_from_sensor: np.ndarray
    point_in_sensor: np.ndarray
    direction_in_sensor: np.ndarray

    def world_from_robot(self):
        """Read the current simulated pose, not the initial scene constant."""
        pose = self.robot.pose()
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        return np.array([[c, -s, pose.x], [s, c, pose.y], [0, 0, 1]])

    def sensor_frame(self):
        """Current world-from-sensor transform, embedded in 3D for drawing."""
        transform = self.world_from_robot() @ self.robot_from_sensor
        frame = np.eye(4)
        frame[:2, :2] = transform[:2, :2]
        frame[:2, 3] = transform[:2, 2]
        return frame


def setup_planar_scene(robot):
    """Attach the sensor and its observations to the robot created by setup."""
    return PlanarScene(
        robot, PLANAR_ROBOT_FROM_SENSOR.copy(),
        PLANAR_POINT_IN_SENSOR.copy(), PLANAR_DIRECTION_IN_SENSOR.copy())
