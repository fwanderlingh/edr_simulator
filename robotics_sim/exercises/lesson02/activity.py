"""Lesson 2 scene and commands. Select an example in the UI, then Start."""

import math
import numpy as np
from robotics_sim import Pose, Robot
from robotics_sim.geometry.rotations import rot_z
from robotics_sim.geometry.transforms import homogeneous
from .planar import PLANAR_WORLD_FROM_ROBOT

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
