"""Lesson 3 scene and commands; kinematics formulas live in kinematics.py."""

from robotics_sim import Pose
from robotics_sim.robots.differential_drive import (
    DifferentialDriveRobot, ReferenceDifferentialDrive,
)
from .kinematics import StudentDifferentialDrive


def setup(world, *, task, kinematics="reference"):
    """Spawn a differential-drive robot using the model selected in the UI."""
    model = StudentDifferentialDrive if kinematics == "student" else ReferenceDifferentialDrive
    robot = DifferentialDriveRobot(world, Pose(x=0, y=0), motor_model=model())
    # Initialize the command so the Ready view also shows the chosen speeds.
    update(robot, 0.0, world.physics.dt, task=task)
    return robot


def update(robot, time, dt, *, task):
    """Apply wheel speeds (rad/s) or a body twist (m/s, rad/s)."""
    if task == "drive_line":
        robot.set_wheel_velocities(2.0, 2.0)
    elif task == "drive_circle":
        robot.set_wheel_velocities(1.3, 2.7)
    elif task == "drive_spin":
        robot.set_wheel_velocities(-1.0, 1.0)
    elif task == "drive_inverse":
        robot.set_twist(0.24, 0.4)
