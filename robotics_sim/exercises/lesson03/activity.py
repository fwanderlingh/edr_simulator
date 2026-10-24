"""Lesson 3 scenes: assigned wheel motion and guided goal/waypoint control."""

from robotics_sim import Pose
from robotics_sim.robots.differential_drive import (
    DifferentialDriveRobot, ReferenceDifferentialDrive,
)
from .kinematics import StudentDifferentialDrive
from .control import DEFAULT_GOALS, StudentWaypointController
from .control_reference import ReferenceWaypointController


WAYPOINT_TASKS = {
    "drive_goal": (DEFAULT_GOALS[0],),
    "drive_waypoints": DEFAULT_GOALS,
    "drive_goal_behind": ((-1.0, 0.0, 0.0),),
}


def setup(world, *, task, kinematics="reference"):
    """Spawn a differential-drive robot using the model selected in the UI."""
    if task in WAYPOINT_TASKS:
        # Isolate the controller exercise from the separate wheel-formula task.
        # The student method is first called on Step/Start, so an unfinished
        # controller can still be inspected with its goals visible in Ready.
        robot = DifferentialDriveRobot(world, Pose(x=0, y=0))
        controller = (StudentWaypointController if kinematics == "student"
                      else ReferenceWaypointController)
        robot.waypoint_controller = controller(WAYPOINT_TASKS[task])
        return robot
    model = StudentDifferentialDrive if kinematics == "student" else ReferenceDifferentialDrive
    robot = DifferentialDriveRobot(world, Pose(x=0, y=0), motor_model=model())
    # Initialize the command so the Ready view also shows the chosen speeds.
    update(robot, 0.0, world.physics.dt, task=task)
    return robot


def update(robot, time, dt, *, task):
    """Apply wheel speeds (rad/s) or a body twist (m/s, rad/s)."""
    if task in WAYPOINT_TASKS:
        robot.waypoint_controller.update(robot, time, dt)
    elif task == "drive_line":
        robot.set_wheel_velocities(2.0, 2.0)
    elif task == "drive_circle":
        robot.set_wheel_velocities(1.3, 2.7)
    elif task == "drive_spin":
        robot.set_wheel_velocities(-1.0, 1.0)
    elif task == "drive_inverse":
        robot.set_twist(0.24, 0.4)
