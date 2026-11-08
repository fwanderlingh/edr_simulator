"""Lesson 3 scenes: assigned wheel motion and guided goal/waypoint control."""

from robotics_sim import Pose
from robotics_sim.robots.differential_drive import (
    DifferentialDriveRobot, ReferenceDifferentialDrive,
)
from .kinematics import StudentDifferentialDrive
from . import control, waypoint
from ._control_runtime import controller_parameters
from .control import StudentPoseController
from .waypoint import StudentWaypointController
from robotics_sim.sim.references.control import ReferencePoseController
from robotics_sim.sim.references.waypoint_control import ReferenceWaypointController
from .proportional import GAIN_POS, GOAL, EPS_POS, WHEEL_MAX, StudentProportionalController
from robotics_sim.sim.references.proportional_control import ReferenceProportionalController


WAYPOINT_TASKS = {
    "drive_proportional": (GOAL,),
    "drive_goal": (control.GOAL,),
    "drive_waypoints": waypoint.GOALS,
    "drive_goal_behind": ((-1.0, 0.0, 0.0),),
}


def setup(world, *, task, kinematics="reference"):
    """Spawn a differential-drive robot using the model selected in the UI."""
    if task in WAYPOINT_TASKS:
        # Isolate the controller exercise from the separate wheel-formula task.
        # The student method is first called on Step/Start, so an unfinished
        # controller can still be inspected with its goals visible in Ready.
        robot = DifferentialDriveRobot(world, Pose(x=0, y=0))
        controller_options = {}
        if task == "drive_proportional":
            controller = (StudentProportionalController if kinematics == "student"
                          else ReferenceProportionalController)
            controller_options = {"gain_pos": GAIN_POS, "eps_pos": EPS_POS, "wheel_max": WHEEL_MAX}
        elif task == "drive_waypoints":
            controller = (StudentWaypointController if kinematics == "student"
                          else ReferenceWaypointController)
            controller_options = controller_parameters()
        else:
            controller = (StudentPoseController if kinematics == "student"
                          else ReferencePoseController)
            controller_options = controller_parameters()
        robot.waypoint_controller = controller(WAYPOINT_TASKS[task], **controller_options)
        return robot
    model = StudentDifferentialDrive if kinematics == "student" else ReferenceDifferentialDrive
    robot = DifferentialDriveRobot(world, Pose(x=0, y=0), motor_model=model())
    # Keep Ready inspectable even when the selected student model is a TODO.
    # The guarded Start/Step path evaluates the exercise before physics advances.
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
