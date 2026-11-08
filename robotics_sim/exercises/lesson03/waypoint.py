"""Follow a sequence of poses using the controller completed in control.py.

Complete StudentWaypointController.wheel_command, save and click Reload exercise.
Choose Lesson 3 > Follow two waypoints > student.
Reference: robotics_sim/sim/references/waypoint_control.py.
"""

import math

from ._control_runtime import WaypointController
from .control import StudentPoseController


GOALS = ((1.0, 0.5, math.pi / 2), (1.0, 1.5, 0.0))  # x [m], y [m], heading [rad]


class StudentWaypointController(WaypointController):
    # The inherited __init__ calls this class once to create self.pose_controller.
    # Reuse that object for every waypoint by calling its set_goal(next_goal).
    pose_controller_class = StudentPoseController

    def wheel_command(self, pose, r, L):
        """Return wheel references for the current waypoint, then advance in order.

        The inherited initializer creates self.pose_controller once, using
        pose_controller_class, and gives it self.goals[0] as its first target.
        Reuse that object: its wheel_command(pose, r, L) is the method from control.py.
        After each arrival, set_goal(next_goal) changes its target and resets TURN.
        Do not call set_goal on every step: it would erase the current phase.
        Gains, tolerances and wheel limit come from control.py.
        Initially self.index = 0 and self.phase = "TURN".

        Here self.done is an inherited, read-only @property that evaluates
        self.index == len(self.goals), not self.phase == "DONE". Read it
        without parentheses; advance self.index rather than assigning to done.
        self.pose_controller.done instead checks that controller's phase:
        it means the current pose is complete; self.done means all poses are.

        1. If self.done, return two zeros.
        2. Call self.pose_controller.wheel_command(pose, r, L).
           Copy that controller's phase into self.phase for the live readout.
        3. If that controller is done (both pose tolerances met), increment
           self.index. If self.done, set self.phase to DONE. Otherwise call
           self.pose_controller.set_goal(self.goals[self.index]) and set
           self.phase to TURN. Return two zeros for this arrival sample.
        4. Otherwise return the current pose controller's wheel references.
        """
        # TODO: current pose controller -> arrival -> next waypoint or DONE.
        raise NotImplementedError(
            "Complete StudentWaypointController.wheel_command in "
            "robotics_sim/exercises/lesson03/waypoint.py."
        )
