"""Follow a sequence of poses using the controller completed in control.py.

Complete StudentWaypointController.wheel_command, save and restart the app.
Choose Lesson 3 > Follow two waypoints > student.
Reference: robotics_sim/sim/references/waypoint_control.py.
"""

import math

from ._control_runtime import WaypointController
from .control import StudentPoseController


GOALS = ((1.0, 0.5, math.pi / 2), (1.0, 1.5, 0.0))  # x [m], y [m], heading [rad]


class StudentWaypointController(WaypointController):
    pose_controller_class = StudentPoseController

    def wheel_command(self, pose, r, L):
        """Return wheel references for the current waypoint, then advance in order.

        self.controllers contains a fresh StudentPoseController for each goal.
        Gains, tolerances and wheel limit come from control.py.
        Initially self.index = 0 and self.phase = "TURN".

        1. If self.done, return two zeros.
        2. Select self.controllers[self.index] and call its wheel_command.
           Copy that controller's phase into self.phase for the live readout.
        3. If that controller is done (both pose tolerances met), increment
           self.index. Set self.phase to TURN for the next goal, or DONE when
           self.done. Return two zeros for this arrival sample.
        4. Otherwise return the current pose controller's wheel references.
        """
        # TODO: current pose controller -> arrival -> next waypoint or DONE.
        raise NotImplementedError(
            "Complete StudentWaypointController.wheel_command in "
            "robotics_sim/exercises/lesson03/waypoint.py."
        )


if __name__ == "__main__":
    from ._control_runtime import main
    raise SystemExit(main(task="waypoints"))
