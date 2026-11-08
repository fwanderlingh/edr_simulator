"""Solution for exercises/lesson03/waypoint.py: sequence the pose controller."""

from robotics_sim.exercises.lesson03._control_runtime import WaypointController
from .control import ReferencePoseController


class ReferenceWaypointController(WaypointController):
    # The same inherited initializer creates one reusable ReferencePoseController.
    pose_controller_class = ReferencePoseController

    def wheel_command(self, pose, r, L):
        if self.done:
            return 0.0, 0.0
        controller = self.pose_controller
        wheels = controller.wheel_command(pose, r, L)
        self.phase = controller.phase
        if controller.done:
            self.index += 1
            if self.done:
                self.phase = "DONE"
            else:
                controller.set_goal(self.goals[self.index])
                self.phase = "TURN"
            return 0.0, 0.0
        return wheels
