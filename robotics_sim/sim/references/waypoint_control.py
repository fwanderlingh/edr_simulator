"""Solution for exercises/lesson03/waypoint.py: sequence the pose controller."""

from robotics_sim.exercises.lesson03._control_runtime import WaypointController
from .control import ReferencePoseController


class ReferenceWaypointController(WaypointController):
    # The same inherited initializer creates one reusable ReferencePoseController.
    pose_controller_class = ReferencePoseController

    def wheel_command(self, pose, r, L):
        self.scale = 1.0
        if self.done:
            return 0.0, 0.0
        wheels = self.pose_controller.wheel_command(pose, r, L)
        self.phase = self.pose_controller.phase
        self.scale = self.pose_controller.scale
        if self.pose_controller.done:
            self.index += 1
            if self.done:
                self.phase = "DONE"
            else:
                self.pose_controller.set_goal(self.goals[self.index])
                self.phase = "TURN"
            self.scale = 1.0
            return 0.0, 0.0
        return wheels
