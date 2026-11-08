"""Solution for exercises/lesson03/waypoint.py: sequence the pose controller."""

from robotics_sim.exercises.lesson03._control_runtime import WaypointController
from .control import ReferencePoseController


class ReferenceWaypointController(WaypointController):
    pose_controller_class = ReferencePoseController

    def wheel_command(self, pose, r, L):
        if self.done:
            return 0.0, 0.0
        controller = self.controllers[self.index]
        wheels = controller.wheel_command(pose, r, L)
        self.phase = controller.phase
        if controller.done:
            self.index += 1
            self.phase = "DONE" if self.done else "TURN"
            return 0.0, 0.0
        return wheels
