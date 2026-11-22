"""Guided reactive supervisor for one isolated obstacle; this is not a planner.

Choose Lesson 4 > Reach a goal and avoid an obstacle, select the student
supervisor, then save your implementation and click Reload exercise.
The student sector summary and supervisor command are deliberately unfinished.
Reference: robotics_sim/sim/references/avoidance.py.
"""

import math

from ._runtime import ObstacleSupervisor, SectorSummary
from ..lesson03._control_runtime import wrap


# Defaults for programmatic sessions. The desktop supplies the LiDAR and offers
# sectors (exercise 1) and avoid (exercise 2) under EXPERIMENT STAGE.
AVOIDANCE_STAGE = "sectors"  # "sectors", "stop" (reuse rangefinder.py), or "avoid"
LIDAR_MODEL = "reference"  # "reference" or "student"
# Scene settings for all LiDAR stages, shared by student and reference modes.
OBSTACLE_POSITION = (1.5, 0.0)   # (x, y) of the box centre in world metres
OBSTACLE_SIZE = (0.4, 0.6, 1.0)  # full lengths along (x, y, z), in metres
# The box rests on the floor. Decide motion from sensor readings, not these values.
D_IN = 0.65              # m; interrupt the nominal command
D_OUT = 0.8              # m; authorize restarting, greater than D_IN
TURN_ANGLE = math.pi / 2   # rad; isolated-box scene
BYPASS_DISTANCE = 1.0      # m; offset of the intermediate goal from the stop pose
# Nominal gains, goal tolerance and WHEEL_MAX are reused from lesson03/control.py.


class StudentSectors(SectorSummary):
    def summarize(self, angles, ranges, hits, max_range):
        """Return dict(front,left,right) of minima.

        Assume valid data with all three sectors covered.
        Front: |bearing|<=30 degrees. Misses observe to max_range.
        """
        # TODO: implement the behavior described above.
        raise NotImplementedError(
            "Complete StudentSectors.summarize in exercises/lesson04/avoidance.py"
        )


class StudentObstacleSupervisor(ObstacleSupervisor):
    def command(self, pose, goal, sectors):
        """Return (v,omega), updating self.phase; assume valid sector readings.

        self.goal_command(pose, active_goal) reuses the Lesson 3 pose controller.
        Goal is (x, y, yaw). self.phase starts at GO; use GO, STOP, DETOUR, DONE.
        Use self.d_in and self.d_out, self.turn_angle and self.bypass_distance.
        In STOP choose the side with the larger minimum (left on a tie), then enter
        DETOUR without a lateral distance threshold. Save one fixed (x, y, yaw)
        in self.detour_goal.
        In DETOUR pass that pose to goal_command; do not compute new velocities.
        After that call, self.goal_controller_done reports arrival at the active pose.
        Intermediate arrival returns to GO; only final-pose arrival means DONE.
        The adapter resets the same pose controller when the target changes.

        self.advance_allowed starts False for each detour. Set it False below
        self.d_in, True at or above self.d_out, and retain it between thresholds.
        If the proposed v is positive while advance is blocked, return (0, omega).
        Allow pure rotation toward the intermediate pose even with an obstructed
        front; only the front distance authorizes forward motion.

        The stop stage reuses StudentRangeStop from rangefinder.py on the front
        sector. This method is called only in avoid, to add the detour.
        Set self.phase = 'DONE' and return zero only within both pose tolerances.
        Keep the Lesson 3 final-heading alignment; timeouts are external.
        The supplied adapter converts/saturates body commands for the wheels;
        it does not call or replace your decision method.
        """
        # TODO: implement the behavior described above.
        raise NotImplementedError(
            "Complete StudentObstacleSupervisor.command in exercises/lesson04/avoidance.py"
        )
