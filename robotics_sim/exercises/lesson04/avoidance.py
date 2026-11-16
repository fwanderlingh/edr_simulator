"""Guided reactive supervisor for one isolated obstacle; this is not a planner.

Choose Lesson 4 > Reach a goal and avoid an obstacle, select the student
supervisor, then save your implementation and click Reload exercise.
The student sector summary and supervisor command are deliberately unfinished.
Reference: robotics_sim/sim/references/avoidance.py.
"""

import math

from ._runtime import ObstacleSupervisor, SectorSummary


AVOIDANCE_STAGE = "avoid"  # "sectors", "stop" (reuse rangefinder.py), or "avoid"
# Use "student" here after implementing the ray directions in lidar.py.
# This choice is independent of ideal/noisy measurement mode.
LIDAR_MODEL = "reference"  # "reference" or "student"
# Scene settings for all LiDAR stages, shared by student and reference modes.
OBSTACLE_POSITION = (1.5, 0.0)   # (x, y) of the box centre in world metres
OBSTACLE_SIZE = (0.4, 0.6, 1.0)  # full lengths along (x, y, z), in metres
# The box rests on the floor. Decide motion from sensor readings, not these values.
ENTER_DISTANCE = 0.65      # m; interrupt the nominal command
EXIT_DISTANCE = 0.8        # m; authorize restarting, greater than ENTER_DISTANCE
TURN_ANGLE = math.pi / 2   # rad; isolated-box scene
TURN_TOLERANCE = 0.04      # rad
TURN_GAIN = 1.5            # 1/s
BYPASS_DISTANCE = 1.0      # m; isolated-box scene
BYPASS_SPEED = 0.25        # m/s
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

        self.goal_command(pose, goal) calls the completed Lesson 3 pose controller.
        Use self.eps_pos, self.enter/exit, self.turn_angle,
        self.turn_tolerance/gain and self.bypass_distance/speed.
        self.phase starts at GO; save turn_yaw and pass_start for the detour.
        reset_goal_controller() restarts the nominal controller after interruption.

        The stop stage reuses StudentRangeStop from rangefinder.py on the front
        sector. This method is called only in avoid, to add the detour.
        Return DONE and zero motion only inside self.eps_pos; timeouts are external.
        The supplied adapter converts/saturates body commands for the wheels;
        it does not call or replace your decision method.
        """
        # TODO: implement the behavior described above.
        raise NotImplementedError(
            "Complete StudentObstacleSupervisor.command in exercises/lesson04/avoidance.py"
        )
