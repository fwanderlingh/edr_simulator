"""Guided reactive supervisor for one isolated obstacle; this is not a planner.

Choose Lesson 4 > Reach a goal and avoid an obstacle, select the student
supervisor, then save your implementation and click Reload exercise.
The student sector summary and supervisor command are deliberately unfinished.
"""

from robotics_sim.sim.references.sensors import ReferenceObstacleSupervisor, ReferenceSectors


class StudentSectors(ReferenceSectors):
    def summarize(self, angles, ranges, hits, max_range):
        """Return dict(front,left,right) of minima, or None for invalid/missing data.

        Front: |bearing|<=30 degrees. Misses are valid observations to max_range.
        """
        raise NotImplementedError("Complete StudentSectors.summarize in lesson04/avoidance.py")


class StudentObstacleSupervisor(ReferenceObstacleSupervisor):
    def command(self, pose, goal, sectors, age):
        """Return (v,omega), updating self.phase; stale data must stop the robot.

        Keep a turn direction and bypass progress; do not immediately point back
        at the box after a clear scan. Finishing and timeout are separate outcomes.
        """
        raise NotImplementedError("Complete StudentObstacleSupervisor.command in lesson04/avoidance.py")
