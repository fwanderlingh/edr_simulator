"""First perception/control exercise: stop before an obstacle using one front ray.

Complete command(), save and click Reload exercise. Choose Lesson 4 >
Reach a goal: front rangefinder stop > student. Pose comes from the simulator;
the sensor measures the environment, not the robot position.
"""

from ._runtime import ObstacleSupervisor


ENTER_DISTANCE = 0.65  # m, measured from the sensor/robot centre
EXIT_DISTANCE = 0.8    # m, greater than ENTER_DISTANCE (hysteresis)
MAX_SCAN_AGE = 0.25    # s
# Nominal gains, position tolerance and WHEEL_MAX come from lesson03/control.py.


class StudentRangeStop(ObstacleSupervisor):
    def command(self, pose, goal, distance, hit, max_range, age):
        """Return (v, omega), with v in m/s and omega in rad/s.

        1. Validate finite distance/range/age and stop on invalid or stale data.
        2. A valid miss (hit=False) observes only to max_range; it is not a hit.
        3. Use self.enter/exit and keep STOP until the release threshold is met.
        4. Inside self.eps_pos return zero and set self.phase = 'DONE'.
        5. Otherwise pass self.goal_command(pose, goal), reusing Lesson 3.

        State starts at GO. Use self.max_scan_age for freshness and
        self.reset_goal_controller() when fresh data authorize restarting.
        One ray supports a front stop in the supplied straight scene; it does
        not provide side observations for choosing an avoidance manoeuvre.
        """
        raise NotImplementedError("Complete StudentRangeStop.command in lesson04/rangefinder.py")
