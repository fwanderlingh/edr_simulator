"""First perception/control exercise: stop before an obstacle using one front ray.

Complete command(), save and click Reload exercise. Choose Lesson 4 >
Reach a goal: front rangefinder stop > student. Pose comes from the simulator;
the sensor measures the environment, not the robot position.
Reference: robotics_sim/sim/references/rangefinder.py.
"""

from ._runtime import ObstacleSupervisor


ENTER_DISTANCE = 0.65  # m, measured from the sensor/robot centre
EXIT_DISTANCE = 0.8    # m, greater than ENTER_DISTANCE (hysteresis)
# Scene settings, shared by student and reference modes. Reload after editing.
OBSTACLE_POSITION = (1.5, 0.0)   # (x, y) of the box centre in world metres
OBSTACLE_SIZE = (0.4, 0.6, 1.0)  # full lengths along (x, y, z), in metres
# The box rests on the floor. Decide motion from sensor readings, not these values.
# Nominal gains, position tolerance and WHEEL_MAX come from lesson03/control.py.


class StudentRangeStop(ObstacleSupervisor):
    def command(self, pose, goal, distance, hit, max_range):
        """Return (v, omega), with v in m/s and omega in rad/s.

        Sensor data are assumed always available and valid.
        1. A miss (hit=False) observes only to max_range; it is not a hit.
        2. Use self.enter/exit and keep STOP until the release threshold is met.
        3. Inside self.eps_pos return zero and set self.phase = 'DONE'.
        4. Otherwise pass self.goal_command(pose, goal), reusing Lesson 3.

        State starts at GO. Call self.reset_goal_controller() when the release
        threshold authorizes restarting after STOP.
        One ray supports a front stop in the supplied straight scene; it does
        not provide side observations for choosing an avoidance manoeuvre.
        """
        # TODO: implement the behavior described above.
        raise NotImplementedError(
            "Complete StudentRangeStop.command in exercises/lesson04/rangefinder.py"
        )
