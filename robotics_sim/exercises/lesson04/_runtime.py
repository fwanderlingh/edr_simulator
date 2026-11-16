"""Supplied state and wheel/body adapter; student algorithms live beside this file."""

import math


class OdometryState:
    def __init__(self, radius=0.12, width=0.42):
        self.radius, self.width = radius, width
        self.x = self.y = self.yaw = 0.0
        self.speed = self.yaw_rate = 0.0


class InertialState:
    def __init__(self):
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = 0.0


class SectorSummary:
    pass


class PoseCommandAdapter:
    """Reuse Lesson 3 wheel_command unchanged for a full (x, y, yaw) goal.

    Preserve TURN, DRIVE and final-heading ALIGN. Reset the pose controller
    only for a new target pose or after an interruption, never on every sample.
    """

    def __init__(self, controller, radius=0.12, width=0.42):
        self.controller = controller
        self.radius, self.width = radius, width
        self.target = None

    def reset(self):
        self.target = None

    def command(self, pose, goal):
        target = tuple(goal)
        if target != self.target:
            self.controller.set_goal(target)
            self.target = target
        left, right = self.controller.wheel_command(pose, self.radius, self.width)
        if not all(math.isfinite(w) for w in (left, right)):
            raise ValueError("Lesson 3 wheel_command must return finite wheel references.")
        # The existing Lesson 3 method already applied its common saturation.
        # Express those wheels as a body command for the supervisor interface.
        return (self.radius * (right + left) / 2,
                self.radius * (right - left) / self.width)


class ObstacleSupervisor:
    """Initial state, parameters and nominal-controller adapter, without the solution."""

    def __init__(self, *, pose_controller=None, radius=0.12, width=0.42,
                 enter=0.65, exit=0.8,
                 turn_angle=math.pi / 2, turn_tolerance=0.04,
                 turn_gain=1.5, bypass_distance=1.0, bypass_speed=0.25,
                 stage="avoid"):
        if pose_controller is None:
            from robotics_sim.sim.references.control import ReferencePoseController
            pose_controller = ReferencePoseController()
        positive = (radius, width, enter, exit, turn_angle,
                    turn_tolerance, turn_gain, bypass_distance, bypass_speed)
        if (not all(math.isfinite(x) and x > 0 for x in positive)
                or exit <= enter or stage not in ("sectors", "stop", "avoid")):
            raise ValueError("Require positive finite supervisor parameters, exit > enter and a valid stage.")
        self.nominal = PoseCommandAdapter(pose_controller, radius, width)
        self.eps_pos = pose_controller.eps_pos
        self.eps_angle = pose_controller.eps_angle
        self.wheel_max = pose_controller.wheel_max
        self.enter, self.exit = enter, exit
        self.turn_angle, self.turn_tolerance, self.turn_gain = turn_angle, turn_tolerance, turn_gain
        self.bypass_distance, self.bypass_speed = bypass_distance, bypass_speed
        self.stage = stage
        self.phase = "GO"
        self.turn_yaw = 0.0
        self.pass_start = (0.0, 0.0)

    def goal_command(self, pose, goal):
        """Call the Lesson 3 controller; no new go-to-goal law is introduced here."""
        return self.nominal.command(pose, goal)

    def reset_goal_controller(self):
        """Re-evaluate TURN/DRIVE after an interruption or a completed detour."""
        self.nominal.reset()


def supervisor_parameters():
    """Read the same editable settings for student and reference activities."""
    from . import avoidance
    return {name: getattr(avoidance, constant) for name, constant in (
        ("enter", "ENTER_DISTANCE"), ("exit", "EXIT_DISTANCE"),
        ("turn_angle", "TURN_ANGLE"),
        ("turn_tolerance", "TURN_TOLERANCE"), ("turn_gain", "TURN_GAIN"),
        ("bypass_distance", "BYPASS_DISTANCE"), ("bypass_speed", "BYPASS_SPEED"))}


def rangefinder_parameters():
    from . import rangefinder
    return dict(enter=rangefinder.ENTER_DISTANCE, exit=rangefinder.EXIT_DISTANCE)
