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
                 d_in=0.65, d_out=0.8,
                 turn_angle=math.pi / 2, bypass_distance=1.0,
                 stage="avoid"):
        if pose_controller is None:
            from robotics_sim.sim.references.control import ReferencePoseController
            pose_controller = ReferencePoseController()
        positive = (radius, width, d_in, d_out, turn_angle, bypass_distance)
        if (not all(math.isfinite(x) and x > 0 for x in positive)
                or d_out <= d_in or stage not in ("sectors", "stop", "avoid")):
            raise ValueError("Require positive finite supervisor parameters, d_out > d_in and a valid stage.")
        self.nominal = PoseCommandAdapter(pose_controller, radius, width)
        self.eps_pos = pose_controller.eps_pos
        self.eps_angle = pose_controller.eps_angle
        self.wheel_max = pose_controller.wheel_max
        self.d_in, self.d_out = d_in, d_out
        self.turn_angle, self.bypass_distance = turn_angle, bypass_distance
        self.stage = stage
        self.phase = "GO"
        self.detour_goal = None
        self.advance_allowed = False

    def goal_command(self, pose, goal):
        """Call the Lesson 3 controller; no new go-to-goal law is introduced here."""
        return self.nominal.command(pose, goal)

    @property
    def goal_controller_done(self):
        """After goal_command, report arrival at that controller's active pose."""
        return self.nominal.controller.done

    def reset_goal_controller(self):
        """Re-evaluate TURN/DRIVE after an interruption or a completed detour."""
        self.nominal.reset()


def supervisor_parameters():
    """Read the same editable settings for student and reference activities."""
    from . import avoidance
    return {name: getattr(avoidance, constant) for name, constant in (
        ("d_in", "D_IN"), ("d_out", "D_OUT"),
        ("turn_angle", "TURN_ANGLE"), ("bypass_distance", "BYPASS_DISTANCE"))}


def rangefinder_parameters():
    from . import rangefinder
    return dict(d_in=rangefinder.D_IN, d_out=rangefinder.D_OUT)
