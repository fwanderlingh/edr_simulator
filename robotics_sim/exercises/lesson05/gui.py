"""Planar-arm exercises embedded in the shared desktop activity lifecycle.

The student model is evaluated only when Start/Step is used. Merely selecting
an unfinished exercise therefore leaves the scene available for inspection.
"""

import math

import numpy as np

from robotics_sim import Pose, Velocity
from .control import StudentArmController
from .kinematics import StudentPlanarArm
from .reference import ReferenceArmController, ReferencePlanarArm


DEFAULT_INITIAL = (0.2, 0.5)
DEFAULT_TARGET = (1.0, 1.0)


def finite_vector(value, size, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (size,) or not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must contain {size} finite values.")
    return value


class ArmBase:
    """Fixed mounting frame used by the session's common pose/stop interfaces."""

    def __init__(self, world):
        self.world = world
        self.body_id = world.add_box(Pose(z=0.07), size=(0.18, 0.18, 0.14),
                                     color=(0.16, 0.24, 0.31, 1))
        self.command = Velocity()

    def pose(self):
        return Pose(z=0.15)

    def set_velocity(self, vx=0.0, vy=0.0, omega=0.0, *, frame="body"):
        if vx or vy or omega:
            raise ValueError("The arm exercise has a fixed base.")
        self.command = Velocity(frame=frame)

    def apply_command(self):
        pass


class ArmActivity:
    """Forward, inverse and endpoint-control exercises for a fixed 2R arm.

    ``step`` advances only the kinematic activity; ActivitySession owns the
    shared physics clock and calls ``world.physics.step`` exactly once.
    """

    tasks = ("arm_forward", "arm_inverse", "arm_control")
    extra_columns = ("q1_rad", "q2_rad", "tip_x_m", "tip_y_m", "target_x_m",
                     "target_y_m", "residual_m", "q1_dot_rad_s", "q2_dot_rad_s",
                     "model_error_m", "branch_count", "outcome")
    height = 0.2

    def __init__(self, world, task, model="reference", *, initial=DEFAULT_INITIAL,
                 target=DEFAULT_TARGET, damping=0.05, max_speed=1.0):
        if task not in self.tasks:
            raise ValueError("Choose a forward, inverse or control arm activity.")
        if model not in ("reference", "student"):
            raise ValueError("Choose model reference or student.")
        self.world, self.task, self.model_name = world, task, model
        self.q = finite_vector(initial, 2, "Initial joint configuration").copy()
        if np.any(np.abs(self.q) > math.pi):
            raise ValueError("Initial joints must be within [-pi, pi].")
        self.target_xy = finite_vector(target, 2, "Target").copy()
        self.plant = ReferencePlanarArm()
        self.model = StudentPlanarArm() if model == "student" else ReferencePlanarArm()
        controller = StudentArmController if model == "student" else ReferenceArmController
        self.controller = controller(self.model, damping=damping, max_speed=max_speed)
        self.robot = ArmBase(world)
        self.dq = np.zeros(2)
        self.time = 0.0
        self.outcome = "RUNNING"
        self.evaluated = False
        self.model_error = None
        self.predictions = ()
        self.branches = ()
        self.branch_solutions = ()

    @property
    def done(self):
        return self.outcome != "RUNNING"

    @property
    def target(self):
        return (*self.target_xy, self.height)

    def chain(self, q):
        """Independent plant geometry, never the student's forward result."""
        q1, _ = q
        return np.array([(0.0, 0.0, self.height),
                         (self.plant.l1 * math.cos(q1), self.plant.l1 * math.sin(q1), self.height),
                         (*self.plant.forward(q), self.height)])

    @property
    def points(self):
        return self.chain(self.q)

    @property
    def residual(self):
        return float(np.linalg.norm(self.target_xy - self.plant.forward(self.q)))

    @property
    def extra_values(self):
        tip = self.plant.forward(self.q)
        return (*self.q, *tip, *self.target_xy, self.residual, *self.dq,
                "" if self.model_error is None else self.model_error,
                len(self.branches), self.outcome)

    @property
    def readout(self):
        tip = self.plant.forward(self.q)
        lines = [f"q1 / q2: {self.q[0]:.3f} / {self.q[1]:.3f} rad",
                 f"Tip: ({tip[0]:.3f}, {tip[1]:.3f}) m",
                 f"Target: ({self.target_xy[0]:.3f}, {self.target_xy[1]:.3f}) m"]
        if not self.evaluated:
            lines.append("Press Start or Step to evaluate the selected model.")
        elif self.task == "arm_forward":
            lines.append(f"Forward error: {self.model_error:.6f} m | {self.outcome}")
        elif self.task == "arm_inverse":
            lines.append(f"Branches: {len(self.branches)} | {self.outcome}")
            for index, q in enumerate(self.branch_solutions):
                error = np.linalg.norm(self.target_xy - self.plant.forward(q))
                lines.append(f"Branch {index + 1}: {error:.6f} m")
        else:
            lines.append(f"Residual: {self.residual:.6f} m | {self.outcome}")
            lines.append(f"Joint speeds: {self.dq[0]:.3f} / {self.dq[1]:.3f} rad/s")
        return "\n".join(lines)

    def stop(self):
        self.dq = np.zeros(2)
        self.robot.set_velocity()

    def timeout(self):
        if not self.done:
            self.outcome = "TIMEOUT"
        self.stop()

    def step(self, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("The activity timestep must be positive and finite.")
        if self.done:
            return
        # Do not mark the exercise evaluated before student code has succeeded.
        if self.task == "arm_forward":
            self._forward()
        elif self.task == "arm_inverse":
            self._inverse()
        else:
            self._control(dt)
        self.evaluated = True
        self.time += dt

    def _forward(self):
        prediction = finite_vector(self.model.forward(self.q), 2, "Forward endpoint")
        self.predictions = ((*prediction, self.height),)
        self.model_error = float(np.linalg.norm(prediction - self.plant.forward(self.q)))
        self.outcome = "OK" if self.model_error <= 1e-6 else "CHECK"

    def _inverse(self):
        degenerate = (np.linalg.norm(self.target_xy) < 1e-12 and
                      abs(self.plant.l1 - self.plant.l2) < 1e-12)
        try:
            solutions = self.model.inverse(self.target_xy)
        except ValueError:
            if not degenerate:
                raise
            self.outcome = "DEGENERATE"
            return
        if degenerate:
            self.outcome = "CHECK"
            return
        expected = self.plant.inverse(self.target_xy)
        solutions = tuple(finite_vector(q, 2, "IK solution") for q in solutions)
        self.branch_solutions = solutions
        self.branches = tuple(self.chain(q) for q in solutions)
        if not solutions:
            self.outcome = "CHECK" if expected else "UNREACHABLE"
            return
        errors, predictions, seen = [], [], []
        valid = len(solutions) == len(expected)
        for q in solutions:
            actual = self.plant.forward(q)
            prediction = finite_vector(self.model.forward(q), 2, "Forward endpoint")
            errors.append(float(np.linalg.norm(prediction - actual)))
            predictions.append((*prediction, self.height))
            duplicate = any(np.linalg.norm((q - old + math.pi) % (2 * math.pi) - math.pi)
                            < 1e-6 for old in seen)
            seen.append(q)
            valid = valid and np.linalg.norm(actual - self.target_xy) <= 1e-6 and not duplicate
        self.q = solutions[0].copy()
        self.predictions = tuple(predictions)
        self.model_error = max(errors)
        self.outcome = "OK" if valid and self.model_error <= 1e-6 else "CHECK"

    def _control(self, dt):
        radius = np.linalg.norm(self.target_xy)
        if not abs(self.plant.l1 - self.plant.l2) <= radius <= self.plant.l1 + self.plant.l2 + 1e-12:
            self.outcome = "UNREACHABLE"
            self.stop()
            return
        if self.residual <= 1e-3:
            self.outcome = "DONE"
            self.stop()
            return
        velocity = finite_vector(self.controller.velocity(self.q, self.target_xy), 2, "Joint velocity")
        if np.max(np.abs(velocity)) > self.controller.max_speed + 1e-9:
            raise ValueError("The controller exceeded its configured joint-speed limit.")
        candidate = self.q + dt * velocity
        if np.any(np.abs(candidate) > math.pi):
            self.outcome = "JOINT_LIMIT"
            self.stop()
            return
        self.q, self.dq = candidate, velocity
        if self.residual <= 1e-3:
            self.outcome = "DONE"
            self.stop()
