"""Instructor reference models; consult after attempting the guided exercises."""

import math
import numpy as np


class ReferencePlanarArm:
    def __init__(self, l1=1.0, l2=1.0):
        if not all(math.isfinite(x) and x > 0 for x in (l1, l2)):
            raise ValueError("Link lengths must be finite and positive.")
        self.l1, self.l2 = l1, l2

    def forward(self, q):
        q1, q2 = q
        return np.array([self.l1*math.cos(q1) + self.l2*math.cos(q1+q2),
                         self.l1*math.sin(q1) + self.l2*math.sin(q1+q2)])

    def inverse(self, target):
        x, y = target
        radius = math.hypot(x, y)
        if not abs(self.l1-self.l2)-1e-12 <= radius <= self.l1+self.l2+1e-12:
            return []
        if radius < 1e-12 and abs(self.l1-self.l2) < 1e-12:
            raise ValueError("Folded equal-link arm: infinitely many q1 solutions.")
        c2 = np.clip((radius**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2), -1, 1)
        solutions = []
        for sign in (1, -1):
            s2 = sign*math.sqrt(max(0.0, 1-c2*c2))
            q2 = math.atan2(s2, c2)
            q1 = math.atan2(y, x)-math.atan2(self.l2*s2, self.l1+self.l2*c2)
            candidate = np.array([q1, q2])
            if not any(np.linalg.norm((candidate-previous+math.pi) % (2*math.pi)-math.pi)
                       < 1e-9 for previous in solutions):
                solutions.append(candidate)
        return solutions

    def jacobian(self, q):
        q1, q2 = q
        s1, c1 = math.sin(q1), math.cos(q1)
        s12, c12 = math.sin(q1+q2), math.cos(q1+q2)
        return np.array([[-self.l1*s1-self.l2*s12, -self.l2*s12],
                         [self.l1*c1+self.l2*c12, self.l2*c12]])
class ReferenceArmController:
    def __init__(self, model, *, gain=1.5, damping=0.05, max_speed=1.0):
        if not all(math.isfinite(x) and x > 0 for x in (gain, damping, max_speed)):
            raise ValueError("Gain, damping and speed limit must be positive.")
        self.model = model
        self.gain, self.damping, self.max_speed = gain, damping, max_speed

    def velocity(self, q, target):
        error = np.asarray(target)-self.model.forward(q)
        jac = self.model.jacobian(q)
        velocity = jac.T @ np.linalg.solve(
            jac @ jac.T + self.damping**2*np.eye(2), self.gain*error)
        peak = np.max(np.abs(velocity))
        return velocity*min(1.0, self.max_speed/peak) if peak else velocity


