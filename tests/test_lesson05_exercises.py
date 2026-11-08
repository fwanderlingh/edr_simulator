"""Check arm geometry and closed-loop outcomes against independent expectations."""

import csv

import numpy as np
import pytest

from robotics_sim.exercises.lesson05 import gui
from robotics_sim.session import ActivitySession
from robotics_sim.sim.references.planar_arm import ReferenceArmController, ReferencePlanarArm


def test_two_branches_and_inner_boundary_are_distinct_modulo_angles():
    model = ReferencePlanarArm()
    solutions = model.inverse((1, 1))
    assert len(solutions) == 2
    for q in solutions:
        assert model.forward(q) == pytest.approx((1, 1))
    assert model.inverse((3, 0)) == []
    with pytest.raises(ValueError, match="infinitely"):
        model.inverse((0, 0))
    inner = ReferencePlanarArm(2, 1).inverse((1, 0))
    assert len(inner) == 1


def test_jacobian_matches_finite_differences_and_limits():
    model = ReferencePlanarArm()
    q = np.array([0.4, -0.6])
    delta = 1e-6
    numeric = np.column_stack([(model.forward(q + delta * basis) -
                                model.forward(q - delta * basis)) / (2 * delta)
                               for basis in np.eye(2)])
    np.testing.assert_allclose(model.jacobian(q), numeric, atol=1e-8)
    ctrl = ReferenceArmController(model, max_speed=0.2)
    assert np.max(np.abs(ctrl.velocity(q, (-1, 1)))) <= 0.2 + 1e-12


@pytest.mark.parametrize("changes,status", [
    ({"target": (3, 0)}, "UNREACHABLE"),
    ({"initial": (0, 0), "target": (1, 0), "duration": 0.1}, "TIMEOUT"),
])
def test_control_outcomes_and_final_zero_speed(tmp_path, changes, status):
    target = changes.get("target", (1, 1))
    session = ActivitySession("5", "arm_control", goal=(*target, 0), dt=1 / 120,
                              duration=changes.get("duration", 10), exercise_model="reference")
    try:
        if "initial" in changes:
            session.laboratory.q = np.asarray(changes["initial"], dtype=float)
        session.start()
        while session.state == "Running":
            session.step()
        assert session.outcome == status
        path = tmp_path / "arm.csv"
        session.export_csv(path)
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        assert rows[-1]["outcome"] == status
        assert float(rows[-1]["q1_dot_rad_s"]) == float(rows[-1]["q2_dot_rad_s"]) == 0
        assert all(max(abs(float(row["q1_dot_rad_s"])), abs(float(row["q2_dot_rad_s"])))
                   <= session.laboratory.controller.max_speed + 1e-12 for row in rows)
    finally:
        session.close()


def test_missing_reachable_branch_is_check_not_unreachable(monkeypatch):
    class IncompleteInverse(ReferencePlanarArm):
        def inverse(self, target):
            return []
    monkeypatch.setattr(gui, "StudentPlanarArm", IncompleteInverse)
    session = ActivitySession("5", "arm_inverse", exercise_model="student")
    try:
        session.step(single=True)
        assert session.outcome == "CHECK"
    finally:
        session.close()
