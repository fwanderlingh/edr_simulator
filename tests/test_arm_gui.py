"""Headless checks for the arm activity embedded in the desktop session."""

import numpy as np
import pytest

from robotics_sim import World
from robotics_sim.exercises.lesson05.gui import ArmActivity


def test_student_activity_loads_before_attempting_exercise():
    with World(gui=False) as world:
        activity = ArmActivity(world, "arm_forward", "student")
        assert not activity.done and not activity.evaluated
        np.testing.assert_allclose(activity.points[-1, :2], activity.plant.forward(activity.q))
        with pytest.raises(NotImplementedError, match="forward"):
            activity.step(1 / 240)
        assert activity.time == 0 and not activity.evaluated


def test_forward_and_inverse_results_remain_for_inspection():
    with World(gui=False) as world:
        forward = ArmActivity(world, "arm_forward")
        forward.step(1 / 240)
        assert forward.done and forward.outcome == "OK"
        assert forward.model_error < 1e-12
        inverse = ArmActivity(world, "arm_inverse")
        inverse.step(1 / 240)
        assert inverse.done and inverse.outcome == "OK"
        assert len(inverse.branches) == 2
        for branch in inverse.branches:
            np.testing.assert_allclose(branch[-1, :2], (1, 1), atol=1e-12)
        old_points = inverse.points.copy()
        inverse.step(1 / 240)
        np.testing.assert_array_equal(inverse.points, old_points)


@pytest.mark.parametrize("target,status", [((3, 0), "UNREACHABLE"), ((0, 0), "DEGENERATE")])
def test_geometric_special_cases_are_explicit(target, status):
    with World(gui=False) as world:
        activity = ArmActivity(world, "arm_inverse", target=target)
        activity.step(1 / 240)
        assert activity.outcome == status


def test_reference_control_reaches_target_with_limits_and_csv_contract():
    with World(gui=False) as world:
        activity = ArmActivity(world, "arm_control")
        for _ in range(2400):
            activity.step(1 / 240)
            assert np.max(np.abs(activity.dq)) <= 1 + 1e-12
            assert np.max(np.abs(activity.q)) <= np.pi
            if activity.done:
                break
        assert activity.outcome == "DONE"
        assert activity.residual <= 1e-3
        assert len(activity.extra_columns) == len(activity.extra_values)
        np.testing.assert_array_equal(activity.dq, (0, 0))


def test_timeout_is_separate_and_stops_joint_commands():
    with World(gui=False) as world:
        activity = ArmActivity(world, "arm_control")
        activity.step(1 / 240)
        assert np.max(np.abs(activity.dq)) > 0
        activity.timeout()
        assert activity.done and activity.outcome == "TIMEOUT"
        np.testing.assert_array_equal(activity.dq, (0, 0))


def test_broken_inverse_is_not_misreported_as_unreachable():
    with World(gui=False) as world:
        activity = ArmActivity(world, "arm_inverse")
        activity.model.inverse = lambda target: []
        activity.step(1 / 240)
        assert activity.outcome == "CHECK"
