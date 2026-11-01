"""Check feedback against correct, incomplete, and plausible-but-wrong solutions."""

import numpy as np
import pytest

from robotics_sim.exercise_checks import ExerciseChecks
from robotics_sim.session import ActivitySession, TASKS


@pytest.mark.parametrize("lesson,task", [(lesson, task) for lesson in ("3", "4", "5") for task in TASKS[lesson]])
def test_every_runtime_exercise_has_checks_without_evaluating_the_model(lesson, task):
    session = ActivitySession(lesson, task, duration=.2, dt=.01, kinematics="student", exercise_model="student")
    try:
        checks = ExerciseChecks(session).evaluate()
        assert checks
        assert not all(passed for _, passed in checks)
        assert session.sim.steps == 0
    finally:
        session.close()


@pytest.mark.parametrize("mode", ["ideal", "noisy"])
@pytest.mark.parametrize("task", ["sensor_estimation_straight", "sensor_estimation_curve", "sensor_estimation_blocked"])
def test_estimation_uses_same_stream_even_when_ui_skips_samples(task, mode):
    session = ActivitySession("4", task, duration=.4, dt=.01, sensor_mode=mode)
    checks = ExerciseChecks(session)
    try:
        session.start()
        while session.state != "Completed":
            session.step()
        assert all(passed for _, passed in checks.evaluate())
        # Repeated UI refreshes must not integrate the same measurements twice.
        assert all(passed for _, passed in checks.evaluate())
        session.laboratory.odom.x += .1
        assert [passed for _, passed in checks.evaluate()] == [True, False, True]
    finally:
        session.close()


@pytest.mark.parametrize("lesson,task,duration", [
    ("3", "drive_inverse", .2), ("3", "drive_proportional", 20),
    ("3", "drive_waypoints", 40), ("4", "sensor_lidar", .2),
    ("4", "sensor_avoidance", 40), ("5", "arm_forward", .2),
    ("5", "arm_inverse", .2), ("5", "arm_control", 20),
])
def test_reference_runs_pass_only_after_evaluation(lesson, task, duration):
    session = ActivitySession(lesson, task, duration=duration, dt=.01)
    try:
        checks = ExerciseChecks(session)
        assert not all(passed for _, passed in checks.evaluate())
        session.start()
        while session.state != "Completed":
            session.step()
        assert all(passed for _, passed in checks.evaluate()), checks.evaluate()
    finally:
        session.close()


def test_wheel_checks_reject_mutually_consistent_wrong_formulas(monkeypatch):
    session = ActivitySession("3", "drive_inverse", duration=.02, dt=.01)
    try:
        session.start()
        while session.state != "Completed":
            session.step()
        model = session.robot.motor_model
        monkeypatch.setattr(model, "forward", lambda *w: (0, 0))
        monkeypatch.setattr(model, "inverse", lambda *v: (0, 0))
        assert [passed for _, passed in ExerciseChecks(session).evaluate()] == [True, False, False]
    finally:
        session.close()


@pytest.mark.parametrize("task", ["drive_proportional", "arm_control", "sensor_avoidance"])
def test_timeout_is_not_a_pass(task):
    lesson = "3" if task.startswith("drive") else "5" if task.startswith("arm") else "4"
    session = ActivitySession(lesson, task, duration=.01, dt=.01)
    try:
        session.step(single=True)
        assert session.outcome == "TIMEOUT"
        assert not all(passed for _, passed in ExerciseChecks(session).evaluate())
    finally:
        session.close()


def test_planar_checks_reject_translation_applied_to_a_direction():
    session = ActivitySession("2", "planar")
    try:
        scene = session.planar_scene
        transform = scene.world_from_robot() @ scene.robot_from_sensor
        point = (transform @ np.r_[scene.point_in_sensor, 1])[:2]
        direction = transform[:2, :2] @ scene.direction_in_sensor
        checks = ExerciseChecks(session)
        assert all(passed for _, passed in checks.evaluate(planar_output=(point, direction)))
        assert [passed for _, passed in checks.evaluate(planar_output=(point, direction + transform[:2, 2]))] == [True, False]
        assert not any(passed for _, passed in checks.evaluate())
    finally:
        session.close()
