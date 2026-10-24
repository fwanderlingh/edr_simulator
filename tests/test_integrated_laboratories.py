"""Exercise the practical GUI activities through their shared headless session."""

import csv
from unittest.mock import patch

import pytest

from robotics_sim.session import ActivitySession, TASKS
from robotics_sim.exercises.lesson04.estimation import StudentOdometry
from robotics_sim.exercises.lesson05.kinematics import StudentPlanarArm


def finish(session):
    session.start()
    while session.state == "Running":
        session.step()


def rows(path):
    with path.open(encoding="utf-8") as stream:
        result = list(csv.DictReader(stream))
    assert result and all(None not in row for row in result)
    return result


@pytest.mark.parametrize("task", ["sensor_estimation_straight", "sensor_estimation_curve"])
def test_encoder_imu_comparison_uses_shared_clock_and_export(task, tmp_path):
    session = ActivitySession("4", task, duration=2, dt=1 / 120, exercise_model="reference")
    try:
        assert task in TASKS["4"] and session.state == "Ready"
        finish(session)
        assert session.state == "Completed"
        assert session.outcome == "COMPLETED"
        assert session.sim.time == pytest.approx(2)
        assert session.sim.steps == len(session.samples) == 240
        path = tmp_path / "comparison.csv"
        session.export_csv(path)
        pose_rows = rows(path)
        sensor_rows = rows(tmp_path / "comparison_sensors.csv")
        assert float(pose_rows[-1]["enc_position_error_m"]) < 0.005
        assert float(pose_rows[-1]["imu_position_error_m"]) < 0.04
        assert float(sensor_rows[-1]["time_s"]) == pytest.approx(2)
        assert len(sensor_rows) < len(pose_rows)
    finally:
        session.close()


def test_blocked_encoder_exercise_reveals_disagreement():
    session = ActivitySession("4", "sensor_estimation_blocked", duration=8,
                              dt=1 / 120, exercise_model="reference")
    try:
        finish(session)
        comparison = dict(zip(session.laboratory.extra_columns, session.laboratory.extra_values))
        assert comparison["enc_position_error_m"] > 0.3
        assert comparison["imu_position_error_m"] < 0.05
        assert session.robot.wheel_velocities == (0, 0)
    finally:
        session.close()


def test_avoidance_is_available_in_catalog_and_reaches_goal(tmp_path):
    session = ActivitySession("4", "sensor_avoidance", duration=35, exercise_model="reference")
    try:
        finish(session)
        assert session.outcome == "DONE"
        assert session.laboratory.done
        assert session.sim.time < 35
        assert session.robot.wheel_velocities == (0, 0)
        path = tmp_path / "avoidance.csv"
        session.export_csv(path)
        data = rows(path)
        assert float(data[-1]["goal_error_m"]) <= 0.05
        assert data[-1]["outcome"] == "DONE"
    finally:
        session.close()


@pytest.mark.parametrize("task,expected", [("arm_forward", "OK"), ("arm_inverse", "OK"),
                                           ("arm_control", "DONE")])
def test_arm_catalog_evaluates_stays_open_and_exports(task, expected, tmp_path):
    session = ActivitySession("5", task, duration=10, exercise_model="reference")
    try:
        assert task in TASKS["5"] and session.state == "Ready"
        finish(session)
        assert session.state == "Completed" and session.outcome == expected
        assert session.world.physics.connected
        path = tmp_path / "arm.csv"
        session.export_csv(path)
        data = rows(path)
        assert data[-1]["outcome"] == expected
        assert float(data[-1]["q1_dot_rad_s"]) == float(data[-1]["q2_dot_rad_s"]) == 0
        if task == "arm_control":
            assert float(data[-1]["residual_m"]) <= 0.001
        elif task == "arm_inverse":
            assert int(data[-1]["branch_count"]) == 2
    finally:
        session.close()


@pytest.mark.parametrize("goal,status", [((3, 0, 0), "UNREACHABLE"), ((0, 0, 0), "DEGENERATE")])
def test_arm_inverse_goal_fields_drive_special_cases(goal, status):
    session = ActivitySession("5", "arm_inverse", goal=goal, exercise_model="reference")
    try:
        finish(session)
        assert session.outcome == status
    finally:
        session.close()


def test_arm_one_shared_physics_step_and_timeout():
    session = ActivitySession("5", "arm_control", duration=0.025,
                              dt=0.01, exercise_model="reference")
    try:
        with patch.object(session.world.physics, "step", wraps=session.world.physics.step) as physics_step:
            finish(session)
            assert physics_step.call_count == session.sim.steps == 3
        assert session.outcome == "TIMEOUT"
        assert tuple(session.laboratory.dq) == (0, 0)
    finally:
        session.close()


def test_incomplete_student_models_do_not_fall_back_to_reference(monkeypatch):
    def incomplete(*args, **kwargs):
        raise NotImplementedError("student implementation requested")

    monkeypatch.setattr(StudentPlanarArm, "forward", incomplete)
    arm = ActivitySession("5", "arm_forward", exercise_model="student")
    try:
        assert arm.state == "Ready"
        with pytest.raises(NotImplementedError, match="student implementation requested"):
            arm.step(single=True)
        assert arm.sim.time == 0
    finally:
        arm.close()

    monkeypatch.setattr(StudentOdometry, "update", incomplete)
    sensors = ActivitySession("4", "sensor_estimation_straight", exercise_model="student")
    try:
        sensors.start()
        with pytest.raises(NotImplementedError, match="student implementation requested"):
            for _ in range(30):
                sensors.step()
    finally:
        sensors.close()
