"""Control with simulator pose: one front ray first, then a lidar supervisor."""

import csv

import pytest

from robotics_sim import Pose
from robotics_sim.session import ActivitySession, TASKS
from robotics_sim.exercise_checks import ExerciseChecks
from robotics_sim.exercises.lesson03 import control
from robotics_sim.exercises.lesson04 import avoidance, rangefinder
from robotics_sim.exercises.lesson04.rangefinder import StudentRangeStop
from robotics_sim.sim.references.control import ReferencePoseController
from robotics_sim.sim.references.sensors import ReferenceRangeStop, ReferenceSectors


def unfinished(*args, **kwargs):
    raise NotImplementedError("The later exercise must not be called")


def finish(session):
    session.start()
    while session.state == "Running":
        session.step()


def test_main_course_sequence_is_rangefinder_lidar_and_avoidance():
    assert list(TASKS["4"]) == ["sensor_front_stop", "sensor_lidar", "sensor_avoidance"]


@pytest.mark.parametrize("mode", ["ideal", "noisy"])
def test_front_stop_uses_only_one_ray_and_known_pose(monkeypatch, tmp_path, mode):
    monkeypatch.setattr(StudentRangeStop, "command", ReferenceRangeStop.command)
    monkeypatch.setattr(control.StudentPoseController, "wheel_command", ReferencePoseController.wheel_command)
    monkeypatch.setattr(avoidance.StudentSectors, "summarize", unfinished)
    monkeypatch.setattr(avoidance.StudentObstacleSupervisor, "command", unfinished)
    session = ActivitySession("4", "sensor_front_stop", duration=8,
                              exercise_model="student", sensor_mode=mode)
    try:
        lab = session.laboratory
        assert not hasattr(lab, "odom") and not hasattr(lab, "imu")
        assert session.sensors.encoder is None and session.sensors.imu is None
        assert len(session.sensors.lidar.angles) == 1
        finish(session)
        assert session.outcome == "COMPLETED" and lab.controller.phase == "STOP"
        assert .2 < session.robot.pose().x < .8
        assert session.robot.wheel_velocities == (0, 0)
        assert all(ok for _, ok in ExerciseChecks(session).evaluate())
        output = tmp_path / "front.csv"
        session.export_csv(output)
        with output.open() as stream:
            rows = list(csv.DictReader(stream))
        assert all(row["left_m"] == row["right_m"] == "" for row in rows)
        assert float(rows[-1]["goal_error_m"]) > 1
        assert session.sensors.columns == ("time_s", "range_0_m", "true_range_0_m", "hit_0")
    finally:
        session.close()


def test_rangefinder_hysteresis_valid_miss_and_stale_data():
    controller = ReferenceRangeStop(stage="stop")
    pose, goal = Pose(), (3, 0)
    assert controller.command(pose, goal, .5, True, 4, 0) == (0, 0)
    assert controller.phase == "STOP"
    assert controller.command(pose, goal, .7, True, 4, 0) == (0, 0)
    assert controller.command(pose, goal, .9, True, 4, 0)[0] > 0
    assert controller.command(pose, goal, 4, False, 4, 0)[0] > 0
    assert controller.command(pose, goal, 4, False, 4, .3) == (0, 0)
    assert controller.phase == "STALE"
    assert controller.command(pose, goal, float("nan"), True, 4, 0) == (0, 0)


def test_rangefinder_student_todo_is_reported_on_first_step(monkeypatch):
    monkeypatch.setattr(StudentRangeStop, "command", unfinished)
    session = ActivitySession("4", "sensor_front_stop", exercise_model="student")
    try:
        assert session.state == "Ready"
        with pytest.raises(NotImplementedError):
            session.step(single=True)
        assert session.sim.time == 0
    finally:
        session.close()


def test_lidar_stop_reuses_the_completed_rangefinder_exercise(monkeypatch):
    monkeypatch.setattr(avoidance, "AVOIDANCE_STAGE", "stop")
    monkeypatch.setattr(StudentRangeStop, "command", ReferenceRangeStop.command)
    monkeypatch.setattr(control.StudentPoseController, "wheel_command", ReferencePoseController.wheel_command)
    monkeypatch.setattr(avoidance.StudentSectors, "summarize", ReferenceSectors.summarize)
    monkeypatch.setattr(avoidance.StudentObstacleSupervisor, "command", unfinished)
    session = ActivitySession("4", "sensor_avoidance", exercise_model="student", duration=8)
    try:
        assert type(session.laboratory.controller) is StudentRangeStop
        assert len(session.sensors.lidar.angles) == 37
        finish(session)
        assert session.laboratory.controller.phase == "STOP"
        assert all(ok for _, ok in ExerciseChecks(session).evaluate())
    finally:
        session.close()


def test_front_stop_reads_editable_thresholds(monkeypatch):
    monkeypatch.setattr(rangefinder, "ENTER_DISTANCE", .9)
    monkeypatch.setattr(rangefinder, "EXIT_DISTANCE", 1.0)
    session = ActivitySession("4", "sensor_front_stop", duration=6)
    try:
        finish(session)
        assert session.robot.pose().x < .5
        assert session.laboratory.controller.enter == .9
    finally:
        session.close()
