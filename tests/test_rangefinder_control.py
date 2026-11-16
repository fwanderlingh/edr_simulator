"""Control with simulator pose: one front ray first, then a lidar supervisor."""

import csv
import math

import pytest

from robotics_sim import Pose
from robotics_sim.session import ActivitySession, TASKS
from robotics_sim.exercise_checks import ExerciseChecks
from robotics_sim.experiment_data import capture_experiment
from robotics_sim.exercises.lesson03 import control
from robotics_sim.exercises.lesson04 import avoidance, rangefinder
from robotics_sim.exercises.lesson04.rangefinder import StudentRangeStop
from robotics_sim.sim.references.control import ReferencePoseController
from robotics_sim.sim.references.sensors import ReferenceRangeStop, ReferenceSectors
from robotics_sim.sim.references.avoidance import ReferenceObstacleSupervisor


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


def test_rangefinder_hysteresis_and_miss():
    controller = ReferenceRangeStop(stage="stop")
    pose, goal = Pose(), (3, 0, 0)
    assert controller.command(pose, goal, .5, True, 4) == (0, 0)
    assert controller.phase == "STOP"
    assert controller.command(pose, goal, .7, True, 4) == (0, 0)
    assert controller.command(pose, goal, .9, True, 4)[0] > 0
    assert controller.command(pose, goal, 4, False, 4)[0] > 0


@pytest.mark.parametrize("controller_class", [ReferenceRangeStop, ReferenceObstacleSupervisor])
def test_supervisor_requires_heading_and_reuses_final_pose_alignment(controller_class):
    controller = controller_class()
    goal = (0, 0, math.pi / 2)
    observations = (4, False, 4) if controller_class is ReferenceRangeStop else (
        {"front": 4, "left": 4, "right": 4},)
    v, omega = controller.command(Pose(), goal, *observations)
    assert controller.phase == "GO"  # Correct position alone does not complete the task.
    assert controller.nominal.controller.phase == "ALIGN"
    assert controller.nominal.controller.goal == goal
    assert v == 0 and omega > 0
    assert controller.command(Pose(yaw=math.pi / 2), goal, *observations) == (0, 0)
    assert controller.phase == "DONE"


@pytest.mark.parametrize("model", ["reference", "student"])
@pytest.mark.parametrize("task,stage", [("sensor_front_stop", None),
                                       ("sensor_avoidance", "stop"),
                                       ("sensor_avoidance", "avoid")])
def test_pose_goal_is_preserved_in_control_and_recording(monkeypatch, tmp_path, model, task, stage):
    monkeypatch.setattr(control.StudentPoseController, "wheel_command", ReferencePoseController.wheel_command)
    monkeypatch.setattr(StudentRangeStop, "command", ReferenceRangeStop.command)
    monkeypatch.setattr(avoidance.StudentSectors, "summarize", ReferenceSectors.summarize)
    monkeypatch.setattr(avoidance.StudentObstacleSupervisor, "command", ReferenceObstacleSupervisor.command)
    goal = (0, 0, math.pi / 2)
    session = ActivitySession("4", task, exercise_model=model, sensor_stage=stage,
                              goal=goal, duration=5, dt=.05)
    try:
        assert session.laboratory.target == goal
        session.step(single=True)
        assert session.outcome == "RUNNING"
        assert session.laboratory.pose_controller.phase == "ALIGN"
        assert "Final orientation alignment" in session.laboratory.readout
        finish(session)
        assert session.outcome == "DONE"
        assert abs(session.robot.pose().yaw - goal[2]) <= control.EPS_ANGLE
        assert session.robot.wheel_velocities == (0, 0)
        assert capture_experiment(session)["goals"] == (goal,)
        output = tmp_path / "pose.csv"
        session.export_csv(output)
        with output.open() as stream:
            final = list(csv.DictReader(stream))[-1]
        assert float(final["target_yaw_rad"]) == goal[2]
        assert abs(float(final["goal_heading_error_rad"])) <= control.EPS_ANGLE
    finally:
        session.close()


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


@pytest.mark.parametrize("model", ["student", "reference"])
@pytest.mark.parametrize("task,stage,expected_distance", [
    ("sensor_front_stop", "avoid", 1.7),
    ("sensor_avoidance", "sectors", 2.8),
    ("sensor_avoidance", "stop", 2.8),
    ("sensor_avoidance", "avoid", 2.8),
])
def test_scene_reads_the_active_exercise_position_and_size(monkeypatch, model, task, stage,
                                                          expected_distance):
    monkeypatch.setattr(rangefinder, "OBSTACLE_POSITION", (2.0, 0.0))
    monkeypatch.setattr(rangefinder, "OBSTACLE_SIZE", (0.6, 0.6, 1.0))
    monkeypatch.setattr(avoidance, "OBSTACLE_POSITION", (3.0, 0.0))
    monkeypatch.setattr(avoidance, "OBSTACLE_SIZE", (0.4, 3.0, 1.0))
    monkeypatch.setattr(avoidance, "AVOIDANCE_STAGE", stage)
    session = ActivitySession("4", task, exercise_model=model)
    try:
        lidar = session.sensors.lidar
        centre = len(lidar.angles) // 2
        assert lidar.hits[centre]
        assert lidar.scan()[centre] == pytest.approx(expected_distance)
    finally:
        session.close()


@pytest.mark.parametrize("task", ["sensor_front_stop", "sensor_avoidance"])
def test_moving_the_box_sideways_changes_the_actual_sensor_returns(monkeypatch, task):
    exercise = rangefinder if task == "sensor_front_stop" else avoidance
    monkeypatch.setattr(exercise, "OBSTACLE_POSITION", (1.5, 0.5))
    session = ActivitySession("4", task)
    try:
        lidar = session.sensors.lidar
        centre = len(lidar.angles) // 2
        assert not lidar.hits[centre]
        assert lidar.scan()[centre] == lidar.max_range
        if task == "sensor_avoidance":
            assert any(lidar.hits)  # Other rays observe the off-centre box.
    finally:
        session.close()
