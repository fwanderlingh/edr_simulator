"""Saved exercise edits are loaded in isolation from the developer's source files."""

from pathlib import Path
import shutil
import subprocess
import sys


def run_in_copy(tmp_path, program):
    package = Path(__file__).resolve().parents[1] / "robotics_sim"
    shutil.copytree(package, tmp_path / "robotics_sim", ignore=shutil.ignore_patterns("__pycache__"))
    result = subprocess.run([sys.executable, "-c", program], cwd=tmp_path,
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def test_reload_refreshes_importers_and_removes_deleted_globals_in_every_lesson(tmp_path):
    run_in_copy(tmp_path, r'''
from pathlib import Path
import importlib
from robotics_sim.exercise_reload import reload_lesson
from robotics_sim.session import ActivitySession
import robotics_sim.app as application

cases = (
    ("1", "lesson01.straight", "update", "straight"),
    ("2", "lesson02.planar", "solve_planar_task", "planar"),
    ("3", "lesson03.control", "StudentPoseController", "drive_goal"),
    ("4", "lesson04.estimation", "StudentOdometry", "sensor_estimation_straight"),
    ("5", "lesson05.kinematics", "StudentPlanarArm", "arm_forward"),
)
for lesson, name, definition, task in cases:
    module = importlib.import_module("robotics_sim.exercises." + name)
    previous = getattr(module, definition)
    module.DELETED_SETTING = True
    path = Path(module.__file__)
    path.write_text(path.read_text(encoding="utf-8") + "\nRELOAD_MARKER = True\n", encoding="utf-8")
    reload_lesson(lesson)
    assert module.RELOAD_MARKER and not hasattr(module, "DELETED_SETTING")
    assert getattr(module, definition) is not previous
    session = ActivitySession(lesson, task, kinematics="student", exercise_model="student")
    try:
        assert session.state == "Ready" and session.sim.time == 0
        if lesson == "1":
            session.step(single=True)
            assert session.robot.command.vx == .2
        elif lesson == "2":
            assert application.planar_exercise.solve_planar_task is module.solve_planar_task
        elif lesson == "3":
            from robotics_sim.exercises.lesson03 import activity, waypoint
            assert type(session.controller) is module.StudentPoseController
            assert activity.StudentPoseController is module.StudentPoseController
            assert waypoint.StudentWaypointController.pose_controller_class is module.StudentPoseController
            sequencer = waypoint.StudentWaypointController()
            assert type(sequencer.pose_controller) is module.StudentPoseController
            assert sequencer.pose_controller.goal == sequencer.goals[0]
        elif lesson == "4":
            from robotics_sim.exercises.lesson04 import gui, lidar
            from robotics_sim.exercises.lesson04.sensors import suite
            assert type(session.laboratory) is gui.SensorActivity
            assert type(session.laboratory.odom) is module.StudentOdometry
            assert suite.StudentLidar is lidar.StudentLidar
        else:
            from robotics_sim.exercises.lesson05 import gui
            assert type(session.laboratory) is gui.ArmActivity
            assert type(session.laboratory.model) is module.StudentPlanarArm
    finally:
        session.close()
''')


def test_reload_reads_fast_edits_and_rolls_back_syntax_and_import_failures(tmp_path):
    run_in_copy(tmp_path, r'''
from pathlib import Path
import os
import py_compile
import sys
from robotics_sim.exercise_reload import LESSON_MODULES, reload_lesson
from robotics_sim.session import ActivitySession
from robotics_sim.exercises.lesson03 import control

path = Path(control.__file__)
original = path.read_text(encoding="utf-8")
first = original + "\nRELOAD_MARKER = 'a'\n"
path.write_text(first, encoding="utf-8")
reload_lesson("3")
py_compile.compile(str(path), doraise=True)
stamp = path.stat()
updated = original + "\nRELOAD_MARKER = 'b'\n"
assert len(first) == len(updated)
path.write_text(updated, encoding="utf-8")
os.utime(path, ns=(stamp.st_atime_ns, stamp.st_mtime_ns))
reload_lesson("3")
assert control.RELOAD_MARKER == 'b'

for suffix, expected in (("\ndef broken(\n", SyntaxError),
                         ("\nimport missing_student_helper_for_reload_test\n", ModuleNotFoundError)):
    modules = [sys.modules["robotics_sim." + name] for name in LESSON_MODULES["3"]]
    previous = [dict(vars(module)) for module in modules]
    path.write_text(updated + suffix, encoding="utf-8")
    try:
        reload_lesson("3")
    except expected:
        pass
    else:
        raise AssertionError("The invalid source must be reported")
    for module, saved in zip(modules, previous):
        assert vars(module).keys() == saved.keys()
        assert all(vars(module)[key] is value for key, value in saved.items())
    path.write_text(updated, encoding="utf-8")
    reload_lesson("3")
    session = ActivitySession("3", "drive_waypoints")
    try:
        session.step(single=True)
        assert session.sim.time > 0
    finally:
        session.close()
''')


def test_lesson4_reload_reads_the_saved_lesson3_controller_and_parameters(tmp_path):
    run_in_copy(tmp_path, r'''
from pathlib import Path
from robotics_sim.exercise_reload import reload_lesson
from robotics_sim.exercises.lesson03 import control
from robotics_sim.session import ActivitySession

previous = control.StudentPoseController
path = Path(control.__file__)
path.write_text(path.read_text(encoding="utf-8") + "\nGAIN_POS = .123\nWHEEL_MAX = .75\n", encoding="utf-8")
reload_lesson("4")
assert control.StudentPoseController is not previous
session = ActivitySession("4", "sensor_avoidance", exercise_model="student")
try:
    assert type(session.laboratory.pose_controller) is control.StudentPoseController
    assert session.laboratory.pose_controller.gain_pos == .123
    assert session.laboratory.wheel_max == .75
finally:
    session.close()
''')
