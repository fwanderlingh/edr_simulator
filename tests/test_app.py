"""Opt-in real desktop tests; CI uses Xvfb on Ubuntu."""

import os
import functools
from pathlib import Path
import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("ROBOTICS_SIM_GUI_TEST") != "1",
                                reason="Set ROBOTICS_SIM_GUI_TEST=1 with a desktop or Xvfb.")


def test_lesson4_stages_use_the_same_ui_with_explicit_model_labels(tmp_path, monkeypatch):
    import tkinter as tk
    from robotics_sim.app import SimulatorApp
    from robotics_sim.session import TASKS
    from robotics_sim.exercises.lesson03 import control
    from robotics_sim.exercises.lesson04 import avoidance, rangefinder
    from robotics_sim.sim.references.control import ReferencePoseController
    from robotics_sim.sim.references.sensors import ReferenceRangeStop, ReferenceSectors, ReferenceObstacleSupervisor

    monkeypatch.setattr(avoidance, "AVOIDANCE_STAGE", "sectors")
    monkeypatch.setattr(avoidance.StudentSectors, "summarize", ReferenceSectors.summarize)
    root = tk.Tk()
    root.withdraw()  # This checks actual Tk state without opening an interactive window.
    app = SimulatorApp(root, lesson="4", duration=.2, dt=.01,
                       settings_path=tmp_path / "stages.json")
    try:
        app.task.set(TASKS["4"]["sensor_avoidance"])
        app.exercise_model.set("student")
        app.load_task()
        app.single_step()
        assert app.model_heading.cget("text") == "SECTOR MODEL"
        assert "Stage: sectors" in app.values.get()
        assert app.session.sim.steps == 1

        monkeypatch.setattr(avoidance, "AVOIDANCE_STAGE", "stop")
        monkeypatch.setattr(rangefinder.StudentRangeStop, "command", ReferenceRangeStop.command)
        monkeypatch.setattr(avoidance.StudentObstacleSupervisor, "command", ReferenceObstacleSupervisor.command)
        monkeypatch.setattr(control.StudentPoseController, "wheel_command", ReferencePoseController.wheel_command)
        app.load_task()
        app.single_step()
        assert app.model_heading.cget("text") == "SUPERVISOR MODEL"
        assert "Stage: stop" in app.values.get()
        assert app.session.laboratory.pose_controller.__class__ is control.StudentPoseController
        app.task.set(TASKS["4"]["sensor_front_stop"])
        app.load_task()
        app.single_step()
        assert app.model_heading.cget("text") == "STOP CONTROLLER MODEL"
        assert "front rangefinder" in app.values.get()
        assert "Encoder" not in app.values.get() and "Gyro" not in app.values.get()
    finally:
        app.close()


def test_escape_room_preview_has_scene_labels_and_its_own_camera(tmp_path):
    import tkinter as tk
    from robotics_sim.app import SimulatorApp
    from robotics_sim.session import LESSONS
    from robotics_sim.exercises.lesson08.activity import CAMERA

    root = tk.Tk()
    root.withdraw()
    app = SimulatorApp(root, lesson="8", settings_path=tmp_path / "preview.json")
    try:
        assert app.session.task == "escape_room_preview" and app.session.sim.time == 0
        assert app.camera.distance == CAMERA["distance"]
        assert not app.model_controls.grid_info()
        assert not app.sensor_controls.grid_info() and not app.goal_controls.grid_info()
        assert not app.checks_card.grid_info()
        assert app.implementation_heading.cget("text") == "SCENE SETUP"
        assert "stationary" in app.values.get()
        app._render()
        labels = [app.canvas.itemcget(i, "text") for i in app.canvas.find_all()
                  if app.canvas.type(i) == "text"]
        assert "EXIT" in labels and "Switch" in labels and "Medical kit" in labels
        app.camera.distance = 3
        app.reset_camera()
        assert app.camera.distance == CAMERA["distance"]
        initial = app.session.robot.pose()
        app.single_step()
        assert app.session.robot.pose() == initial
        app.lesson.set(LESSONS["1"])
        app.select_lesson()
        assert app.camera.distance == 5.5
        assert app.implementation_heading.cget("text") == "YOUR IMPLEMENTATION"
    finally:
        app.close()


def desktop_process(test):
    """Run each desktop scenario in its own process, like a real app launch."""
    @functools.wraps(test)
    def isolated(tmp_path):
        if os.environ.get("ROBOTICS_SIM_GUI_WORKER") == "1":
            return test(tmp_path)
        environment = dict(os.environ, ROBOTICS_SIM_GUI_WORKER="1", PYTEST_ADDOPTS="")
        result = subprocess.run(
            [sys.executable, "-m", "pytest", f"{Path(__file__).resolve()}::{test.__name__}",
             "-q", "-p", "no:cacheprovider", "--basetemp", str(tmp_path / "desktop")],
            env=environment, capture_output=True, text=True, timeout=45)
        assert result.returncode == 0, result.stdout + result.stderr
    return isolated


def wait_for_ui(root, condition, *, timeout=5, description="UI condition"):
    """Pump Tk until an observable result exists, not just currently due events."""
    deadline = time.monotonic() + timeout
    while True:
        root.update()
        if condition():
            return
        if time.monotonic() >= deadline:
            pytest.fail(f"Timed out waiting for {description}")
        time.sleep(0.005)


@desktop_process
@pytest.mark.solution
def test_exercise_checks_card_pass_failure_todo_switch_and_reset(tmp_path):
    import tkinter as tk
    from unittest.mock import patch
    import numpy as np
    from robotics_sim.app import SimulatorApp
    from robotics_sim.exercises.lesson02 import rpy
    from robotics_sim.session import LESSONS, TASKS

    root = tk.Tk()
    app = SimulatorApp(root, lesson="2", duration=.02, dt=.01, settings_path=tmp_path / "checks.json")
    try:
        app.task.set(TASKS["2"]["rpy"])
        app.load_task()
        root.update()
        assert app.checks_card.winfo_ismapped()
        assert len(app.check_rows) == 4
        assert [marker.cget("text") for marker, _ in app.check_rows] == ["✓"] * 4
        assert app.checks_badge.winfo_ismapped()
        assert app.checks_badge.cget("text") == "Passed"
        assert "Checks:" not in app.values.get()
        # The card wraps inside the narrow sidebar and stays above measurements.
        assert app.checks_card.winfo_y() < app.values_label.winfo_y()
        assert all(label.winfo_x() + label.winfo_width() <= app.checks_list.winfo_width()
                   for _, label in app.check_rows)
        with patch.object(rpy, "solve_rpy_task", return_value=(np.eye(3), np.eye(3))):
            app._update_status()
            root.update()
            assert [marker.cget("text") for marker, _ in app.check_rows] == ["→", "✓", "✓", "✓"]
            assert app.check_rows[0][0].cget("style") == "CheckFailed.TLabel"
            assert app.check_rows[1][0].cget("style") == "CheckPassed.TLabel"
            assert not app.checks_badge.winfo_ismapped()
        with patch.object(rpy, "solve_rpy_task", side_effect=NotImplementedError):
            app._update_status()
            root.update()
            assert [marker.cget("text") for marker, _ in app.check_rows] == ["→"] * 4
            assert not app.checks_badge.winfo_ismapped()
        app.lesson.set(LESSONS["3"])
        app.select_lesson()
        app.single_step()
        app.single_step()
        root.update()
        assert app.checks_badge.winfo_ismapped()
        app.load_task()
        root.update()
        assert not app.checks_badge.winfo_ismapped()
        assert all(marker.cget("text") == "→" for marker, _ in app.check_rows)
        app.lesson.set(LESSONS["1"])
        app.select_lesson()
        root.update()
        assert not app.checks_card.winfo_ismapped()
    finally:
        app.close()


@desktop_process
@pytest.mark.solution
def test_module2_menu_readouts_render_and_todo(tmp_path):
    import tkinter as tk
    from unittest.mock import patch
    from robotics_sim.app import SimulatorApp
    from robotics_sim.exercises.lesson02 import module2
    from robotics_sim.session import TASKS

    root = tk.Tk()
    app = SimulatorApp(root, lesson="2", settings_path=tmp_path / "settings.json")
    try:
        for task, example in (("spatial", "Sensor origin"), ("rpy", "RPY (rad)"), ("quaternions", "Quaternions (wxyz)")):
            app.task.set(TASKS["2"][task])
            app.load_task()
            root.update()
            app._render()
            assert app.session.task == task
            assert example in app.values.get()
            assert app.canvas.find_withtag("scene")
            with patch.object(module2, "evaluate", side_effect=NotImplementedError):
                app._update_status()
                app._render()
                assert "not implemented" in app.values.get()
                assert app.values_label.cget("background") == "#fff2cd"
    finally:
        app.close()


@desktop_process
@pytest.mark.solution
def test_desktop_completion_pause_switch_reset_and_labels(tmp_path):
    import tkinter as tk
    from robotics_sim.app import SimulatorApp, enable_high_dpi
    from robotics_sim.session import LESSONS, TASKS

    enable_high_dpi()
    root = tk.Tk()
    app = SimulatorApp(root, duration=0.04, dt=0.01, csv_path=tmp_path / "automatic.csv", settings_path=tmp_path / "settings.json")
    try:
        # Deliberately delay the first frame to exercise the startup race seen
        # on fast Ubuntu/Xvfb runners. The normal render loop must draw it.
        root.after_cancel(app._timer)
        app._timer = root.after(150, app._tick)
        wait_for_ui(root, lambda: app.canvas.find_withtag("scene"), description="the first rendered scene")
        assert app.session.state == "Ready"
        assert app.lesson.get() == "Lesson 1 - System overview"
        scene_items = app.canvas.find_withtag("scene")
        assert scene_items
        assert not any(app.canvas.type(item) == "image" for item in app.canvas.find_all())
        app._render()
        assert app.canvas.find_withtag("scene") == scene_items
        app.start_button.invoke()
        wait_for_ui(root, lambda: app.session.state == "Completed", timeout=10,
                    description="activity completion")
        assert app.session.state == "Completed"
        assert root.winfo_exists()
        assert app.session.world.physics.connected
        assert (tmp_path / "automatic.csv").exists()
        assert "Completed" in app.status.get()
        assert str(app.start_button["state"]) == "disabled"
        app.load_task()
        assert app.session.sim.time == 0
        assert app.session.samples == []
        app.start_button.invoke()
        app.start_button.invoke()
        assert app.session.state == "Paused"
        root.update()
        assert app.session.sim.time == 0
        app.step_button.invoke()
        assert app.session.sim.time == 0.01
        previous = app.session
        app.lesson.set(LESSONS["2"])
        app.select_lesson()
        root.update()
        assert previous.state == "Closed"
        assert app.session.lesson == "2"
        assert app.session.sim.time == 0
        app.set_label_style(size=16, color="#803399", background=False, background_color="#ffffcc")
        app._render()
        text_items = [item for item in app.canvas.find_all() if app.canvas.type(item) == "text"]
        labels = {app.canvas.itemcget(item, "text") for item in text_items}
        assert {"World", "Robot", "Sensor", "Sensor point",
                "Direction (no translation)"} <= labels
        assert "Module 1 output: point in world (2.00, 2.50) m" in app.values.get()
        assert "direction in world (0.00, 1.00)" in app.values.get()
        assert "World goal" not in labels
        assert "Goal in robot" not in app.values.get()
        assert all("16" in app.canvas.itemcget(item, "font") for item in text_items)
        assert all(app.canvas.itemcget(item, "fill") == "#803399" for item in text_items)
        assert not app.canvas.find_withtag("label-background")
        app.set_label_style(size=14, color="#123456", background=True, background_color="#ffffcc")
        app._render()
        backgrounds = app.canvas.find_withtag("label-background")
        assert len(backgrounds) == len(labels)
        assert all(app.canvas.itemcget(item, "fill") == "#ffffcc" for item in backgrounds)
        app.set_label_style(size=14, color="#123456", background=True,
                            background_color="#ffffcc", transparency=50)
        app._render()
        backgrounds = app.canvas.find_withtag("label-background")
        assert len(backgrounds) == len(labels)
        assert all(app.canvas.type(item) == "image" for item in backgrounds)
        cached_images = dict(app._label_images)
        app._render()
        assert dict(app._label_images) == cached_images
        assert len(app.canvas.find_withtag("label-text")) == len(labels)
        # Check Tk's actual alpha compositing, not just PNG metadata.
        sample = next(iter(app._label_images.values()))
        import tkinter as tk
        target = tk.PhotoImage(width=1, height=1, master=root)
        target.put("#000000", to=(0, 0, 1, 1))
        root.tk.call(str(target), "copy", str(sample), "-from", 0, 0, 1, 1, "-compositingrule", "overlay")
        assert target.get(0, 0) == (128, 128, 102)
        app.set_label_style(size=14, color="#123456", background=True,
                            background_color="#ffffcc", transparency=100)
        app._render()
        assert not app.canvas.find_withtag("label-background")
        assert len(app.canvas.find_withtag("label-text")) == len(labels)
        app.label_settings()
        assert app._label_dialog.winfo_exists()
        app._label_dialog.destroy()
        for title in TASKS["2"].values():
            app.task.set(title)
            app.load_task()
            root.update()
            assert app.session.state == "Ready"
            assert app.label_size == 14
            assert app.label_color == "#123456"
        app.lesson.set(LESSONS["3"])
        app.select_lesson()
        app.kinematics.set("student")
        for key, title in TASKS["3"].items():
            app.kinematics.set("reference" if key in ("drive_proportional", "drive_goal", "drive_waypoints", "drive_goal_behind") else "student")
            app.task.set(title)
            app.load_task()
            app.single_step()
            app._render()
            assert len(app.renderer.scene.shapes) == 4
            assert "Wheels L/R" in app.values.get()
            if app.session.controller is not None:
                assert app.session.controller.__class__.__name__ == (
                    "ReferenceProportionalController" if key == "drive_proportional" else
                    "ReferenceWaypointController" if key == "drive_waypoints" else "ReferencePoseController")
            else:
                assert app.session.robot.motor_model.__class__.__name__ == "StudentDifferentialDrive"
        app.lesson.set(LESSONS["4"])
        app.select_lesson()
        for mode, task in (("ideal", "sensor_front_stop"), ("noisy", "sensor_avoidance"),
                           ("ideal", "sensor_lidar")):
            app.sensor_mode.set(mode)
            app.task.set(TASKS["4"][task])
            app.load_task()
            app.single_step()
            app._render()
            assert app.session.sensors.time == 0  # Cached until next sensor sample.
            while app.session.state != "Completed":
                app.single_step()
            assert app.session.sensors.time == pytest.approx(0.04)
            assert "front" in app.values.get().lower()
            assert "Encoder L/R" not in app.values.get()
            assert "Gyro z" not in app.values.get()
            assert app.canvas.find_withtag("scene")
    finally:
        app.close()
    assert app.session.state == "Closed"


@desktop_process
def test_proportional_exercise_selects_its_student_file_and_reference(tmp_path):
    import tkinter as tk
    from unittest.mock import patch
    from robotics_sim.app import SimulatorApp, enable_high_dpi
    from robotics_sim.exercises.lesson03.proportional import StudentProportionalController
    from robotics_sim.session import TASKS

    enable_high_dpi()
    root = tk.Tk()
    app = SimulatorApp(root, lesson="3", settings_path=tmp_path / "proportional.json")
    try:
        app.kinematics.set("reference")
        app.task.set(TASKS["3"]["drive_proportional"])
        app.load_task()
        root.update()
        app._render()
        assert str(app.model_heading.cget("text")) == "CONTROLLER MODEL"
        assert "lesson03/proportional.py" in str(app.edit_file.cget("text"))
        assert "activity.py" not in str(app.edit_file.cget("text"))
        assert not app.goal_controls.winfo_ismapped()
        assert any(app.canvas.itemcget(i, "text") == "Goal 1"
                   for i in app.canvas.find_withtag("label-text"))
        app.single_step()
        assert app.session.robot.wheel_velocities == pytest.approx((.4 / .12, .4 / .12))
        app.kinematics.set("student")
        app.load_task()
        with patch.object(StudentProportionalController, "wheel_command",
                          side_effect=NotImplementedError("Complete wheel_command")):
            app.single_step()
        assert "wheel_command" in app.status.get()
        assert "not implemented" in app.status.get().lower()
        assert app.session.sim.steps == 0
    finally:
        app.close()


@desktop_process
def test_settings_restore_after_closing_and_reopening(tmp_path):
    import json
    import tkinter as tk
    from robotics_sim.app import SimulatorApp, enable_high_dpi
    from robotics_sim.session import LESSONS, TASKS

    enable_high_dpi()
    path = tmp_path / "settings.json"
    root = tk.Tk()
    app = SimulatorApp(root, settings_path=path)
    try:
        # Saving must capture the live geometry even if Configure notifications
        # have not updated the cached bounds (as observed under Ubuntu/Xvfb).
        root.unbind("<Configure>")
        root.geometry("1240x820+50+60")
        root.update()
        # Exercise the native divider's mouse bindings, including live reflow.
        sash_x, sash_y = app.workspace.sash_coord(0)
        app.workspace.event_generate("<ButtonPress-1>", x=sash_x + 3, y=sash_y + 50)
        root.update()
        app.workspace.event_generate("<B1-Motion>", x=sash_x - 147, y=sash_y + 50)
        root.update()
        app.workspace.event_generate("<ButtonRelease-1>", x=sash_x - 147, y=sash_y + 50)
        root.update()
        expected_sidebar = app.inspector_canvas.master.winfo_width()
        assert expected_sidebar == pytest.approx(444, abs=2)
        assert int(app.edit_hint.cget("wraplength")) > 350
        app.lesson.set(LESSONS["2"])
        app.select_lesson()
        app.task.set(TASKS["2"]["static"])
        app.duration.set("25")
        app.sensor_mode.set("noisy")
        app.load_task()
        app.show_frames.set(False)
        app.set_label_style(size=18, color="#123456", background=False,
                            background_color="#abcdef", transparency=37.5)
        app.camera.yaw = 70
        app.camera.pitch = -40
        app.camera.distance = 4
        root.update()
        expected_size = root.winfo_width(), root.winfo_height()
    finally:
        app.close()

    saved = json.loads(path.read_text())
    assert (saved["width"], saved["height"]) == expected_size
    assert saved["task"] == "static"
    assert saved["sidebar_width"] == expected_sidebar

    root = tk.Tk()
    app = SimulatorApp(root, settings_path=path)
    try:
        root.update()
        assert (root.winfo_width(), root.winfo_height()) == expected_size
        assert app.inspector_canvas.master.winfo_width() == expected_sidebar
        assert app.session.lesson == "2"
        assert app.session.task == "static"
        assert app.session.duration == 25
        assert app.sensor_mode.get() == "noisy"
        assert app.session.state == "Ready"
        assert app.label_size == 18
        assert app.label_color == "#123456"
        assert app.label_background is False
        assert app.label_background_color == "#abcdef"
        assert app.label_background_transparency == 37.5
        assert app.show_frames.get() is False
        assert (app.camera.yaw, app.camera.pitch, app.camera.distance) == (70, -40, 4)
    finally:
        app.close()

    # Explicit launch arguments override remembered selections on both platforms.
    root = tk.Tk()
    app = SimulatorApp(root, lesson="1", duration=3, settings_path=path)
    try:
        assert app.session.lesson == "1"
        assert app.session.task == "straight"
        assert app.session.duration == 3
    finally:
        app.close()


@pytest.mark.skipif(os.name != "nt", reason="Requires a window manager supporting Windows zoomed state.")
@desktop_process
def test_maximized_state_preserves_normal_window_dimensions(tmp_path):
    import json
    import tkinter as tk
    from robotics_sim.app import SimulatorApp, enable_high_dpi

    enable_high_dpi()
    path = tmp_path / "maximized.json"
    root = tk.Tk()
    app = SimulatorApp(root, settings_path=path)
    try:
        root.geometry("1200x800+30+30")
        root.update()
        normal_size = app._normal_window["width"], app._normal_window["height"]
        root.state("zoomed")
        root.update()
    finally:
        app.close()
    saved = json.loads(path.read_text())
    assert saved["maximized"] is True
    assert (saved["width"], saved["height"]) == normal_size
    root = tk.Tk()
    app = SimulatorApp(root, settings_path=path)
    try:
        root.update()
        assert root.state() == "zoomed"
        assert (app._normal_window["width"], app._normal_window["height"]) == normal_size
    finally:
        app.close()

@desktop_process
def test_unimplemented_exercises_show_a_hint_instead_of_crashing(tmp_path):
    import tkinter as tk
    import robotics_sim.app as application
    from robotics_sim.session import LESSONS

    def todo(*args, **kwargs):
        raise NotImplementedError("complete the exercise")

    application.planar_exercise.solve_planar_task = todo
    application.frame_exercise.point_in_world = todo
    application.enable_high_dpi()
    root = tk.Tk()
    app = application.SimulatorApp(root, duration=0.04, dt=0.01, settings_path=tmp_path / "settings.json")
    try:
        app.lesson.set(LESSONS["2"])
        app.select_lesson()
        app._render()
        assert "Module 1 not implemented yet" in app.values.get()
        app.session.step = todo
        app.step_button.invoke()
        assert "Exercise not implemented yet: complete the exercise" in app.status.get()
        assert app.session.state == "Ready"
    finally:
        app.close()


@desktop_process
def test_student_runtime_errors_stop_motion_and_leave_the_ui_usable(tmp_path):
    import tkinter as tk
    from unittest.mock import patch
    from robotics_sim.app import SimulatorApp
    from robotics_sim.session import LESSONS, TASKS
    from robotics_sim.exercises.lesson03.control import StudentPoseController
    from robotics_sim.exercises.lesson03.proportional import StudentProportionalController
    from robotics_sim.exercises.lesson04.rangefinder import StudentRangeStop
    from robotics_sim.exercises.lesson04.avoidance import StudentSectors
    from robotics_sim.exercises.lesson05.kinematics import StudentPlanarArm

    root = tk.Tk()
    callback_errors = []
    root.report_callback_exception = lambda *error: callback_errors.append(error)
    app = SimulatorApp(root, lesson="1", dt=.01, settings_path=tmp_path / "runtime-errors.json")
    cases = (
        ("3", "drive_proportional", StudentProportionalController, "wheel_command", NameError),
        ("3", "drive_goal", StudentPoseController, "wheel_command", RuntimeError),
        ("4", "sensor_front_stop", StudentRangeStop, "command", IndexError),
        ("4", "sensor_avoidance", StudentSectors, "summarize", KeyError),
        ("5", "arm_forward", StudentPlanarArm, "forward", AttributeError),
    )
    try:
        app.kinematics.set("student")
        app.exercise_model.set("student")
        for lesson, task, controller, method, error_type in cases:
            with patch.object(controller, method, side_effect=error_type("student mistake")):
                app.lesson.set(LESSONS[lesson])
                app.select_lesson()
                app.task.set(TASKS[lesson][task])
                app.load_task()
                app.toggle_running()
                wait_for_ui(root, lambda: "student mistake" in app.status.get(),
                            description=f"handled {error_type.__name__} in {task}")
                assert app.session.state == "Paused"
                assert error_type.__name__ in app.status.get()
                assert "Check your implementation:" in app.status.get()
                assert app.session.robot.command.vx == app.session.robot.command.omega == 0
                if lesson == "5":
                    assert tuple(app.session.laboratory.dq) == (0, 0)
                else:
                    assert app.session.robot.wheel_velocities == (0, 0)
                assert root.tk.call("after", "info", app._timer)
                assert not callback_errors
        app.lesson.set(LESSONS["1"])
        app.select_lesson()
        app.single_step()
        assert app.session.sim.steps == 1
        assert not app._notice and not callback_errors
    finally:
        app.close()


@desktop_process
def test_lesson2_exceptions_are_handled_during_startup_and_rendering(tmp_path):
    import tkinter as tk
    from unittest.mock import patch
    import robotics_sim.app as application
    from robotics_sim.session import TASKS

    root = tk.Tk()
    callback_errors = []
    root.report_callback_exception = lambda *error: callback_errors.append(error)
    with patch.object(application.planar_exercise, "solve_planar_task", side_effect=NameError("bad planar formula")):
        app = application.SimulatorApp(root, lesson="2", settings_path=tmp_path / "static-errors.json")
        try:
            assert "NameError: bad planar formula" in app.status.get()
            assert "not implemented" not in app.values.get().lower()
            app._render()
            root.update()
            assert not callback_errors
            with patch.object(application.module2.spatial, "solve_spatial_task",
                              side_effect=IndexError("bad transform index")):
                app.task.set(TASKS["2"]["spatial"])
                app.load_task()
                app._render()
                root.update()
                assert "IndexError: bad transform index" in app.status.get()
            with patch.object(application.frame_exercise, "point_in_world", side_effect=AttributeError("bad frame")):
                for task in ("frames", "static"):
                    app.task.set(TASKS["2"][task])
                    app.load_task()
                    app._render()
                    root.update()
                    assert "AttributeError: bad frame" in app.status.get()
            assert not callback_errors
            assert app.session.world.physics.connected
        finally:
            app.close()


@desktop_process
def test_student_error_while_loading_keeps_startup_and_previous_activity_usable(tmp_path):
    import json
    import tkinter as tk
    from unittest.mock import patch
    from robotics_sim.app import SimulatorApp
    from robotics_sim.exercises.lesson04 import lidar
    from robotics_sim.session import LESSONS, TASKS

    settings = tmp_path / "loading-error.json"
    settings.write_text(json.dumps({"lesson": "4", "task": "sensor_lidar", "sensor_mode": "student"}))
    saved = tmp_path / "lidar.py"
    root = tk.Tk()
    with patch.object(lidar.StudentLidar, "directions", side_effect=RuntimeError("bad scan directions")), \
         patch.object(lidar, "__file__", str(saved)), \
         patch("robotics_sim.app.messagebox.showerror") as showerror:
        app = SimulatorApp(root, settings_path=settings)
        try:
            assert app.session.lesson == "1" and app.session.task == "straight"
            assert "RuntimeError: bad scan directions" in app.status.get()
            assert showerror.call_count == 1
            previous = app.session
            app.lesson.set(LESSONS["4"])
            app.select_lesson()
            assert app.session is previous
            assert app.lesson_id == "1" and app.task.get() == TASKS["1"]["straight"]
            assert showerror.call_count == 2
            assert previous.world.physics.connected
            app._render()
            root.update()
            # Simulate saving a repair without depending on completed exercises:
            # the public student clone has an unfinished lidar implementation.
            saved.write_text("from .sensors.lidar import LidarSensor\n"
                             "class StudentLidar(LidarSensor):\n    pass\n", encoding="utf-8")
            # Reload also retries the activity whose student sensor failed to load.
            app.reload_button.invoke()
            assert app.session.lesson == "4" and app.session.task == "sensor_encoders"
            assert app.session is not previous and not app._notice
            assert showerror.call_count == 2
        finally:
            app.close()


@desktop_process
def test_reload_button_loads_saved_fixes_and_recovers_from_invalid_source(tmp_path):
    import math
    import tkinter as tk
    from textwrap import dedent
    from unittest.mock import patch
    from robotics_sim.app import SimulatorApp
    from robotics_sim.exercises.lesson03 import control, waypoint
    from robotics_sim.session import LESSONS, TASKS

    def source(command):
        return dedent('''\
            from ._control_runtime import PoseController
            GOAL = (1, .5, 0)
            GAIN_POS = .8
            GAIN_BEARING = 2.0
            GAIN_HEADING = 1.5
            EPS_POS = .03
            EPS_ANGLE = .03
            ANGLE_IN = .08
            ANGLE_OUT = .6
            WHEEL_MAX = 5.0
            class StudentPoseController(PoseController):
                def wheel_command(self, pose, r, L):
                    ''') + '        ' + command + '\n'

    saved = tmp_path / "control.py"
    saved.write_text(source('raise NameError("fix this exercise")'), encoding="utf-8")
    root = tk.Tk()
    callback_errors = []
    root.report_callback_exception = lambda *error: callback_errors.append(error)
    app = SimulatorApp(root, lesson="1", dt=.01, settings_path=tmp_path / "reload.json")
    with patch.object(control, "__file__", str(saved)):
        try:
            root.geometry("900x620+20+20")
            app.lesson.set(LESSONS["3"])
            app.select_lesson()
            app.kinematics.set("student")
            app.task.set(TASKS["3"]["drive_goal"])
            app.load_task()
            app.goal_x.set("2")
            app.goal_y.set(".25")
            app.goal_yaw.set("60")
            app.duration.set("5")
            app.load_task()
            app.reload_button.invoke()
            app.single_step()
            assert "NameError: fix this exercise" in app.status.get()

            previous = app.session
            saved.write_text(source("return 2.0, 2.0"), encoding="utf-8")
            app.reload_button.invoke()
            assert app.session is not previous and not previous.world.physics.connected
            assert root.winfo_exists() and app.session.state == "Ready"
            assert app.session.sim.time == 0 and app.session.samples == []
            assert app.session.duration == 5 and app.kinematics.get() == "student"
            assert app.session.controller.goal == (2, .25, math.radians(60))
            assert waypoint.StudentWaypointController.pose_controller_class is control.StudentPoseController
            app.single_step()
            assert app.session.robot.wheel_velocities == (2, 2)
            assert not app._notice

            previous = app.session
            saved.write_text("def invalid(\n", encoding="utf-8")
            app.reload_button.invoke()
            assert app.session is previous and app.session.state == "Paused"
            assert app.session.robot.wheel_velocities == (0, 0)
            assert "SyntaxError" in app.status.get()
            assert str(app.start_button.cget("state")) == "disabled"
            assert str(app.step_button.cget("state")) == "disabled"
            assert str(app.reload_button.cget("state")) == "normal"
            app._render()
            root.update()
            assert not callback_errors

            saved.write_text(source("return 3.0, 3.0"), encoding="utf-8")
            app.reload_button.invoke()
            app.single_step()
            assert app.session.robot.wheel_velocities == (3, 3)
            assert not app._notice and not app._reload_failed
            root.update()
            assert app.reload_button.winfo_viewable()
            assert not callback_errors
            app.toggle_running()
            assert app.session.state == "Running"
            app.reload_button.invoke()
            assert app.session.state == "Ready" and app.session.sim.time == 0
            assert app.session.robot.wheel_velocities == (0, 0)
        finally:
            app.close()


@desktop_process
def test_saved_student_kinematics_opens_and_uses_the_todo_warning(tmp_path):
    import json
    import tkinter as tk
    from unittest.mock import patch
    from robotics_sim.app import SimulatorApp, enable_high_dpi
    from robotics_sim.exercises.lesson03.kinematics import StudentDifferentialDrive
    from robotics_sim.session import TASKS

    enable_high_dpi()
    path = tmp_path / "student.json"
    path.write_text(json.dumps({"lesson": "3", "task": "drive_inverse",
                               "kinematics": "student"}))
    root = tk.Tk()
    with patch.object(StudentDifferentialDrive, "forward",
                      side_effect=NotImplementedError("complete forward()")), \
         patch.object(StudentDifferentialDrive, "inverse",
                      side_effect=NotImplementedError("complete inverse()")):
        app = SimulatorApp(root, settings_path=path)
        try:
            root.update()
            assert app.session.task == "drive_inverse" and app.session.state == "Ready"
            for task in ("drive_line", "drive_circle", "drive_spin", "drive_inverse"):
                app.task.set(TASKS["3"][task])
                app.load_task()
                root.update()
                assert app.canvas.winfo_viewable()
                assert app.session.state == "Ready" and app.session.sim.steps == 0
                app.step_button.invoke()
                assert "Exercise not implemented yet:" in app.status.get()
                assert app.session.sim.steps == 0 and app.session.samples == []
                assert app.session.robot.wheel_velocities == (0, 0)
            app.load_task()
            app.start_button.invoke()
            wait_for_ui(root, lambda: app.session.state == "Paused"
                        and "Exercise not implemented yet:" in app.status.get(),
                        description="standard TODO warning after Start")
            assert app.session.sim.steps == 0
            app.kinematics.set("reference")
            app.load_task()
            app.single_step()
            assert app.session.sim.steps == 1
            assert "not implemented" not in app.status.get()
        finally:
            app.close()


@desktop_process
def test_compact_layout_keeps_viewport_and_activity_controls_visible(tmp_path):
    """Long instructions and sensor readouts must not squeeze the scene away."""
    import tkinter as tk
    from tkinter import ttk

    from robotics_sim.app import SimulatorApp, enable_high_dpi
    from robotics_sim.session import LESSONS

    enable_high_dpi()
    root = tk.Tk()
    app = SimulatorApp(root, duration=0.2, dt=0.01,
                       settings_path=tmp_path / "compact.json")

    def descendants(widget):
        for child in widget.winfo_children():
            yield child
            yield from descendants(child)

    def assert_visible(widget):
        assert widget.winfo_viewable(), str(widget)
        x = widget.winfo_rootx() - root.winfo_rootx()
        y = widget.winfo_rooty() - root.winfo_rooty()
        assert 0 <= x < root.winfo_width(), (str(widget), x)
        assert 0 <= y < root.winfo_height(), (str(widget), y)
        assert x + widget.winfo_width() <= root.winfo_width(), str(widget)
        assert y + widget.winfo_height() <= root.winfo_height(), str(widget)

    try:
        root.geometry("900x620+20+20")
        root.update()
        for lesson in LESSONS:
            app.lesson.set(LESSONS[lesson])
            app.select_lesson()
            root.update()
            app._render()
            assert app.canvas.winfo_width() >= 450
            assert app.canvas.winfo_height() >= 260
            assert_visible(app.canvas)
            assert_visible(app.start_button)
            assert_visible(app.step_button)
            assert_visible(app.tasks)
            lesson_selector = next(
                widget for widget in descendants(root)
                if isinstance(widget, ttk.Combobox)
                and str(widget.cget("textvariable")) == str(app.lesson)
            )
            assert_visible(lesson_selector)
            assert "ready" in str(app.state_badge.cget("text")).lower()
            assert float(app.progress.cget("maximum")) == 100
            assert float(app.progress.cget("value")) == pytest.approx(0)
            assert app.time_readout.get()
            app.step_button.invoke()
            assert 0 < float(app.progress.cget("value")) <= 100
            assert app.session.state.lower() in str(app.state_badge.cget("text")).lower()

        for _ in range(20):
            if app.session.state == "Completed":
                break
            app.single_step()
        assert app.session.state == "Completed"
        assert "completed" in str(app.state_badge.cget("text")).lower()
        assert float(app.progress.cget("value")) == pytest.approx(100)
    finally:
        app.close()


@desktop_process
def test_practical_goals_sensors_and_arm_share_the_desktop_app(tmp_path):
    import tkinter as tk
    from unittest.mock import patch
    from robotics_sim.app import SimulatorApp, enable_high_dpi
    from robotics_sim.session import LESSONS, TASKS
    from robotics_sim.exercises.lesson03.waypoint import StudentWaypointController

    enable_high_dpi()
    root = tk.Tk()
    app = SimulatorApp(root, duration=30, dt=0.05, settings_path=tmp_path / "integrated.json")

    def choose(lesson, task):
        app.kinematics.set("reference")
        app.exercise_model.set("reference")
        app.duration.set("30")
        app.lesson.set(LESSONS[lesson])
        app.select_lesson()
        app.task.set(TASKS[lesson][task])
        app.load_task()
        root.update()
        app._render()

    def finish():
        for _ in range(650):
            if app.session.state == "Completed":
                break
            app.single_step()
        assert app.session.state == "Completed"
        app._render()

    try:
        choose("3", "drive_waypoints")
        assert any("Goal 1" == app.canvas.itemcget(i, "text")
                   for i in app.canvas.find_withtag("label-text"))
        finish()
        assert app.session.outcome == "DONE"
        assert len(app.session.arrivals) == 2
        app.kinematics.set("student")
        app.load_task()
        with patch.object(StudentWaypointController, "wheel_command",
                          side_effect=NotImplementedError("complete wheel_command")):
            app.single_step()
        assert "not implemented" in app.status.get().lower()
        assert app.session.sim.time == 0

        choose("4", "sensor_front_stop")
        for _ in range(5):
            app.single_step()
        assert "front rangefinder" in app.values.get()
        assert app.session.sensors.time > 0
        assert str(app.model_heading.cget("text")) == "STOP CONTROLLER MODEL"
        choose("4", "sensor_avoidance")
        app.single_step()
        assert "Goal error" in app.values.get()
        assert str(app.model_heading.cget("text")) == "SUPERVISOR MODEL"

        choose("5", "arm_control")
        finish()
        assert app.session.outcome == "DONE"
        assert app.session.laboratory.residual < 0.001
        assert any(app.canvas.itemcget(i, "text") == "Target"
                   for i in app.canvas.find_withtag("label-text"))
        app.goal_x.set("3")
        app.goal_y.set("0")
        app.load_task()
        app.single_step()
        assert app.session.outcome == "UNREACHABLE"
        assert "UNREACHABLE" in app.status.get()
    finally:
        app.close()


@desktop_process
def test_arm_shadows_move_show_both_ik_branches_and_reset(tmp_path):
    import tkinter as tk
    from robotics_sim.app import SimulatorApp, enable_high_dpi
    from robotics_sim.session import LESSONS, TASKS

    enable_high_dpi()
    root = tk.Tk()
    app = SimulatorApp(root, lesson="5", settings_path=tmp_path / "arm-shadows.json")

    def visible_shadows():
        return {key: item for key, item in app.renderer.items.items()
                if key[0] == -3 and isinstance(key[1], tuple)
                and app.canvas.itemcget(item, "state") != "hidden"}

    try:
        app.task.set(TASKS["5"]["arm_control"])
        app.load_task()
        root.update()
        app._render()
        initial = visible_shadows()
        assert len(initial) == 6
        before = {key: app.canvas.coords(item) for key, item in initial.items()}
        app.single_step()
        app._render()
        moved = visible_shadows()
        assert moved == initial  # Retain the canvas items as joints move.
        assert any(app.canvas.coords(item) != before[key] for key, item in moved.items())
        app.task.set(TASKS["5"]["arm_inverse"])
        app.load_task()
        app.single_step()
        app._render()
        assert len(app.session.laboratory.branches) == 2
        assert len(visible_shadows()) == 12
        app.load_task()
        app._render()
        assert len(visible_shadows()) == 6
        app.lesson.set(LESSONS["1"])
        app.select_lesson()
        app._render()
        assert not visible_shadows()
    finally:
        app.close()


@desktop_process
def test_plots_use_latest_record_and_survive_activity_reset(tmp_path):
    import tkinter as tk
    from robotics_sim.app import SimulatorApp, enable_high_dpi
    from robotics_sim.session import TASKS

    enable_high_dpi()
    root = tk.Tk()
    app = SimulatorApp(root, lesson="3", duration=0.2, dt=0.05,
                       settings_path=tmp_path / "plots.json")
    try:
        root.geometry("900x620+20+20")
        root.update()
        assert str(app.plot_button.cget("state")) == "disabled"
        for _ in range(4):
            app.single_step()
        assert app.session.state == "Completed"
        assert str(app.plot_button.cget("state")) == "normal"
        app.plot_button.invoke()
        root.update()
        assert app._plot_window.winfo_exists()
        assert len(app._plot_figure.axes) >= 2
        assert app.last_experiment["task"] == "drive_line"
        first_samples = app.last_experiment["samples"]

        app.task.set(TASKS["3"]["drive_goal_behind"])
        app.load_task()
        assert not app.session.samples
        app.plot_button.invoke()
        root.update()
        assert app.last_experiment["task"] == "drive_line"
        assert app.last_experiment["samples"] == first_samples

        app.single_step()
        app.plot_button.invoke()
        root.update()
        assert app.last_experiment["task"] == "drive_goal_behind"
        assert app._plot_canvas is not None
        x = app.plot_button.winfo_rootx() - root.winfo_rootx()
        assert x >= 0 and x + app.plot_button.winfo_width() <= root.winfo_width()
        app._close_plot()
        assert app._plot_window is None
    finally:
        app.close()
