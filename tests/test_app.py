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
                    "ReferenceProportionalController" if key == "drive_proportional" else "ReferenceWaypointController")
            else:
                assert app.session.robot.motor_model.__class__.__name__ == "StudentDifferentialDrive"
        app.lesson.set(LESSONS["4"])
        app.select_lesson()
        for mode, task in (("ideal", "sensor_encoders"), ("noisy", "sensor_imu"),
                           ("student", "sensor_lidar")):
            app.sensor_mode.set(mode)
            app.task.set(TASKS["4"][task])
            app.load_task()
            app.single_step()
            app._render()
            assert app.session.sensors.time == 0  # Cached until next sensor sample.
            while app.session.state != "Completed":
                app.single_step()
            assert app.session.sensors.time == pytest.approx(0.04)
            assert "Encoder L/R" in app.values.get()
            assert "Gyro z" in app.values.get()
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

    application.solve_planar_task = todo
    application.point_in_world = todo
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
    from robotics_sim.exercises.lesson03.control import StudentWaypointController

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
        with patch.object(StudentWaypointController, "nominal_command",
                          side_effect=NotImplementedError("complete nominal_command")):
            app.single_step()
        assert "not implemented" in app.status.get().lower()
        assert app.session.sim.time == 0

        choose("4", "sensor_estimation_curve")
        for _ in range(5):
            app.single_step()
        assert "Encoder error" in app.values.get()
        assert app.session.sensors.time > 0
        assert str(app.model_heading.cget("text")) == "ESTIMATOR MODEL"
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
