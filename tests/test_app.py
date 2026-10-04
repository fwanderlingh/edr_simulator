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
        assert {"World", "Robot", "Sensor", "Sensor point", "World goal",
                "Direction (no translation)"} <= labels
        assert "Module 1 output: point in world (2.80, 2.50) m" in app.values.get()
        assert "direction in world (0.00, 1.00)" in app.values.get()
        assert "Goal in robot: (2.00, -2.00) m" in app.values.get()
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
        for title in TASKS["3"].values():
            app.task.set(title)
            app.load_task()
            app.single_step()
            app._render()
            assert len(app.renderer.scene.shapes) == 4
            assert "Wheels L/R" in app.values.get()
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

    root = tk.Tk()
    app = SimulatorApp(root, settings_path=path)
    try:
        root.update()
        assert (root.winfo_width(), root.winfo_height()) == expected_size
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
        assert app.session.task == "square"
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
