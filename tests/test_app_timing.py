"""Exercise resize scheduling without opening a desktop window."""

from itertools import count
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from robotics_sim import app as app_module
from robotics_sim.app import SimulatorApp
from robotics_sim.session import ActivitySession


@pytest.fixture
def app_clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(app_module.time, "perf_counter", lambda: now[0])
    app = SimulatorApp.__new__(SimulatorApp)
    app.root = Mock()
    identifiers = count()
    app.root.after.side_effect = lambda *args: f"timer-{next(identifiers)}"
    app.session = ActivitySession(duration=3, dt=.01)
    app.dt = .01
    app.closed = False
    app._reload_failed = False
    app._accumulator = 0.0
    app._last_tick = now[0]
    app._dirty = True
    app._capture_window_geometry = Mock()
    app._queue_save = Mock()
    app._after_steps = Mock()
    app._render = Mock()
    app._remember_experiment = Mock()
    app._update_status = Mock()
    app._save_timer = None
    app._save_settings = Mock()
    app._close_plot = Mock()
    app._label_images = {}
    app._timer = "tick"
    app._restore_timer = "restore"
    try:
        yield app, now
    finally:
        if not app.closed:
            app.session.close()


def resize(app, width, height):
    app._window_changed(SimpleNamespace(widget=app.root, width=width, height=height))
    app._redraw(SimpleNamespace(width=width, height=height))


def test_continuous_resize_keeps_physics_measurements_and_drawing_running(app_clock):
    app, now = app_clock
    app.session.start()
    app.session.step()
    original_samples = list(app.session.samples)
    start = now[0]
    for frame in range(60):
        steps = app.session.sim.steps
        for event in range(5):
            resize(app, 1000 + frame * 5 + event, 700 + frame)
        now[0] += .0175
        app._tick()
        assert app.session.sim.steps > steps
        assert app.session.state == "Running"
        assert app.session.sim.time == pytest.approx(.01 + now[0] - start, abs=app.dt)
    assert app.session.samples[:len(original_samples)] == original_samples
    assert app.session.robot.pose().x > 0
    assert app._render.call_count == 60
    assert app._after_steps.call_count == 60


def test_resize_preserves_partial_physics_step_and_elapsed_time(app_clock):
    app, now = app_clock
    app.session.start()
    app._accumulator = .006
    now[0] += .006
    resize(app, 1100, 750)
    app._tick()
    assert app.session.sim.steps == 1
    assert app._accumulator == pytest.approx(.002)


def test_resize_events_are_combined_into_one_frame(app_clock):
    app, _ = app_clock
    for width in range(1000, 1100):
        resize(app, width, 750)
    app._render.assert_not_called()
    app._tick()
    app._render.assert_called_once()
    app._tick()
    app._render.assert_called_once()


def test_each_frame_is_presented_before_more_physics_steps(app_clock):
    app, now = app_clock
    app.session.start()
    pending = []
    presented = []
    app._render.side_effect = lambda: pending.append(app.session.robot.pose())

    def paint():
        presented.extend(pending)
        pending.clear()

    app.root.update_idletasks.side_effect = paint
    for frame in range(5):
        resize(app, 1000 + frame, 750)
        now[0] += .02
        app._tick()
        assert len(presented) == frame + 1
        assert presented[-1] == app.session.robot.pose()
        assert not pending
    assert all(after.x > before.x for before, after in zip(presented, presented[1:]))


def test_painting_a_resize_keeps_the_next_redraw_request(app_clock):
    app, _ = app_clock
    app.root.update_idletasks.side_effect = lambda: resize(app, 1100, 750)
    app._tick()
    assert app._dirty
    app._tick()
    assert app._render.call_count == 2


@pytest.mark.parametrize("state", ["Ready", "Paused", "Completed"])
def test_resize_does_not_start_an_inactive_session(app_clock, state):
    app, now = app_clock
    app.session.state = state
    resize(app, 1100, 750)
    now[0] += .1
    app._tick()
    assert app.session.state == state
    assert app.session.sim.steps == 0


def test_manual_pause_during_resize_is_preserved(app_clock):
    app, now = app_clock
    app.session.start()
    resize(app, 1100, 750)
    app.toggle_running()
    now[0] += .1
    app._tick()
    assert app.session.state == "Paused"
    assert app.session.sim.steps == 0


def test_move_and_child_layout_events_do_not_suspend_simulation(app_clock):
    app, now = app_clock
    app.session.start()
    resize(app, 1000, 700)
    app._window_changed(SimpleNamespace(widget=object(), width=200, height=300))
    now[0] += .015
    app._tick()
    assert app.session.sim.steps == 1


def test_close_after_resize_cancels_the_frame_callback_and_closes_physics(app_clock):
    app, _ = app_clock
    resize(app, 1100, 750)
    app.close()
    app.root.after_cancel.assert_any_call("tick")
    assert not app.session.world.physics.connected

