"""Render recorded experiments without opening Tk or reading live sensors."""

import csv

import matplotlib
matplotlib.use("Agg")
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np
import pytest

from robotics_sim.experiment_plots import create_experiment_figure
from robotics_sim.session import ActivitySession


def snapshot_of(session, tmp_path):
    path = tmp_path / "record.csv"
    session.export_csv(path)
    with path.open() as stream:
        reader = csv.reader(stream)
        columns, samples = tuple(next(reader)), tuple(tuple(row) for row in reader)
    metadata = {}
    if session.lesson == "5":
        lab = session.laboratory
        metadata.update(arm_points=lab.points.tolist(), arm_branches=[b.tolist() for b in lab.branches],
                        arm_predictions=list(lab.predictions), arm_target=lab.target)
    if session.sensors is not None:
        metadata.update(lidar_angles=session.sensors.lidar.angles.copy(),
                        lidar_max_range=session.sensors.lidar.max_range)
    return dict(lesson=session.lesson, task=session.task, title=session.task,
                columns=columns, samples=samples,
                sensor_columns=tuple(session.sensors.columns) if session.sensors else (),
                sensor_samples=tuple(session.sensors.rows) if session.sensors else (),
                goals=tuple(session.controller.goals) if session.controller else (),
                outcome=session.outcome, metadata=metadata)


@pytest.mark.parametrize("lesson,task", [
    ("1", "square"), ("2", "planar"), ("3", "drive_line"), ("3", "drive_waypoints"),
    ("4", "sensor_encoders"), ("4", "sensor_imu"), ("4", "sensor_lidar"),
    ("4", "sensor_estimation_curve"), ("4", "sensor_avoidance"),
    ("5", "arm_forward"), ("5", "arm_inverse"), ("5", "arm_control"),
])
def test_all_practical_activity_plots_render(lesson, task, tmp_path):
    session = ActivitySession(lesson, task, duration=.4, dt=.02, exercise_model="reference")
    try:
        session.start()
        while session.state == "Running":
            session.step()
        snapshot = snapshot_of(session, tmp_path)
    finally:
        session.close()
    figure = create_experiment_figure(snapshot)
    FigureCanvasAgg(figure).draw()
    assert len(figure.axes) == 4
    assert task in figure._suptitle.get_text()
    image = tmp_path / f"{task}.png"
    figure.savefig(image)
    assert image.stat().st_size > 10_000
    if lesson == "5":
        assert figure.axes[0].get_title() == "Endpoint and arm geometry"
        assert any("Joint" in ax.get_title() for ax in figure.axes)
    elif task == "sensor_estimation_curve":
        assert len(figure.axes[0].lines) == 3
        assert {line.get_label() for line in figure.axes[1].lines} == {"Encoder", "IMU"}


def test_empty_experiment_has_actionable_message():
    with pytest.raises(ValueError, match="Run or step"):
        create_experiment_figure({"task": "drive_goal", "columns": (), "samples": ()})


def test_sensor_plot_uses_acquisition_times_and_preserves_miss_flags():
    snapshot = dict(task="sensor_encoders", title="Ranges", lesson="4",
        columns=("time_s", "x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad"),
        samples=((.02, 0, 0, .16, 0, 0, 0), (.04, 0, 0, .16, 0, 0, 0)),
        sensor_columns=("time_s", "left_delta_rad", "right_delta_rad", "range_0_m", "hit_0"),
        sensor_samples=((0, 0, 0, 4, 0), (.1, .1, .1, 2, 1), (.2, .1, .1, 4, 0)))
    figure = create_experiment_figure(snapshot)
    FigureCanvasAgg(figure).draw()
    encoder = figure.axes[0].lines[0]
    np.testing.assert_allclose(encoder.get_xdata(), [0, .1, .2])
    measured = next(line for line in figure.axes[1].lines if line.get_label() == "Measured hit")
    assert np.isnan(measured.get_ydata()[0]) and np.isnan(measured.get_ydata()[2])
    assert measured.get_ydata()[1] == 2
    assert any(collection.get_label() == "No return (range limit)"
               for collection in figure.axes[1].collections)


def test_estimates_use_sensor_times_without_repeated_state_samples(tmp_path):
    session = ActivitySession("4", "sensor_estimation_curve", duration=.35, dt=.01)
    try:
        session.start()
        while session.state == "Running":
            session.step()
        snapshot = snapshot_of(session, tmp_path)
    finally:
        session.close()
    figure = create_experiment_figure(snapshot)
    time = figure.axes[2].lines[0].get_xdata()
    np.testing.assert_allclose(time, [0, .1, .2, .3, .35])
    assert len(time) < len(snapshot["samples"])


def test_static_inverse_plots_both_configurations_not_a_stationary_base(tmp_path):
    session = ActivitySession("5", "arm_inverse")
    try:
        session.step(single=True)
        snapshot = snapshot_of(session, tmp_path)
    finally:
        session.close()
    figure = create_experiment_figure(snapshot)
    labels = {line.get_label() for line in figure.axes[0].lines}
    assert {"Branch 1", "Branch 2"} <= labels
    assert len(figure.axes[1].patches) == 4
    assert len(figure.axes[2].patches) == 2
