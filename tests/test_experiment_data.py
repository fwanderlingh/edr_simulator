"""Experiment plots must survive reset and share the exported CSV schema."""

import csv

import pytest

from robotics_sim.experiment_data import capture_experiment
from robotics_sim.session import ActivitySession


@pytest.mark.parametrize("lesson,task", [
    ("3", "drive_waypoints"), ("4", "sensor_estimation_curve"),
    ("4", "sensor_lidar"), ("5", "arm_inverse"), ("5", "arm_control"),
])
def test_snapshot_detaches_and_matches_csv(lesson, task, tmp_path):
    session = ActivitySession(lesson, task, duration=0.2, dt=0.05)
    try:
        assert capture_experiment(session) is None
        session.start()
        while session.state == "Running":
            session.step()
        snapshot = capture_experiment(session)
        path = tmp_path / "record.csv"
        session.export_csv(path)
        with path.open(encoding="utf-8") as stream:
            exported = list(csv.reader(stream))
        assert tuple(exported[0]) == snapshot["columns"]
        assert len(exported) - 1 == len(snapshot["samples"])
        if session.sensors is not None:
            assert snapshot["sensor_samples"][-1][0] == pytest.approx(session.sensors.time)
            assert "lidar_angles" in snapshot["metadata"]
        if lesson == "5":
            assert "arm_points" in snapshot["metadata"]
        before = snapshot["samples"][0]
        session.samples.clear()
        assert snapshot["samples"][0] == before
        assert snapshot["title"]
    finally:
        session.close()
    assert snapshot["samples"]
