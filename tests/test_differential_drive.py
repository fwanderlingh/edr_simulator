import csv
import math
from unittest.mock import Mock

import pytest

from robotics_sim.robots.differential_drive import ReferenceDifferentialDrive
from robotics_sim.exercises.lesson03.kinematics import StudentDifferentialDrive
from robotics_sim.session import ActivitySession
from robotics_sim.sim.scene_renderer import BoxScene
from robotics_sim.sim.camera import Camera


def test_forward_and_inverse_conventions():
    model = ReferenceDifferentialDrive(wheel_radius=0.1, track_width=0.5)
    assert model.forward(2, 2) == pytest.approx((0.2, 0))
    assert model.forward(-2, 2) == pytest.approx((0, 0.8))
    assert model.inverse(0.3, -0.4) == pytest.approx((4, 2))
    assert model.forward(*model.inverse(-0.3, 0.7)) == pytest.approx((-0.3, 0.7))


@pytest.mark.parametrize("task,method", [
    ("drive_line", "forward"), ("drive_circle", "forward"),
    ("drive_spin", "forward"), ("drive_inverse", "inverse"),
])
def test_unfinished_kinematics_can_open_stop_and_report_on_step(task, method, monkeypatch):
    forward = Mock(side_effect=NotImplementedError("complete forward()"))
    inverse = Mock(side_effect=NotImplementedError("complete inverse()"))
    monkeypatch.setattr(StudentDifferentialDrive, "forward", forward)
    monkeypatch.setattr(StudentDifferentialDrive, "inverse", inverse)
    session = ActivitySession("3", task, kinematics="student")
    try:
        assert session.state == "Ready" and session.sim.steps == 0
        assert session.robot.wheel_velocities == (0, 0)
        forward.assert_not_called()
        inverse.assert_not_called()
        with pytest.raises(NotImplementedError, match=f"complete {method}"):
            session.step(single=True)
        assert session.sim.steps == 0 and session.samples == []
        calls_before_stop = forward.call_count, inverse.call_count
        session.sim.stop()
        assert (forward.call_count, inverse.call_count) == calls_before_stop
        assert session.robot.wheel_velocities == (0, 0)
        assert session.robot.command.vx == session.robot.command.omega == 0
    finally:
        session.close()


@pytest.mark.parametrize("task,v,omega", [
    ("drive_line", 0.24, 0),
    ("drive_spin", 0, 0.24 / 0.42), ("drive_inverse", 0.24, 0.4)])
def test_paths_wheels_and_csv(task, v, omega, tmp_path):
    session = ActivitySession("3", task, duration=2)
    try:
        camera = Camera()
        camera.prepare(800, 600)
        scene = BoxScene(session.world)
        assert len(scene.shapes) == 4  # floor, chassis, two cylinders
        assert scene.faces(camera)
        session.start()
        while session.state == "Running":
            session.step()
        pose = session.robot.pose()
        expected_x = v * 2 if omega == 0 else v / omega * math.sin(omega * 2)
        expected_y = 0 if omega == 0 else v / omega * (1 - math.cos(omega * 2))
        assert (pose.x, pose.y, pose.yaw) == pytest.approx(
            (expected_x, expected_y, omega * 2), abs=0.002)
        assert session.robot.wheel_velocities == (0, 0)
        path = tmp_path / "drive.csv"
        session.export_csv(path)
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 480
        assert float(rows[-1]["forward_m_s"]) == pytest.approx(v)
        assert float(rows[-1]["yaw_rate_rad_s"]) == pytest.approx(omega)
    finally:
        session.close()
