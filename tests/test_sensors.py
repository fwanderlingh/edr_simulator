import csv
import math

import numpy as np
import pytest

from robotics_sim import Pose, World
from robotics_sim.session import ActivitySession
from robotics_sim.sensors.encoder import EncoderSensor
from robotics_sim.sensors.lidar import LidarSensor


def test_encoder_increments_quantization_and_cached_reads():
    ideal = EncoderSensor()
    noisy = EncoderSensor(noisy=True)
    measured = np.zeros(2)
    for _ in range(10):
        for sensor in (ideal, noisy):
            sensor.advance((2, -1), 0.1)
            sensor.sample()
        assert ideal.read() == pytest.approx((0.2, -0.1))
        assert noisy.read() == noisy.read()
        measured += noisy.read()
    assert measured / noisy.resolution == pytest.approx(np.round(measured / noisy.resolution))
    assert measured == pytest.approx((2, -1), abs=2 * noisy.resolution)


def test_lidar_known_distance_miss_and_rotated_mount():
    with World(gui=False) as world:
        world.add_box(Pose(x=2, z=0.5), size=(0.2, 1, 1))
        lidar = LidarSensor(world, angles=[0, math.pi / 2])
        assert lidar.sample(Pose()) == pytest.approx((1.9, 4))
        assert list(lidar.hits) == [True, False]
        assert lidar.sample(Pose(yaw=-math.pi / 2)) == pytest.approx((4, 1.9))


def test_sampling_pause_completion_and_sensor_csv(tmp_path):
    session = ActivitySession("4", "sensor_encoders", duration=0.25, dt=0.01)
    try:
        assert session.sensors.lidar.ranges == pytest.approx([2.9])
        session.start()
        session.step()
        session.pause()
        count = len(session.sensors.rows)
        session.step()
        assert len(session.sensors.rows) == count
        session.start()
        while session.state == "Running":
            session.step()
        assert [r[0] for r in session.sensors.rows] == pytest.approx([0, 0.1, 0.2, 0.25])
        assert session.sensors.encoder.read() == pytest.approx((0.0625, 0.0625))
        assert session.sensors.lidar.ranges == pytest.approx([2.9 - 0.15 * 0.25], abs=1e-5)
        session.export_csv(tmp_path / "test.csv")
        with (tmp_path / "test_sensors.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 4
        assert float(rows[-1]["time_s"]) == pytest.approx(0.25)
        assert rows[-1]["hit_0"] == "1"
        assert None not in rows[-1]
    finally:
        session.close()


def test_noise_is_repeatable_and_distinct_from_truth():
    sessions = [ActivitySession("4", "sensor_imu", duration=0.3, dt=0.01, sensor_mode=mode)
                for mode in ("noisy", "noisy", "ideal")]
    try:
        for session in sessions:
            session.start()
            while session.state == "Running":
                session.step()
        a, b, ideal = [session.sensors for session in sessions]
        assert np.asarray(a.rows) == pytest.approx(np.asarray(b.rows))
        assert a.imu.read() != ideal.imu.read()
        assert ideal.imu.read() == ideal.imu.truth
        assert ideal.imu.read()[0] == pytest.approx(0.4, abs=1e-6)
        assert ideal.imu.read()[1] > 0  # Increasing forward speed.
        assert ideal.imu.read()[2] > 0  # Centripetal acceleration to the left.
        assert np.all((a.lidar.ranges >= 0) & (a.lidar.ranges <= 4))
    finally:
        for session in sessions:
            session.close()
