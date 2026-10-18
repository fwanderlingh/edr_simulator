"""Independent numerical checks against the notes."""

import numpy as np
import pytest

from robotics_sim.exercises.lesson02 import module2, spatial, rpy, quaternions
from robotics_sim.geometry.rotations import quaternion_to_matrix
from robotics_sim.session import ActivitySession, TASKS, ACTIVITY_FILES


@pytest.mark.solution
def test_spatial_sensor_pose_and_quaternion_cross_check():
    frame, point = spatial.solve_spatial_task(
        module2.WORLD_FROM_BASE, module2.BASE_FROM_SENSOR, module2.POINT_IN_SENSOR)
    expected = [[0, 0, 1, 1], [1, 0, 0, 2.2], [0, 1, 0, .5], [0, 0, 0, 1]]
    np.testing.assert_allclose(frame, expected, atol=1e-12)
    np.testing.assert_allclose(point, [1, 2.2, 1.5], atol=1e-12)
    a = np.sqrt(.5)
    q = quaternions.hamilton((a, 0, 0, a), (a, a, 0, 0))
    np.testing.assert_allclose(q, [.5] * 4)
    np.testing.assert_allclose(quaternion_to_matrix(q[[1, 2, 3, 0]]), frame[:3, :3], atol=1e-12)


@pytest.mark.solution
def test_rpy_order_and_singularity():
    rotation, reverse = rpy.solve_rpy_task(np.pi / 2, 0, np.pi / 2)
    np.testing.assert_allclose(rotation, [[0, 0, 1], [1, 0, 0], [0, 1, 0]], atol=1e-12)
    np.testing.assert_allclose(reverse, [[0, -1, 0], [0, 0, -1], [1, 0, 0]], atol=1e-12)
    for angles in ((0, np.pi/2, 0), (np.pi/6, np.pi/2, np.pi/6)):
        rotation, _ = rpy.solve_rpy_task(*angles)
        np.testing.assert_allclose(rotation, [[0, 0, 1], [0, 1, 0], [-1, 0, 0]], atol=1e-12)


@pytest.mark.solution
def test_quaternion_inverse_composition_rotation_and_sign():
    a = np.sqrt(.5)
    for scale in (1, -1, 3):
        q = scale * np.array([a, 0, 0, a])
        inverse, square, vector, distance = quaternions.solve_quaternion_task(q, [0, 1, 0])
        np.testing.assert_allclose(inverse, np.sign(scale) * np.array([a, 0, 0, -a]))
        np.testing.assert_allclose(square, [0, 0, 0, 1], atol=1e-12)
        np.testing.assert_allclose(vector, [-1, 0, 0], atol=1e-12)
        assert distance == pytest.approx(0, abs=1e-7)
    with pytest.raises(ValueError):
        quaternions.solve_quaternion_task([0, 0, 0, 0], [0, 1, 0])


@pytest.mark.parametrize('task', module2.TASKS)
def test_module2_lifecycle_and_unfinished_exercise(task, monkeypatch):
    from robotics_sim.app import SimulatorApp
    assert task in TASKS['2'] and ACTIVITY_FILES[task][1].endswith(task + '.py')
    session = ActivitySession('2', task, duration=.02, dt=.01)
    try:
        viewer = SimulatorApp.__new__(SimulatorApp)
        viewer.session = session
        def unfinished(task):
            raise NotImplementedError
        monkeypatch.setattr(module2, 'evaluate', unfinished)
        assert viewer._module2_output() is None
        session.start()
        session.step()
        session.step()
        assert session.state == 'Completed'
        assert session.robot.command.omega == 0
    finally:
        session.close()


@pytest.mark.solution
@pytest.mark.parametrize('task', module2.TASKS)
def test_scene_annotations_are_finite(task):
    frames, arrows, readout = module2.evaluate(task)
    assert 'E2.' in readout
    for transform, _ in frames:
        assert transform.shape == (4, 4)
        assert np.all(np.isfinite(transform))
    for start, end, _, _ in arrows:
        assert np.shape(start) == np.shape(end) == (3,)
        assert np.all(np.isfinite(end))
