import numpy as np
import pytest

from robotics_sim.session import ActivitySession
from robotics_sim.sim.camera import Camera
from robotics_sim.sim.scene_renderer import BoxScene, clip_polygon


@pytest.mark.solution
def test_projection_tracks_physics_and_reuses_static_faces():
    session = ActivitySession()
    try:
        camera = Camera()
        camera.prepare(1920, 1080)
        scene = BoxScene(session.world)
        first = scene.faces(camera)
        floor_cache, obstacle_cache = scene._cache[0], scene._cache[1]
        first_robot = [face.coordinates for face in first if face.key[0] == 2]
        session.start()
        for _ in range(20):
            session.step()
        second = scene.faces(camera)
        assert scene._cache[0] is floor_cache
        assert scene._cache[1] is obstacle_cache
        assert first_robot != [face.coordinates for face in second if face.key[0] == 2]
        assert all(face.key[0] == 0 for face in second[:len(floor_cache[1])])
        for face in second:
            pixels = np.array(face.coordinates).reshape(-1, 2)
            assert np.isfinite(pixels).all()
            assert np.all(pixels >= -1e-6)
            assert np.all(pixels <= (1920 + 1e-6, 1080 + 1e-6))
        assert camera.project(camera.target) == pytest.approx((960, 540), abs=0.01)
    finally:
        session.close()


def test_clipping_handles_a_face_crossing_near_plane():
    vertices = np.array([[-0.5, -0.5, -2, 1], [0.5, -0.5, 0, 1],
                         [0.5, 0.5, 0, 1], [-0.5, 0.5, -2, 1]], dtype=float)
    clipped = clip_polygon(vertices)
    assert len(clipped) == 4
    assert np.all(clipped[:, 2] >= -clipped[:, 3])
    assert np.isfinite(clipped).all()
