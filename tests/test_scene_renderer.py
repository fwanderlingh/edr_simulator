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


def test_contact_shadows_follow_physics_and_stay_below_guides():
    session = ActivitySession(lesson="1", task="square")
    try:
        camera = Camera()
        camera.prepare(1000, 700)
        scene = BoxScene(session.world)
        guide = (([(0, 0), (2, 0)], "#b5660b"),)
        first = scene.faces(camera, guide)
        static_shadow = scene._shadow_cache[1]
        robot_shadow = scene._shadow_cache[2]
        shadow_positions = [i for i, face in enumerate(first) if face.key[0] == -3]
        guide_positions = [i for i, face in enumerate(first) if face.key[0] == -2]
        assert shadow_positions
        assert max(shadow_positions) < min(guide_positions)
        assert all(face.outline for face in first if face.key[0] > 0)
        session.start()
        for _ in range(20):
            session.step()
        scene.faces(camera, guide)
        assert scene._shadow_cache[1] is static_shadow
        assert scene._shadow_cache[2] is not robot_shadow
        assert scene._shadow_cache[2][0].coordinates != robot_shadow[0].coordinates
    finally:
        session.close()


@pytest.mark.parametrize("yaw,pitch", [(35, -55), (120, -20), (-60, -80)])
def test_arm_shadows_follow_joints_and_stay_below_grid_and_base(yaw, pitch):
    session = ActivitySession("5", "arm_control")
    try:
        camera = Camera()
        camera.yaw, camera.pitch = yaw, pitch
        camera.prepare(1000, 700)
        scene = BoxScene(session.world)
        lab = session.laboratory
        first = scene.faces(camera, shadow_paths=((lab.points, .02),))
        shadows = [face for face in first if face.key[0] == -3 and isinstance(face.key[1], tuple)]
        assert len(shadows) == 6  # Two separate links, each with three soft tonal bands.
        ground_and_solids = [i for i, face in enumerate(first) if face.key[0] in (-1, 1)]
        assert max(i for i, face in enumerate(first) if face.key[0] == -3) < min(ground_and_solids)
        for face in shadows:
            pixels = np.asarray(face.coordinates).reshape(-1, 2)
            assert np.isfinite(pixels).all()
            assert np.all(pixels >= -1e-6)
            assert np.all(pixels <= (1000 + 1e-6, 700 + 1e-6))
        lab.q = np.array([1.0, -.4])
        moved = scene.faces(camera, shadow_paths=((lab.points, .02),))
        moved_shadows = [face for face in moved if face.key[0] == -3 and isinstance(face.key[1], tuple)]
        assert [face.coordinates for face in moved_shadows] != [face.coordinates for face in shadows]
        assert not any(isinstance(face.key[1], tuple) for face in scene.faces(camera) if face.key[0] == -3)
    finally:
        session.close()


def test_link_shadows_do_not_fill_the_elbow_and_clip_offscreen_geometry():
    session = ActivitySession("5", "arm_inverse")
    try:
        camera = Camera()
        camera.prepare(1000, 700)
        scene = BoxScene(session.world)
        bent = np.array([(0, 0, .2), (1, 0, .2), (1, 1, .2)])
        shadows = scene.path_shadows(camera, ((bent, .02),))
        assert {face.key[1] for face in shadows} == {("path", 0, 0), ("path", 0, 1)}
        # Each footprint stays close to its own ground-projected segment.
        # Filling the convex hull of the whole arm would cover the elbow interior.
        gap = camera.project((*((.55, .45) - .2 * scene.light[:2] / scene.light[2]), .0005))
        for face in shadows:
            polygon = np.asarray(face.coordinates).reshape(-1, 2)
            edges = np.roll(polygon, -1, axis=0) - polygon
            delta = gap - polygon
            cross = edges[:, 0] * delta[:, 1] - edges[:, 1] * delta[:, 0]
            assert not (np.all(cross >= 0) or np.all(cross <= 0))
        for points in ([(0, 0, .2), (0, 0, .2)], [(1000, 1000, .2), (1001, 1000, .2)]):
            assert scene.path_shadows(camera, ((points, .02),)) == []
    finally:
        session.close()
