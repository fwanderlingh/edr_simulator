"""Ground guides are clipped and painted below opaque objects."""
import unittest
import numpy as np
from robotics_sim.session import ActivitySession
from robotics_sim.sim.camera import Camera
from robotics_sim.sim.scene_renderer import BoxScene


class GroundPathTests(unittest.TestCase):
    def setUp(self):
        self.session = ActivitySession(lesson="1", task="square")
        self.addCleanup(self.session.close)
        self.camera = Camera()
        self.scene = BoxScene(self.session.world)

    def test_layer_order_and_clipping_across_camera_angles(self):
        paths = (([(0, 0), (2, 0), (2, 2), (0, 2), (0, 0)], "#b5660b"),)
        for yaw, pitch in ((35, -55), (120, -20), (-60, -80)):
            with self.subTest(yaw=yaw, pitch=pitch):
                self.camera.yaw, self.camera.pitch = yaw, pitch
                self.camera.prepare(1000, 700)
                faces = self.scene.faces(self.camera, paths)
                guides = [i for i, f in enumerate(faces) if f.key[0] == -2]
                grid = [i for i, f in enumerate(faces) if f.key[0] == -1]
                solids = [i for i, f in enumerate(faces) if f.key[0] > 0]
                self.assertEqual(len(guides), 4)
                self.assertLess(max(grid), min(guides))
                self.assertLess(max(guides), min(solids))
                for i in guides:
                    pixels = np.array(faces[i].coordinates).reshape(-1, 2)
                    self.assertTrue(np.isfinite(pixels).all())
                    self.assertTrue((pixels >= -1e-6).all())
                    self.assertTrue((pixels <= (1000 + 1e-6, 700 + 1e-6)).all())
        self.assertFalse(any(f.key[0] == -2 for f in self.scene.faces(self.camera)))

    def test_degenerate_and_offscreen_segments_are_omitted(self):
        self.camera.prepare(800, 600)
        for points in ([(0, 0), (0, 0)], [(1000, 1000), (1001, 1000)]):
            self.assertEqual(self.scene.ground_paths(self.camera, ((points, "red"),)), [])
