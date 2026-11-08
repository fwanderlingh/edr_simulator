"""Compare vector visibility with independent physics rays while orbiting."""

import numpy as np
from PIL import Image, ImageDraw
import pytest

from robotics_sim.session import ActivitySession
from robotics_sim.sim.camera import Camera
from robotics_sim.sim.scene_renderer import BoxScene


def visibility_mismatches(session, scene, camera):
    """Rasterize ownership only; physics rays independently give the visible body."""
    camera.prepare(720, 520)
    owners = Image.new("I", (camera.width, camera.height), 0)
    edges = Image.new("L", owners.size, 0)
    paint, edge = ImageDraw.Draw(owners), ImageDraw.Draw(edges)
    for face in scene.faces(camera):
        if face.key[0] <= 0:
            continue
        points = list(zip(face.coordinates[::2], face.coordinates[1::2]))
        paint.polygon(points, fill=scene.shapes[face.key[0]][0])
        edge.line(points + [points[0]], fill=255, width=5)
    # Exclude polygon edges: Tk/Pillow rounding and ray intersection tolerances
    # differ there. Decorative wheels have visual shapes but no collision shape.
    pixels = [(x, y) for y in range(7, camera.height, 9)
              for x in range(7, camera.width, 9) if edges.getpixel((x, y)) == 0]
    inv = np.linalg.inv(camera.transform)
    ndc = np.array([(2 * (x + .5) / camera.width - 1,
                     1 - 2 * (y + .5) / camera.height, 1, 1) for x, y in pixels])
    far = ndc @ inv.T
    far = far[:, :3] / far[:, 3, None]
    hits = session.world.raycast([camera.eye] * len(far), far)
    compared, mismatches = 0, []
    for pixel, (body, _) in zip(pixels, hits):
        visible = owners.getpixel(pixel)
        if visible in session.robot.wheels or body == -1:
            continue
        compared += 1
        if visible != body:
            mismatches.append((pixel, visible, body))
    return compared, mismatches


@pytest.mark.parametrize("yaw,pitch", [
    (0, -30), (45, -55), (90, -35), (135, -65),
    (180, -30), (225, -55), (270, -35), (315, -65),
])
def test_escape_room_occlusion_matches_physics_from_all_sides(yaw, pitch):
    session = ActivitySession("8")
    try:
        scene, camera = BoxScene(session.world), Camera()
        for name, value in session.world.preview_camera.items():
            setattr(camera, name, value)
        camera.yaw, camera.pitch = yaw, pitch
        compared, mismatches = visibility_mismatches(session, scene, camera)
        assert compared > 500
        assert not mismatches, f"Wrong occlusion at yaw={yaw}, pitch={pitch}: {mismatches[:8]}"
        # Orbiting reuses the geometry partition rather than rebuilding it.
        tree = scene._surface_tree
        camera.yaw += 3
        camera.prepare(720, 520)
        scene.faces(camera)
        assert scene._surface_tree is tree
    finally:
        session.close()
