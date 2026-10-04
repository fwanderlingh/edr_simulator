"""Retained shaded box and cylinder faces for the introductory worlds.

Read visual shapes once per world and poses from PyBullet. Project only the
handful of visible faces instead of rasterizing/copying millions of pixels.
The painter's algorithm is intended for these opaque, non-intersecting boxes;
complex meshes/transparency will need a different viewport backend.
"""

from dataclasses import dataclass
from itertools import product

import numpy as np
import pybullet as bullet

from ..geometry.rotations import quaternion_to_matrix


@dataclass
class Face:
    key: tuple
    coordinates: tuple
    depth: float
    color: str


def clip_polygon(vertices):
    """Clip homogeneous vertices against all six view-frustum planes."""
    for axis, sign in ((0, 1), (0, -1), (1, 1), (1, -1), (2, 1), (2, -1)):
        if not len(vertices):
            break
        output = []
        previous = vertices[-1]
        previous_distance = previous[3] + sign * previous[axis]
        for current in vertices:
            distance = current[3] + sign * current[axis]
            if (distance >= 0) != (previous_distance >= 0):
                fraction = previous_distance / (previous_distance - distance)
                output.append(previous + fraction * (current - previous))
            if distance >= 0:
                output.append(current)
            previous, previous_distance = current, distance
        vertices = np.asarray(output)
    return vertices


class BoxScene:
    """Project actual physics poses, caching unchanged bodies and camera views."""

    corners = np.array(list(product((-0.5, 0.5), repeat=3)))
    sides = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
             (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))
    normals = np.array(((-1, 0, 0), (1, 0, 0), (0, -1, 0),
                        (0, 1, 0), (0, 0, -1), (0, 0, 1)))

    def __init__(self, world):
        self.world = world
        self.shapes = []
        self._cache = {}
        self._grid_key = None
        self._grid_faces = []
        client = world.physics.client
        for index in range(bullet.getNumBodies(physicsClientId=client)):
            body = bullet.getBodyUniqueId(index, physicsClientId=client)
            for shape in bullet.getVisualShapeData(body, physicsClientId=client):
                if shape[1] != -1 or shape[2] not in (bullet.GEOM_BOX, bullet.GEOM_CYLINDER):
                    raise ValueError("The introductory viewport supports base-link boxes and cylinders only.")
                if shape[2] == bullet.GEOM_BOX:
                    vertices, sides, normals = self.corners * shape[3], self.sides, self.normals
                else:
                    length, radius = shape[3][:2]
                    angles = np.arange(12) * (2 * np.pi / 12)
                    vertices = np.array([(radius * np.cos(a), radius * np.sin(a), z)
                                         for z in (-length / 2, length / 2) for a in angles])
                    sides = [tuple(range(11, -1, -1)), tuple(range(12, 24))]
                    normals = [(0, 0, -1), (0, 0, 1)]
                    for i, a in enumerate(angles + np.pi / 12):
                        j = (i + 1) % 12
                        sides.append((i, j, j + 12, i + 12))
                        normals.append((np.cos(a), np.sin(a), 0))
                    normals = np.asarray(normals)
                self.shapes.append((body, vertices, sides, normals, np.asarray(shape[5]),
                                    quaternion_to_matrix(shape[6]), np.asarray(shape[7][:3])))

    def faces(self, camera):
        faces = []
        for index, (body, local_vertices, sides, local_normals, offset, local_rotation, color) in enumerate(self.shapes):
            position, orientation = bullet.getBasePositionAndOrientation(body, physicsClientId=self.world.physics.client)
            key = camera.key, position, orientation
            cached = self._cache.get(index)
            if cached is None or cached[0] != key:
                rotation = quaternion_to_matrix(orientation)
                vertices = (local_vertices @ local_rotation.T + offset) @ rotation.T + position
                normals = local_normals @ local_rotation.T @ rotation.T
                homogeneous = np.column_stack((vertices, np.ones(len(vertices))))
                clip = homogeneous @ camera.transform.T
                depths = (homogeneous @ camera.view.T)[:, 2]
                projected = []
                for side, ids in enumerate(sides):
                    center = vertices[list(ids)].mean(axis=0)
                    normal = normals[side]
                    if np.dot(normal, camera.eye - center) <= 0:
                        continue
                    polygon = clip_polygon(clip[list(ids)])
                    if len(polygon) < 3:
                        continue
                    ndc = polygon[:, :2] / polygon[:, 3, None]
                    pixels = (ndc + (1, -1)) * (camera.width / 2, -camera.height / 2)
                    shade = 0.65 + 0.35 * max(0, np.dot(normal, np.array((0.3, -0.4, 0.866))))
                    rgb = np.clip(color * shade * 255, 0, 255).astype(int)
                    projected.append(Face((index, side), tuple(pixels.ravel()),
                                          float(depths[list(ids)].mean()), "#%02x%02x%02x" % tuple(rgb)))
                self._cache[index] = key, projected
            faces.extend(self._cache[index][1])
        # The floor is necessarily behind the other bodies; its large extent
        # makes mean-depth sorting inaccurate when the camera crosses its edge.
        faces.extend(self.ground_grid(camera))
        return sorted(faces, key=lambda face: (
            0 if face.key[0] == 0 else 1 if face.key[0] == -1 else 2, face.depth))

    def ground_grid(self, camera):
        """One-metre grey grid on the 10 m floor, cached until the view changes."""
        if camera.key == self._grid_key:
            return self._grid_faces
        self._grid_key = camera.key
        self._grid_faces = []
        # Thin ground-plane strips use the same clipping and retained polygons
        # as scene faces. Draw above the floor and below all robot/obstacle faces.
        for axis in range(2):
            for coordinate in range(-5, 6):
                low, high = max(-5, coordinate - 0.006), min(5, coordinate + 0.006)
                vertices = np.array([(low, -5, 0.001, 1), (high, -5, 0.001, 1),
                                     (high, 5, 0.001, 1), (low, 5, 0.001, 1)])
                if axis == 1:
                    vertices[:, [0, 1]] = vertices[:, [1, 0]]
                polygon = clip_polygon(vertices @ camera.transform.T)
                if len(polygon) < 3:
                    continue
                ndc = polygon[:, :2] / polygon[:, 3, None]
                pixels = (ndc + (1, -1)) * (camera.width / 2, -camera.height / 2)
                self._grid_faces.append(Face((-1, axis * 11 + coordinate + 5),
                    tuple(pixels.ravel()), 0, "#BCBCBC"))
        return self._grid_faces


class SceneRenderer:
    """Keep Canvas polygons alive; update only changed coordinates/styles."""

    def __init__(self, canvas):
        self.canvas = canvas
        self.scene = None
        self.items = {}
        self._values = {}
        self._order = ()

    def draw(self, world, camera, width, height):
        if self.scene is None or self.scene.world is not world:
            self.canvas.delete("scene")
            self.scene = BoxScene(world)
            self.items.clear()
            self._values.clear()
            self._order = ()
        camera.prepare(width, height)
        faces = self.scene.faces(camera)
        visible = {face.key for face in faces}
        for key in self.items.keys() - visible:
            if self._values.get(key) is not None:
                self.canvas.itemconfigure(self.items[key], state="hidden")
                self._values[key] = None
        for face in faces:
            value = face.coordinates, face.color
            if face.key not in self.items:
                self.items[face.key] = self.canvas.create_polygon(
                    *face.coordinates, fill=face.color, outline="", tags="scene")
            elif self._values.get(face.key) != value:
                self.canvas.coords(self.items[face.key], *face.coordinates)
                self.canvas.itemconfigure(self.items[face.key], fill=face.color, state="normal")
            self._values[face.key] = value
        order = tuple(face.key for face in faces)
        if order != self._order:
            for key in reversed(order):
                self.canvas.tag_lower(self.items[key])
            self._order = order
