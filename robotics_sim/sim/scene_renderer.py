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
    outline: str = ""
    line_width: float = 1.0


def convex_hull(points):
    """Small 2D monotone-chain hull used for ground-projected silhouettes."""
    points = sorted(set(map(tuple, points)))
    if len(points) <= 2:
        return np.asarray(points)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1] + upper[:-1])


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
    light = np.array((0.3, -0.4, 0.8660254))
    cylinder_segments = 20

    def __init__(self, world):
        self.world = world
        self.shapes = []
        self._cache = {}
        self._shadow_cache = {}
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
                    count = self.cylinder_segments
                    angles = np.arange(count) * (2 * np.pi / count)
                    vertices = np.array([(radius * np.cos(a), radius * np.sin(a), z)
                                         for z in (-length / 2, length / 2) for a in angles])
                    sides = [tuple(range(count - 1, -1, -1)), tuple(range(count, 2 * count))]
                    normals = [(0, 0, -1), (0, 0, 1)]
                    for i, a in enumerate(angles + np.pi / count):
                        j = (i + 1) % count
                        sides.append((i, j, j + count, i + count))
                        normals.append((np.cos(a), np.sin(a), 0))
                    normals = np.asarray(normals)
                self.shapes.append((body, vertices, sides, normals, np.asarray(shape[5]),
                                    quaternion_to_matrix(shape[6]), np.asarray(shape[7][:3])))

    def faces(self, camera, ground_paths=()):
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
                    shade = 0.62 + 0.30 * max(0, np.dot(normal, self.light)) + 0.08 * max(0, normal[2])
                    # A quiet floor leaves saturated lesson colours and vectors legible.
                    base_color = np.array((0.94, 0.955, 0.965)) if index == 0 else color
                    rgb = np.clip(base_color * shade * 255, 0, 255).astype(int)
                    outline = "" if index == 0 else "#%02x%02x%02x" % tuple((rgb * 0.83).astype(int))
                    projected.append(Face((index, side), tuple(pixels.ravel()),
                                          float(depths[list(ids)].mean()), "#%02x%02x%02x" % tuple(rgb),
                                          outline, 0.75))
                self._cache[index] = key, projected
                if index != 0:
                    self._shadow_cache[index] = self.contact_shadow(camera, index, vertices)
            faces.extend(self._cache[index][1])
            faces.extend(self._shadow_cache.get(index, ()))
        # The floor is necessarily behind the other bodies; its large extent
        # makes mean-depth sorting inaccurate when the camera crosses its edge.
        faces.extend(self.ground_grid(camera))
        faces.extend(self.ground_paths(camera, ground_paths))
        return sorted(faces, key=lambda face: (
            {0: 0, -3: 1, -1: 2, -2: 3}.get(face.key[0], 4), face.depth))

    def contact_shadow(self, camera, index, vertices):
        """Three cached opaque tonal bands approximate a soft ground shadow.

        This is a drawing aid on the introductory flat floor, not a collision
        surface or a general shadow map. Grid lines remain visible above it.
        """
        height = np.maximum(vertices[:, 2], 0)
        projected = vertices[:, :2] - height[:, None] * self.light[:2] / self.light[2]
        hull = convex_hull(projected)
        if len(hull) < 3:
            return []
        center = hull.mean(axis=0)
        result = []
        for layer, (spread, color) in enumerate(((1.10, "#dfe6eb"), (1.045, "#d5dfe6"), (1.0, "#cbd7e0"))):
            expanded = center + (hull - center) * spread
            homogeneous = np.column_stack((expanded, np.full(len(hull), 0.0005), np.ones(len(hull))))
            polygon = clip_polygon(homogeneous @ camera.transform.T)
            if len(polygon) < 3:
                continue
            ndc = polygon[:, :2] / polygon[:, 3, None]
            pixels = (ndc + (1, -1)) * (camera.width / 2, -camera.height / 2)
            result.append(Face((-3, index, layer), tuple(pixels.ravel()), layer, color))
        return result

    def ground_paths(self, camera, paths):
        """Project flat guide strips above the grid and below solid bodies.

        These paths lie on the floor, like the grid; they are not general
        elevated 3D lines. Clip their polygons before projecting them.
        """
        faces = []
        for path_index, (points, color) in enumerate(paths):
            for segment, (start, end) in enumerate(zip(points, points[1:])):
                start, end = np.asarray(start[:2]), np.asarray(end[:2])
                delta = end - start
                length = np.linalg.norm(delta)
                if length == 0:
                    continue
                half_width = 0.0125 * np.array([-delta[1], delta[0]]) / length
                corners = np.array([start - half_width, end - half_width,
                                    end + half_width, start + half_width])
                vertices = np.column_stack((corners, np.full(4, 0.002), np.ones(4)))
                polygon = clip_polygon(vertices @ camera.transform.T)
                if len(polygon) < 3:
                    continue
                ndc = polygon[:, :2] / polygon[:, 3, None]
                pixels = (ndc + (1, -1)) * (camera.width / 2, -camera.height / 2)
                faces.append(Face((-2, path_index, segment), tuple(pixels.ravel()), 0, color))
        return faces

    def ground_grid(self, camera):
        """Half-metre minor / one-metre major grid, cached with the view."""
        if camera.key == self._grid_key:
            return self._grid_faces
        self._grid_key = camera.key
        self._grid_faces = []
        # Thin ground-plane strips use the same clipping and retained polygons
        # as scene faces. Draw above the floor and below all robot/obstacle faces.
        for axis in range(2):
            for step in range(-10, 11):
                coordinate = step / 2
                major = step % 2 == 0
                thickness = 0.004 if major else 0.002
                low, high = max(-5, coordinate - thickness), min(5, coordinate + thickness)
                vertices = np.array([(low, -5, 0.001, 1), (high, -5, 0.001, 1),
                                     (high, 5, 0.001, 1), (low, 5, 0.001, 1)])
                if axis == 1:
                    vertices[:, [0, 1]] = vertices[:, [1, 0]]
                polygon = clip_polygon(vertices @ camera.transform.T)
                if len(polygon) < 3:
                    continue
                ndc = polygon[:, :2] / polygon[:, 3, None]
                pixels = (ndc + (1, -1)) * (camera.width / 2, -camera.height / 2)
                color = "#c9d3dc" if major else "#dde4e9"
                self._grid_faces.append(Face((-1, axis * 21 + step + 10),
                    tuple(pixels.ravel()), 0, color))
        return self._grid_faces


class SceneRenderer:
    """Keep Canvas polygons alive; update only changed coordinates/styles."""

    def __init__(self, canvas):
        self.canvas = canvas
        self.scene = None
        self.items = {}
        self._values = {}
        self._order = ()

    def draw(self, world, camera, width, height, *, ground_paths=()):
        if self.scene is None or self.scene.world is not world:
            self.canvas.delete("scene")
            self.scene = BoxScene(world)
            self.items.clear()
            self._values.clear()
            self._order = ()
        camera.prepare(width, height)
        faces = self.scene.faces(camera, ground_paths)
        visible = {face.key for face in faces}
        for key in self.items.keys() - visible:
            if self._values.get(key) is not None:
                self.canvas.itemconfigure(self.items[key], state="hidden")
                self._values[key] = None
        for face in faces:
            value = face.coordinates, face.color, face.outline, face.line_width
            if face.key not in self.items:
                self.items[face.key] = self.canvas.create_polygon(
                    *face.coordinates, fill=face.color, outline=face.outline,
                    width=face.line_width, joinstyle="round", tags="scene")
            elif self._values.get(face.key) != value:
                self.canvas.coords(self.items[face.key], *face.coordinates)
                self.canvas.itemconfigure(self.items[face.key], fill=face.color,
                                          outline=face.outline, width=face.line_width, state="normal")
            self._values[face.key] = value
        order = tuple(face.key for face in faces)
        if order != self._order:
            for key in reversed(order):
                self.canvas.tag_lower(self.items[key])
            self._order = order
