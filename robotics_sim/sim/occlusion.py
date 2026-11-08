"""Cached BSP ordering for opaque primitive scenes with long intersecting faces.

Split polygons in world space once, then traverse them back to front for each
camera. An average face depth cannot order a long wall against a nearby prop.
This keeps the retained vector viewport instead of adding a raster backend.
"""

from dataclasses import dataclass, replace

import numpy as np


EPSILON = 1e-8


@dataclass
class Surface:
    key: tuple
    vertices: np.ndarray
    normal: np.ndarray
    color: str


def split_surface(surface, origin, normal):
    """Return back/front fragments, preserving winding and stable fragment keys."""
    distances = (surface.vertices - origin) @ normal
    back, front = [], []
    previous, previous_distance = surface.vertices[-1], distances[-1]
    for current, distance in zip(surface.vertices, distances):
        if (previous_distance < -EPSILON and distance > EPSILON
                or previous_distance > EPSILON and distance < -EPSILON):
            point = previous + (current - previous) * previous_distance / (previous_distance - distance)
            back.append(point)
            front.append(point)
        if distance <= EPSILON:
            back.append(current)
        if distance >= -EPSILON:
            front.append(current)
        previous, previous_distance = current, distance
    return (replace(surface, key=surface.key + (0,), vertices=np.asarray(back)),
            replace(surface, key=surface.key + (1,), vertices=np.asarray(front)))


class SurfaceTree:
    def __init__(self, surfaces):
        # Static scene construction already puts the enclosing wall planes first.
        splitter = surfaces[0]
        self.origin, self.normal = splitter.vertices[0], splitter.normal
        self.coplanar, back, front = [], [], []
        for surface in surfaces:
            distances = (surface.vertices - self.origin) @ self.normal
            behind, ahead = np.any(distances < -EPSILON), np.any(distances > EPSILON)
            if behind and ahead:
                b, f = split_surface(surface, self.origin, self.normal)
                back.append(b)
                front.append(f)
            elif behind:
                back.append(surface)
            elif ahead:
                front.append(surface)
            else:
                self.coplanar.append(surface)
        self.back = SurfaceTree(back) if back else None
        self.front = SurfaceTree(front) if front else None

    def ordered(self, eye):
        """Yield polygons in a valid painter order from either side of a plane."""
        near, far = ((self.front, self.back) if np.dot(eye - self.origin, self.normal) >= 0
                     else (self.back, self.front))
        if far is not None:
            yield from far.ordered(eye)
        yield from self.coplanar
        if near is not None:
            yield from near.ordered(eye)
