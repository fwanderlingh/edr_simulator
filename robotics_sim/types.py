"""Backend-independent values shared by lessons and replaceable modules."""

from dataclasses import dataclass
from math import isfinite

import numpy as np

from .geometry.transforms import homogeneous
from .geometry.rotations import rpy_to_matrix


@dataclass(frozen=True)
class Pose:
    """Body pose in the world; R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.16
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def __post_init__(self):
        if not all(isfinite(v) for v in (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)):
            raise ValueError("Pose components must be finite.")

    @property
    def position(self) -> tuple[float, float, float]:
        return self.x, self.y, self.z

    def matrix(self) -> np.ndarray:
        return homogeneous(rpy_to_matrix(self.roll, self.pitch, self.yaw), self.position)


@dataclass(frozen=True)
class Velocity:
    """Planar velocity: vx, vy in m/s; omega in rad/s; world or body frame."""

    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    frame: str = "world"

    def __post_init__(self):
        if self.frame not in ("world", "body"):
            raise ValueError("Velocity frame must be 'world' or 'body'.")
        if not all(isfinite(v) for v in (self.vx, self.vy, self.omega)):
            raise ValueError("Velocity components must be finite.")
