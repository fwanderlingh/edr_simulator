"""Compatibility imports for the Lesson 4 reference modules.

Instructor solutions mirror the exercise filenames: lidar.py, rangefinder.py,
avoidance.py and estimation.py. Existing imports from this module remain valid.
"""

from .avoidance import ReferenceObstacleSupervisor, ReferenceSectors, wrap
from .estimation import ReferenceInertial, ReferenceOdometry
from .rangefinder import ReferenceRangeStop

__all__ = [
    "ReferenceInertial", "ReferenceObstacleSupervisor", "ReferenceOdometry",
    "ReferenceRangeStop", "ReferenceSectors", "wrap",
]
