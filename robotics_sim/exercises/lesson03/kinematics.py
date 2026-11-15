"""Implement the two formulas below.

Conventions: metres, seconds, radians; left is +y; positive yaw turns left.
Select Student in the application to run this class, then click Reload exercise after saving.
"""

from ...robots.differential_drive import ReferenceDifferentialDrive


class StudentDifferentialDrive(ReferenceDifferentialDrive):
    def forward(self, w_left, w_right):
        """Use self.wheel_radius and self.track_width to return (v, omega)."""
        # TODO: return (v, omega) from the wheel speeds.
        raise NotImplementedError("complete forward() in exercises/lesson03/kinematics.py")

    def inverse(self, v, omega):
        """Use self.wheel_radius and self.track_width to return the wheel speeds."""
        # TODO: return (w_left, w_right) from (v, omega).
        raise NotImplementedError("complete inverse() in exercises/lesson03/kinematics.py")
