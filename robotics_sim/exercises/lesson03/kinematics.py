"""Implement the two formulas below.

Conventions: metres, seconds, radians; left is +y; positive yaw turns left.
Select Student in the application to run this class, then restart after editing.
"""

from ...robots.differential_drive import ReferenceDifferentialDrive


class StudentDifferentialDrive(ReferenceDifferentialDrive):
    def forward(self, w_left, w_right):
        r, length = self.wheel_radius, self.track_width
        # TODO: return (v, omega) from the wheel speeds.
        raise NotImplementedError("complete forward() in exercises/lesson03/kinematics.py")

    def inverse(self, v, omega):
        r, length = self.wheel_radius, self.track_width
        # TODO: return (w_left, w_right) from (v, omega).
        raise NotImplementedError("complete inverse() in exercises/lesson03/kinematics.py")
