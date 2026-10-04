"""Write your own open-loop velocity schedule here."""

from ...types import Velocity


class SquareProgram:
    """TODO: travel a 2 m square in 16 s, then stop. No feedback control."""

    def update(self, time, dt):
        # TODO: return a Velocity for each time segment. Stopped for now.
        return Velocity()
