"""Ready-to-run overview previews; customize their velocity schedules here."""

from ...types import Velocity


class ConstantProgram:
    def __init__(self, command):
        self.command = command

    def update(self, time, dt):
        return self.command


class StraightProgram(ConstantProgram):
    """Move along the world x axis at 0.2 m/s."""

    def __init__(self):
        super().__init__(Velocity(vx=0.2))


class CircleProgram(ConstantProgram):
    """Move forward at 0.3 m/s while turning at 0.5 rad/s."""

    def __init__(self):
        super().__init__(Velocity(vx=0.3, omega=0.5, frame="body"))


class SquareProgram:
    """Travel a 2 m square in 16 s; stop afterwards. No feedback control."""

    def update(self, time, dt):
        segment = int(time / 4)
        commands = (Velocity(vx=0.5), Velocity(vy=0.5), Velocity(vx=-0.5), Velocity(vy=-0.5))
        return commands[segment] if 0 <= segment < len(commands) else Velocity()
