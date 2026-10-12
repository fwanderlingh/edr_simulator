"""Overview preview: circle motion. Edit here, then restart the UI."""

from robotics_sim import Pose, Robot


def setup(world):
    """Create the robot once when this activity is loaded or reset."""
    world.add_box(Pose(x=1, y=1, z=0.3), size=(0.6, 0.6, 0.6))
    robot = Robot(world, Pose(x=0, y=0))
    return robot


def update(robot, time, dt):
    """Command this robot before each physics step; time and dt are seconds."""
    robot.set_velocity(vx=0.3, omega=0.5, frame="body")
