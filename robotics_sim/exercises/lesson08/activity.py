"""Escape the Lab: an inspectable set built entirely from solid-colour boxes.

Inspired by docs/the_escape_room_plan.md and its Area 86 reference image.
Everything is static except the existing, stationary mobile-base plant. The
folded arm is a display model, not an articulated or controllable manipulator.
There are no triggers, door logic, grasping, waypoints or autonomous actions.
"""

import math

from robotics_sim import DifferentialDriveRobot, Pose


# Inner room faces; the north wall has exactly one opening.
HALF_WIDTH = 4.4
HALF_DEPTH = 3.5
WALL_THICKNESS = 0.18
WALL_HEIGHT = 0.72
EXIT_X = (2.35, 3.85)
START = (-3.2, -2.25)
CAMERA = {"target": (0.0, 0.20, 0.15), "distance": 14.0,
          "yaw": 32.0, "pitch": -58.0}

WALL = (0.48, 0.57, 0.64, 1)
DARK = (0.16, 0.23, 0.29, 1)
LIGHT = (0.77, 0.83, 0.85, 1)
TEAL = (0.05, 0.52, 0.55, 1)
AMBER = (0.94, 0.60, 0.16, 1)
CRATE = (0.67, 0.43, 0.22, 1)
GREEN = (0.22, 0.68, 0.43, 1)
RED = (0.85, 0.22, 0.20, 1)


def setup(world):
    """Build an enclosed teaching set; returning a robot keeps the shared UI."""
    world.exact_occlusion = True
    groups = {}

    def box(group, center, size, color, yaw=0, pitch=0):
        body = world.add_box(Pose(*center, yaw=yaw, pitch=pitch), size=size, color=color)
        groups.setdefault(group, []).append(body)
        return body

    def beam(group, start, end, width, color):
        delta = tuple(b - a for a, b in zip(start, end))
        length = math.sqrt(sum(v * v for v in delta))
        box(group, tuple((a + b) / 2 for a, b in zip(start, end)),
            (length, width, width), color,
            yaw=math.atan2(delta[1], delta[0]),
            pitch=-math.atan2(delta[2], math.hypot(*delta[:2])))

    def wall(center, size):
        box("perimeter", center, size, WALL)
        box("perimeter", (center[0], center[1], WALL_HEIGHT + .025),
            (size[0], size[1], .05), DARK)

    # Walls overlap at corners. No scenery stands in for a collision boundary.
    t, w, h = WALL_THICKNESS, HALF_WIDTH, HALF_DEPTH
    wall((-w - t / 2, 0, WALL_HEIGHT / 2), (t, 2 * h + 2 * t, WALL_HEIGHT))
    wall((w + t / 2, 0, WALL_HEIGHT / 2), (t, 2 * h + 2 * t, WALL_HEIGHT))
    wall((0, -h - t / 2, WALL_HEIGHT / 2), (2 * w, t, WALL_HEIGHT))
    for low, high in ((-w, EXIT_X[0]), (EXIT_X[1], w)):
        wall(((low + high) / 2, h + t / 2, WALL_HEIGHT / 2),
             (high - low, t, WALL_HEIGHT))
    for x in (-w - t / 2, w + t / 2):
        for y in (-h - t / 2, h + t / 2):
            box("perimeter", (x, y, .49), (.24, .24, .98), DARK)

    # Open portal: its door leaf is parked overhead, never across the opening.
    exit_center = sum(EXIT_X) / 2
    for x in (EXIT_X[0] - .07, EXIT_X[1] + .07):
        box("exit", (x, h + t / 2, .8), (.14, .25, 1.6), DARK)
        box("exit", (x, h - .045, .8), (.055, .025, 1.45), GREEN)
    box("exit", (exit_center, h + t / 2, 1.68), (1.78, .28, .18), DARK)
    box("exit", (exit_center, h + t / 2, 1.45), (1.5, .13, .25), LIGHT)
    box("exit", (exit_center, h - .065, 1.69), (.6, .025, .10), GREEN)
    box("exit", (exit_center, h + .64, .004), (1.48, 1.12, .008), GREEN)
    for y in (h + .3, h + .6, h + .9):
        box("exit", (exit_center, y, .010), (.72, .045, .004), LIGHT)

    # Raised control corner with steps and simple guard rails, like the reference.
    box("control_deck", (-3.0, 2.35, .12), (2.35, 1.35, .24), DARK)
    for y, rise in ((1.18, .08), (1.40, .16)):
        box("control_deck", (-3.0, y, rise / 2), (.85, .22, rise), LIGHT)
    for x in (-4.08, -1.92):
        box("control_deck", (x, 2.93, .65), (.07, .07, .82), AMBER)
    box("control_deck", (-3, 2.93, 1.05), (2.23, .07, .07), AMBER)
    box("control_deck", (-4.08, 1.77, .65), (.07, .07, .82), AMBER)
    box("control_deck", (-4.08, 2.35, 1.05), (.07, 1.23, .07), AMBER)
    box("control_deck", (-3.0, 2.5, .48), (1.25, .56, .48), WALL)
    box("control_deck", (-3.0, 2.5, .745), (1.35, .65, .05), LIGHT)
    box("control_deck", (-3.0, 2.63, .91), (.62, .10, .38), DARK)
    box("control_deck", (-3.0, 2.568, .92), (.5, .025, .26), TEAL)
    box("control_deck", (-2.95, 2.35, .79), (.5, .19, .035), DARK)

    # Central equipment bench, leaving approach space on every side.
    for x in (-1.28, .08):
        for y in (-.08, .58):
            box("workbench", (x, y, .33), (.10, .10, .66), DARK)
    box("workbench", (-.6, .25, .71), (1.65, 1.02, .10), LIGHT)
    box("workbench", (-.88, .35, .96), (.55, .48, .40), WALL)
    box("workbench", (-.88, .095, .99), (.38, .03, .22), TEAL)
    box("workbench", (-.12, .35, .83), (.34, .34, .14), AMBER)

    # Freestanding barriers: feet, panel and high-visibility top edge.
    for x, y, angle in ((-2.45, -.48, -.18), (1.58, -.65, .25)):
        box("barriers", (x, y, .48), (.10, 1.35, .80), WALL, angle)
        box("barriers", (x, y, .90), (.13, 1.4, .055), AMBER, angle)
        for dy in (-.48, .48):
            box("barriers", (x - math.sin(angle) * dy, y + math.cos(angle) * dy, .055),
                (.55, .16, .11), DARK, angle)

    def crate(x, y, z, side):
        box("crates", (x, y, z + side / 2), (side, side, side), CRATE)
        # Two battens suggest a crate without textures or a dense mesh.
        for dx in (-side * .32, side * .32):
            box("crates", (x + dx, y - side / 2 - .015, z + side / 2),
                (.055, .03, side), AMBER)

    crate(1.08, 2.12, 0, .72)  # In front of the switch, not the exit.
    crate(2.85, .62, 0, .5)
    crate(2.85, .62, .5, .36)
    crate(-3.65, .05, 0, .43)
    box("switch", (1.08, 2.91, .38), (.30, .30, .76), DARK)
    box("switch", (1.08, 2.91, .85), (.46, .28, .28), LIGHT)
    box("switch", (1.08, 2.752, .85), (.18, .035, .18), RED)

    # A side objective from the guide: a recognizable first-aid kit on a shelf.
    for x in (2.85, 3.55):
        box("medical_station", (x, -2.35, .24), (.10, .46, .48), DARK)
    box("medical_station", (3.2, -2.35, .5), (1.0, .6, .07), LIGHT)
    box("medical_station", (3.2, -2.35, .71), (.50, .28, .35), LIGHT)
    box("medical_station", (3.2, -2.503, .71), (.08, .025, .23), RED)
    box("medical_station", (3.2, -2.505, .71), (.24, .03, .075), RED)
    box("medical_station", (3.2, -2.35, .92), (.21, .08, .07), DARK)
    for x, y in ((-.35, -2.5), (2.5, -1.65)):
        box("markers", (x, y, .035), (.34, .34, .07), DARK)
        box("markers", (x, y, .13), (.23, .23, .12), AMBER)
        box("markers", (x, y, .23), (.12, .12, .08), LIGHT)
        box("markers", (x, y, .30), (.075, .075, .06), AMBER)

    robot = DifferentialDriveRobot(world, Pose(x=START[0], y=START[1], z=.17))
    # Display-only folded arm: a pedestal, two links and a two-finger gripper.
    sx, sy = START
    box("display_arm", (sx, sy, .41), (.19, .20, .18), DARK)
    shoulder, elbow, wrist = (sx, sy, .5), (sx + .24, sy, .79), (sx - .19, sy, .88)
    beam("display_arm", shoulder, elbow, .10, TEAL)
    beam("display_arm", elbow, wrist, .085, TEAL)
    box("display_arm", elbow, (.15, .15, .15), DARK)
    box("display_arm", wrist, (.12, .17, .10), DARK)
    for dy in (-.07, .07):
        box("display_arm", (wrist[0] - .09, sy + dy, wrist[2]), (.18, .035, .055), LIGHT)

    world.preview_groups = {name: tuple(bodies) for name, bodies in groups.items()}
    world.preview_camera = dict(CAMERA)
    world.preview_labels = (
        ((sx, sy, 1.0), "Robot / folded arm"),
        ((-3.0, 2.63, 1.16), "Control deck"),
        ((1.08, 2.752, 1.02), "Switch"),
        ((1.08, 2.12, .77), "Blocking crate"),
        ((3.2, -2.35, 1.0), "Medical kit"),
        ((exit_center, h + .64, .05), "EXIT"),
    )
    return robot


def update(robot, time, dt):
    """Preview only: advancing the clock never executes a mission."""
    robot.set_velocity()
