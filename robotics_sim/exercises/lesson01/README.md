# Lesson 1: System overview

This lesson introduces the environment, robot, commands, state, and
simulation time. The scripted activities preview the system that later lessons
extend with sensing, estimation, planning, and feedback control.

## Activities

The square preview moves a body around a 2 m square using four constant
world-frame velocity commands, each held for 4 seconds. The central obstacle
is clear of the intended route. This is an **open-loop schedule**, not waypoint
feedback control; changing velocities or timing changes where the robot ends up.

Select **Preview: straight-line motion**, **Preview: circular motion**, then
**Preview: square motion** in the UI and press **Start**. The square preview
needs at least 16 seconds of activity time to complete all four sides.
Compare the path and measured state across the three previews.

Each preview has its own file, shown in bold in the UI:

| Preview | File | Commands |
| --- | --- | --- |
| Straight line | [straight.py](straight.py) | Forward along world x at 0.2 m/s |
| Circle | [circle.py](circle.py) | Body-frame forward speed 0.3 m/s and yaw rate 0.5 rad/s |
| Square | [square.py](square.py) | Four world-frame velocity segments, then stop |

Each file explicitly creates and commands its robot:

```python
from robotics_sim import Pose, Robot


def setup(world):
    robot = Robot(world, Pose(x=0, y=0))
    return robot


def update(robot, time, dt):
    robot.set_velocity(vx=0.2, frame="world")
```

`setup(world)` runs once when the activity is loaded or reset. `update` runs
before each physics step, receiving that same robot and simulation time in
seconds. Commands persist until replaced; `robot.set_velocity()` stops the robot.
The UI owns the world, simulation loop, pause, reset, and cleanup. Do not create
a separate `World`, call physics stepping, or add a `while` loop in the example.

As an optional extension, edit the selected file's initial pose or commands,
save, and restart the app. **Reset / Apply length** creates a fresh robot and
restarts simulation time; restarting the app loads source-code changes.

The simulation core registers robots by name and applies their commands before
advancing the shared world once per tick. This prepares for multi-robot
activities; the present previews, UI labels, and CSV measurements still use one
primary robot.

The initial robot is a dynamic box with ideal imposed planar velocity, zero
damping, and **gravity disabled**. PyBullet still resolves collisions, so
contact can make measured motion differ from prescribed motion. This is not a
wheeled robot or a motor model. Wheel kinematics and actuation belong to
Lesson 3.
