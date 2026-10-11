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

As an optional extension, modify
[SquareProgram.update](program.py) and restart the app to explore other
velocity schedules.

The initial robot is a dynamic box with ideal imposed planar velocity, zero
damping, and **gravity disabled**. PyBullet still resolves collisions, so
contact can make measured motion differ from prescribed motion. This is not a
wheeled robot or a motor model. Wheel kinematics and actuation belong to
Lesson 3.
