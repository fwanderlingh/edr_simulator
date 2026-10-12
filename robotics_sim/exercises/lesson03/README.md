# Lesson 3: Mobile base kinematics

Four tasks demonstrate equal wheel speeds (straight line), unequal speeds
(circle), opposite speeds (rotation in place), and inverse kinematics from a
body twist. The orange trail shows the measured path. The readout and CSV
include left/right wheel speeds, forward speed, and yaw rate, with explicit
units.

The robot uses wheel radius `r = 0.12 m` and track width `L = 0.42 m`.
Positive wheel speed moves forward; positive yaw turns left:

```text
v = r * (w_right + w_left) / 2
omega = r * (w_right - w_left) / L
w_left  = (v - omega * L / 2) / r
w_right = (v + omega * L / 2) / r
```

Edit [StudentDifferentialDrive](kinematics.py) and select **student** in the
Lesson 3 kinematics selector. Restart the application after editing. The
**reference** model is supplied so you can compare your formulas with it.
Both implement `forward(w_left, w_right)` and `inverse(v, omega)`; geometry
belongs to the model. The robot API exposes
`set_wheel_velocities(w_left, w_right)` and `set_twist(v, omega)`. To replace
the model in Python, assign `robot.motor_model` before issuing commands; keep
its geometry unchanged after creating the robot.

Predict straight-line distance `v*t`, circle radius `v/omega`, and spin angle
`omega*t`, then compare with exported ground-truth poses. Small circular-path
errors decrease with the fixed timestep. This is ideal no-slip planar
kinematics: PyBullet integrates the commanded chassis velocity; the cylindrical
wheels are visual geometry, without wheel joints, rolling contact, slip, or
motor dynamics. No feedback controller or odometry estimator is involved.

## Scene and robot commands

Open **[activity.py](activity.py)** to see `setup(world, ...)` explicitly
create the robot and `update(robot, time, dt, ...)` issue commands to it.
The `task` argument selects the example chosen in the UI. Edit this file to
change the initial scene or motion, then save and restart the application.
Use **Reset / Apply length** to recreate the scene and **Start** to run it. The UI
owns simulation time and calls `update` before every physics step.

The formula exercises remain in their separate files listed above. Each
callback receives its robot explicitly; additional robots can be registered
with `Simulator.register_robot(name, robot, update=...)` to share the same
world and clock. The current UI and pose CSV still display the primary robot.
