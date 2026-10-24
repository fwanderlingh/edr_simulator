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
**KINEMATICS MODEL** selector. Restart the application after editing. The
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
motor dynamics. These four GUI tasks do not use a feedback controller or
odometry estimator.

## Part II: guided waypoint controller

First draw the TURN, DRIVE and ALIGN transitions and predict the initial
command from the Lesson 3 notes. Then complete
[`StudentWaypointController.nominal_command`](control.py). The supplied loop
already converts the body command into wheel speeds, applies a common speed
limit and records a CSV. Keep the geometric calculation separate from those
supplied mechanisms.

Open **Lesson 3 - Mobile base kinematics** in the desktop app and choose:

- **Reach a goal: pose control** for one target pose;
- **Follow two waypoints** for two targets in sequence;
- **Reach a goal behind the robot** to test the initial turn.

The selector becomes **CONTROLLER MODEL** for these activities. Choose
**student** to run your `nominal_command`; an unfinished method reports the
file to edit while leaving the scene available for inspection. After your
attempt, compare with **reference**, whose solution is kept separately in
`control_reference.py`. The wheel conversion uses the supplied reference
kinematics so this exercise isolates the controller.

Set an initial maximum duration of **30 seconds**, press **Reset / Apply length**,
then **Start**. Goal markers show positions; arrows show desired final headings.
The live measurements show the current phase, completed-waypoint count and
errors. **Export CSV** records the same evidence for analysis. Reaching all
poses ends the activity early; the time limit produces a timeout.

Verify position and wrapped-angle tolerances at **each** completed waypoint,
wheel limits throughout the run and zero wheel speeds at the end. `goal_index`
identifies the goal active before the recorded step; `completed_waypoints`
counts goals completed after that step. `DONE` and `TIMEOUT` are different
outcomes. Both stop the robot. This controller uses ground-truth poses; the
estimation exercise in Lesson 4 compares them with encoder and IMU estimates.

## Scene and robot commands

Open **[activity.py](activity.py)** to see how the selected activity creates
the robot and chooses its command source. The four kinematics demonstrations
use `update(robot, time, dt, ...)`; the goal activities attach the waypoint
controller to the same simulation loop.
The `task` argument selects the example chosen in the UI. Edit this file to
change a demonstration’s scene or assigned motion; edit `control.py` to
implement the goal controller. Save and restart the application after changes.
Use **Reset / Apply length** to recreate the scene and **Start** to run it. The UI
owns simulation time and calls `update` before every physics step.

The formula exercises remain in their separate files listed above. Each
callback receives its robot explicitly; additional robots can be registered
with `Simulator.register_robot(name, robot, update=...)` to share the same
world and clock. The current UI and pose CSV still display the primary robot.
