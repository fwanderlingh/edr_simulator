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

## Part II: first proportional controller

Start with **Reach a goal: proportional control**, whose goal is fixed at
`(1 m, 0 m, 0 rad)` ahead of the robot. Complete
[`StudentProportionalController.wheel_command`](proportional.py): calculate the
full position error `rho = math.hypot(ex, ey)` from both Cartesian components,
choose a proportional forward speed (zero within `eps_pos`),
then set `omega = 0.0` and use both complete inverse-kinematics equations:

```python
w_left = (v - L * omega / 2) / r
w_right = (v + L * omega / 2) / r
```

The method `wheel_command(self, pose, r, L)` receives the robot's wheel radius
`r` and track width `L` in metres. Keep the complete equations even when
`omega` is zero. The available controller parameters are `self.goal`,
`self.gain` and `self.eps_pos`.
The exercise intentionally asks for wheel references rather than a body command.

Choose **student** in **CONTROLLER MODEL**, save and restart after editing.
The scene, wheel-command application, common actuator limit, arrival detection
and CSV recording are supplied. The activity remains inspectable before an
unfinished method is first called with Start/Step. Use a 20-second duration and
compare the wheel speeds, position error and `DONE`/`TIMEOUT` outcome. The defaults are
`GAIN=0.4`, `EPS_POS=0.03 m` and a supplied wheel limit of 5 rad/s.
The exercise file exposes `GOAL`, `GAIN` and `EPS_POS` at the top;
both models use these parameters. Simulator plumbing is kept in
`_proportional_runtime.py`, outside the method to complete.
After your attempt, choose **reference** to compare with
[`ReferenceProportionalController`](../../sim/references/proportional_control.py).
Only the exercise file needs editing; the kinematics demonstrations remain separate.

## Part II: reach one target pose

After the straight-goal exercise, complete
[`StudentPoseController.wheel_command`](control.py) and choose **Reach a goal:
pose control**. The default target is `(1 m, 0.5 m, pi/2 rad)`.
The method receives `(pose, r, L)` and returns `(w_left, w_right)` in rad/s,
just like `proportional.py`. Read the single target from `self.goal`.

Draw the TURN, DRIVE and ALIGN transitions from the notes first. Check distance
before computing the bearing. Within position tolerance, enter ALIGN and correct
the wrapped final-heading error; when both tolerances are met, set
`self.phase = "DONE"` and return two zeros. Outside position tolerance, leave
ALIGN for TURN, compute the bearing error with
`wrap(math.atan2(ey, ex) - pose.yaw)`, and apply TURN/DRIVE hysteresis.
Choose `(v, omega)` using the updated phase, then calculate both complete
inverse-kinematics equations. No waypoint index is needed in this exercise.

The editable settings at the top of `control.py` are `GOAL`, `GAIN`,
`GAIN_BEARING`, `GAIN_HEADING`, `EPS_POS`, `EPS_ANGLE`, `ANGLE_IN`, `ANGLE_OUT`
and `WHEEL_MAX` (rad/s). Both models use these settings. Validation, wheel limits,
CSV recording and the standalone runner are supplied in `_control_runtime.py`.
The corresponding solution is
[`ReferencePoseController`](../../sim/references/control.py).
Use **Reach a goal: pose behind robot** to test the same pose controller's
initial turn. Single-goal activities also accept a target through the GUI's
**TARGET** fields; Heading is entered in degrees and converted to radians.

## Part II: sequence the waypoints

Next, complete [`StudentWaypointController.wheel_command`](waypoint.py) and
choose **Follow two waypoints**. `GOALS` is editable at the top of `waypoint.py`;
the gains, tolerances and wheel limit are reused from `control.py`.

The supplied adapter prepares `self.controllers`, one independent
`StudentPoseController` per goal. Select `self.controllers[self.index]`, call
its `wheel_command(pose, r, L)`, and copy its phase into `self.phase`.
Advance `self.index` only when that pose controller reports `done`, so both
position and heading must be reached at every waypoint. Return two zeros on
the arrival sample and choose TURN for the next target, or DONE after the last.
Otherwise return the current controller's wheel references. This exercise
adds sequencing to the pose controller you already completed.
The corresponding solution is
[`ReferenceWaypointController`](../../sim/references/waypoint_control.py),
which reuses the reference pose controller.

For both exercises, choose **student** in **CONTROLLER MODEL**, save and restart
after editing. An unfinished method reports its own exercise file while leaving
the scene inspectable. After your attempt, compare with **reference**.
Start with a maximum duration of **30 seconds**, press **Reset / Apply length**,
then **Start**. Goal markers show positions and desired final headings.
**Export CSV** records phases, errors, wheel speeds and completion evidence.

Verify position and wrapped-angle tolerances at each arrival, wheel limits
throughout the run and zero final wheel speeds. `goal_index` identifies the
goal active before the step; `completed_waypoints` counts arrivals after it.
DONE means all targets were reached; TIMEOUT stops without claiming completion.
To test the common speed limit, change `WHEEL_MAX` in `control.py` and restart.
Both exercises use ground-truth poses; Lesson 4 introduces sensor-based estimates.

Standalone reference comparisons, from `simulator/`:

```bash
python -m robotics_sim.exercises.lesson03.control --model reference
python -m robotics_sim.exercises.lesson03.waypoint --model reference
```

## Scene and robot commands

Open **[activity.py](activity.py)** to see how the selected activity creates
the robot and chooses its command source. The four kinematics demonstrations
use `update(robot, time, dt, ...)`; the goal activities attach the selected
controller to the same simulation loop.
The `task` argument selects the example chosen in the UI. Edit this file to
change a demonstration’s scene or assigned motion; edit `control.py` for the
single-pose controller and `waypoint.py` for sequencing. Save and restart the
application after changes.
Use **Reset / Apply length** to recreate the scene and **Start** to run it. The UI
owns simulation time and calls `update` before every physics step.

The formula exercises remain in their separate files listed above. Each
callback receives its robot explicitly; additional robots can be registered
with `Simulator.register_robot(name, robot, update=...)` to share the same
world and clock. The current UI and pose CSV still display the primary robot.
