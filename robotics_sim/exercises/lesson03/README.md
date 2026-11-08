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
`self.gain_pos`, `self.eps_pos` and `self.wheel_max`.
`GAIN_POS` (available as `self.gain_pos`) is the position-error gain `k_rho`
from the notes, shared in name with the following pose-control exercise.
The exercise intentionally asks for wheel references rather than a body command.

Before returning, apply the common wheel-speed saturation from the notes. Compute
the largest absolute wheel speed and use the same factor on both wheels:
`min(1.0, self.wheel_max / peak)`, or `1.0` when `peak` is zero. Record the factor
in `self.scale` for the `wheel_scale` CSV column and return the scaled speeds.
The simulator applies your returned speeds unchanged. The check **Wheel speeds
stay within ...** fails when you exceed the limit; it does not correct your command.

Choose **student** in **CONTROLLER MODEL**, save and click **Reload exercise** after editing.
The scene, wheel-command application, arrival detection
and CSV recording are supplied. The activity remains inspectable before an
unfinished method is first called with Start/Step. Use a 20-second duration and
compare the wheel speeds, position error and `DONE`/`TIMEOUT` outcome. The defaults are
`GAIN_POS=0.4`, `EPS_POS=0.03 m` and `WHEEL_MAX=5 rad/s`.
The exercise file exposes `GOAL`, `GAIN_POS`, `EPS_POS` and `WHEEL_MAX` at the top;
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

For this pose controller, `self.done` is equivalent to `self.phase == "DONE"`.
It is an inherited, read-only Python `@property`: reading `self.done` (without
parentheses) computes the result each time. Set `self.phase = "DONE"` when
the pose is reached; do not assign to `self.done` or maintain a separate flag.

Draw the TURN, DRIVE and ALIGN transitions from the notes first. Check distance
before computing the bearing. Within position tolerance, enter ALIGN and correct
the wrapped final-heading error; when both tolerances are met, set
`self.phase = "DONE"` and return two zeros. Outside position tolerance, leave
ALIGN for TURN, compute the bearing error with
`wrap(math.atan2(ey, ex) - pose.yaw)`, and apply TURN/DRIVE hysteresis.
Choose `(v, omega)` using the updated phase, then calculate both complete
inverse-kinematics equations, then apply the common saturation as in `proportional.py`.
Record its factor in `self.scale`; use `1.0` when returning two zeros.
No waypoint index is needed in this exercise.

The editable settings at the top of `control.py` are `GOAL`, `GAIN_POS`,
`GAIN_BEARING`, `GAIN_HEADING`, `EPS_POS`, `EPS_ANGLE`, `ANGLE_IN`, `ANGLE_OUT`
and `WHEEL_MAX` (rad/s). Both models use these settings. Saturation belongs in
your `wheel_command`; `_control_runtime.py` validates and applies the returned
values without limiting them. The UI session owns the simulation loop and CSV recording.
The corresponding solution is
[`ReferencePoseController`](../../sim/references/control.py).
Use **Reach a goal: pose behind robot** to test the same pose controller's
initial turn. Single-goal activities also accept a target through the GUI's
**TARGET** fields; Heading is entered in degrees and converted to radians.

## Part II: sequence the waypoints

Next, complete [`StudentWaypointController.wheel_command`](waypoint.py) and
choose **Follow two waypoints**. `GOALS` is editable at the top of `waypoint.py`;
the gains, tolerances and wheel limit are reused from `control.py`.

For the waypoint controller, the inherited `self.done` property instead checks
`self.index == len(self.goals)`: the whole sequence is complete. By contrast,
`self.pose_controller.done` checks the pose controller's phase and means
only that pose is complete. Advance `self.index` after each arrival; do not
assign to `self.done`. The sequence's `self.phase` displays the current phase
and does not determine this completion check.

The supplied adapter creates one `StudentPoseController` in `self.pose_controller`
with `self.goals[0]` as its initial target. Call its `wheel_command(pose, r, L)`
and copy its phase into `self.phase` and its scale into `self.scale` for the CSV.
Return its already saturated wheel speeds without applying another saturation.
Reuse the same object throughout the sequence.
Advance `self.index` only when that pose controller reports `done`, so both
position and heading must be reached at every waypoint. If the sequence is complete,
set `self.phase = "DONE"`. Otherwise call
`self.pose_controller.set_goal(self.goals[self.index])` and set `self.phase = "TURN"`.
The supplied `set_goal(next_goal)` method changes the target and resets the pose
controller to TURN. Call it only when advancing to the next waypoint: calling it
on every step would erase the controller's phase. Return two zeros on the arrival
sample, with `self.scale = 1.0`; the next target starts on the following sample.
Otherwise return the pose
controller's wheel references. This exercise adds sequencing to the pose controller
you already completed.
The corresponding solution is
[`ReferenceWaypointController`](../../sim/references/waypoint_control.py),
which reuses the reference pose controller.

For both exercises, choose **student** in **CONTROLLER MODEL**, save and click **Reload exercise**
after editing. An unfinished method reports its own exercise file while leaving
the scene inspectable. After your attempt, compare with **reference**.
Start with a maximum duration of **30 seconds**, press **Reset / Apply length**,
then **Start**. Goal markers show positions and desired final headings.
**Export CSV** records phases, errors, wheel speeds and completion evidence.

Verify position and wrapped-angle tolerances at each arrival, wheel limits
throughout the run and zero final wheel speeds. `goal_index` identifies the
goal active before the step; `completed_waypoints` counts arrivals after it.
DONE means all targets were reached; TIMEOUT stops without claiming completion.
To test the common speed limit, change `WHEEL_MAX` in `control.py` and click **Reload exercise**.
For the proportional exercise, change `WHEEL_MAX` in `proportional.py` instead.
Both exercises use simulator poses. Lesson 4 keeps that pose input and adds
rangefinder/LiDAR supervision for obstacles.
Lesson 4 also reuses this `StudentPoseController` for nominal motion toward its
position goal, with the same gains, position tolerance and wheel limit. Its
supplied adapter preserves this interface while scan supervision is added.

## Scene and robot commands

Open **[activity.py](activity.py)** to see how the selected activity creates
the robot and chooses its command source. The four kinematics demonstrations
use `update(robot, time, dt, ...)`; the goal activities attach the selected
controller to the same simulation loop.
The `task` argument selects the example chosen in the UI. Edit this file to
change a demonstration’s scene or assigned motion; edit `control.py` for the
single-pose controller and `waypoint.py` for sequencing. Save and click **Reload exercise**
after changes.
Use **Reset / Apply length** to recreate the scene and **Start** to run it. The UI
owns simulation time and calls `update` before every physics step.

The formula exercises remain in their separate files listed above. Each
callback receives its robot explicitly; additional robots can be registered
with `Simulator.register_robot(name, robot, update=...)` to share the same
world and clock. The current UI and pose CSV still display the primary robot.
