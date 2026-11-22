# Lesson 4: Sensors and obstacle control

The simulator supplies the robot pose throughout this lesson, just as in
Lesson 3. Rangefinder and LiDAR measurements describe the environment and
supervise the same goal controller. Work through the following activities in
**Lesson 4 - Sensors and obstacle control**.

For this lesson, assume sensor measurements are always available and valid.
Use the latest sample without checking its age. LiDAR scans cover all three
sectors used by the exercises.

## Part I: stop with a front rangefinder

Choose **Reach a goal: front rangefinder stop** and complete
`StudentRangeStop.command` in [rangefinder.py](rangefinder.py). The goal is
the pose `(3 m, 0 m, 0 rad)`; an isolated box at `(1.5, 0) m` blocks the straight path.

The method receives `pose`, `goal`, `distance`, `hit`, and `max_range`,
and returns `(v, omega)` in m/s and rad/s. A miss observes
to `max_range`; it does not identify an obstacle at that distance. Use the
front distance to interrupt nominal motion and retain STOP until the release
threshold is met. The goal is always `(x, y, yaw)`, as in Lesson 3. Set DONE
and return zero only when both position and final-heading tolerances are met.

The editable settings are `D_IN = 0.65 m` and `D_OUT = 0.8 m`.
Distances are measured from the centre of the robot,
so the stopping margin must include its footprint and motion between samples.
One ray supports this aligned scene; it does not observe lateral clearance.

Configure the box at the top of `rangefinder.py`, then save and click
**Reload exercise**:

```python
OBSTACLE_POSITION = (1.5, 0.0)   # centre (x, y) in world metres
OBSTACLE_SIZE = (0.4, 0.6, 1.0)  # full lengths (x, y, z), in metres
```

The box rests on the floor. Both student and reference modes use these scene
settings. To test an off-centre obstacle, change the second position coordinate,
for example to `(1.5, 0.5)`. The controller must still decide motion from sensor
readings, rather than from these known scene coordinates.

For authorized motion, call `self.goal_command(pose, goal)`. This reuses the
completed `StudentPoseController` from Lesson 3 through a supplied wheel/body
adapter. The same instance, gains, pose tolerances and `WHEEL_MAX` are used,
including the final-heading ALIGN phase. The goal heading is passed unchanged.
Call `self.reset_goal_controller()` when restarting after an interruption.
The runtime owns sampling and command conversion, and applies wheel speeds
without rescaling. Common wheel saturation belongs to the Lesson 3 pose
controller; the wheel-limit check reports any excess instead of correcting it.
The CSV field `wheel_scale` records the factor reported by that pose controller.
The student method owns the decision and `self.phase`.

Save, click **Reload exercise**, select **student** under **STOP CONTROLLER
MODEL**, and start with **Sensor mode: ideal**. The Lesson 3 student controller
must already be complete. Predict the stopping point, then inspect the front
distance, phase and wheel commands with **Pause** and **Step**.
The robot should stop before the box and remain there. The experiment ends
with **COMPLETED**, which records the end of this observation rather than
arrival at the goal. Repeat with noisy distances and compare phase changes.

## Part II: understand the LiDAR, summarize sectors, then supervise motion

Choose **Scan obstacles with 2D lidar**. The robot rotates with 37 rays spanning
-90 to +90 degrees at 5-degree intervals and a maximum range of 4 m. Green rays
hit geometry; gray rays are misses. The LiDAR is supplied and requires no
implementation. A time-of-flight LiDAR measures the round-trip travel time of
reflected light to obtain distance. Its scan reports a bearing, range and hit/miss
for each observed direction. In this mounting the bearings are relative to the
robot: zero is forward, positive is left and negative is right.
Use **Sensor mode: ideal**, **Pause** and **Step** to predict and inspect changes
as an obstacle enters the field of view. Live measurements show individual rays
at -90, 0 and +90 degrees. A miss reports 4 m with `hit=False`; it is not an
obstacle at 4 m. The scan does not observe behind the robot or guarantee clearance
between rays or for the whole footprint.

Next choose **Reach a goal and avoid an obstacle**. Complete two exercises in
[avoidance.py](avoidance.py), using
**EXPERIMENT STAGE** in the GUI:

1. `sectors` (default): implement `StudentSectors.summarize`. Compute minima for front,
   left and right; front includes bearings within 30 degrees of the forward
   axis. Assume coherent arrays and at least one ray in each sector. A miss
   observes to maximum range. The robot rotates for inspection; neither the
   supervisor nor the goal controller is called. Select **SECTOR MODEL**.
2. `avoid`: implement `StudentObstacleSupervisor.command`. Reuse the front-stop
   decision from Part I with the front-sector minimum, then add a detour around
   the isolated box. Choose and store one intermediate pose from the side
   observations. Call the same Lesson 3 pose controller for that pose, then
   restore the final goal after intermediate position AND heading arrival. Set DONE only inside both pose tolerances. Arrival gives
   **DONE**; expiration of the duration gives **TIMEOUT**.

Use **student** under **SUPERVISOR MODEL** for avoid. Each student method
raises an explicit error until implemented. The supplied runtime provides
state and the Lesson 3 adapter, without supplying the student's decision.
Use the heading entry in the target controls to specify the final orientation,
in degrees in the GUI and radians in exercise code. Position arrival alone does
not complete the task: the shared Lesson 3 controller must finish alignment.
Reloading Lesson 4 also reloads saved Lesson 3 controller changes.

`avoidance.py` exposes `D_IN`, `D_OUT`,
`TURN_ANGLE` and `BYPASS_DISTANCE`, the bearing offset and distance of the
intermediate goal from the stop pose. Speeds, gains, arrival tolerances and wheel
limits are those already implemented in Lesson 3.
Both models use these settings. The supplied LiDAR works with either control
model and both ideal and noisy range measurements.

The supervisor uses GO, STOP, DETOUR and DONE. Its supplied state includes
`self.detour_goal` (initially `None`) and `self.advance_allowed`. Compute the
intermediate `(x, y, yaw)` once in STOP, using the stop pose, the selected side,
`self.turn_angle` and `self.bypass_distance`; do not move it with the robot.
In DETOUR use `self.goal_command(pose, self.detour_goal)`. After that call,
`self.goal_controller_done` reports arrival at the active pose, including heading.
Switching back to `self.goal_command(pose, goal)` automatically restarts the same
controller for the final target. Only final-pose arrival sets supervisor DONE.

Choose the side with the larger sector minimum (left on a tie) and enter DETOUR
without a lateral distance threshold. Allow the initial pure rotation even while the front is
obstructed. In DETOUR, initialize `advance_allowed` to False; set it False below
`self.d_in`, True at or above `self.d_out`, and retain it between thresholds.
If the pose controller proposes positive forward speed while advance is blocked,
return `(0, omega_c)`, preserving the angular command. Keep the intermediate goal while waiting. A subsequent obstruction
after returning to GO can require another intermediate pose.

Each stage displays its purpose and the components it executes. Live measurements
distinguish individual ray ranges from the three sector minima, show the current
supervisor phase and command. During DETOUR they show the intermediate pose,
its position and heading errors, forward-motion authorization, and the active
TURN/DRIVE/ALIGN phase of the Lesson 3 controller. The scene marks the intermediate
goal in purple and the final goal in orange. Use a 40-second duration for a full run.
A thin pale circle on the floor follows the robot at radius `D_IN`
(`d_in`), showing the stop threshold in both the rangefinder and LiDAR activities.
The default GUI stage is **sectors**, to verify the summary before the supervisor.

Changing a stage, model or sensor mode automatically
reloads saved code and resets the experiment. **Reload exercise** remains useful
after saving changes while keeping the same modes. Both stage and model choices
are remembered between launches. Estimation activities expose the same stage
selector for **encoder**, **imu** and **both**.

Verify the sector summary on scans with known results: all misses; a nearby
hit in one sector; and hits at the +/-30 degree boundaries, both in front.
Use the minimum so that a close return cannot be hidden by larger ranges.
Then predict and check each supervisor phase using these sector readings.
Repeat identical conditions with noise. As an extension, edit the scene
using `OBSTACLE_POSITION` and `OBSTACLE_SIZE` at the top of
[avoidance.py](avoidance.py). These settings apply to both exercises and both
controller models. To test a wide wall, try `OBSTACLE_SIZE = (0.4, 3.0, 1.0)`
and explain the result. Click **Reload exercise** after saving.
This local detour is designed for the supplied box and does not guarantee
arrival in arbitrary environments.

## Sampling, noise and exported evidence

Measurements are cached: reading, rendering and pausing do not acquire another
sample or consume random noise. Sampling occurs every `ceil(0.1 / dt)` physics
steps, plus initial and final samples. At the default timestep this is 10 Hz.
Use acquisition timestamps when `dt` does not divide 0.1 seconds exactly.
The displayed scan remains at its acquisition pose until the next sample.

Noisy hits have Gaussian range noise with standard deviation 0.02 m, clipped
to 0-4 m. Misses remain at maximum range with `hit=False`. Fixed seeds make a
reset repeat the same noise sequence. **Sensor mode** is independent of the
student/reference controller selector.

**Export CSV** writes the simulator pose and control trace to the chosen file,
and acquisitions to a companion `_sensors.csv` file. Existing files are
overwritten. Sensor columns are `time_s`, `range_i_m`, `true_range_i_m`, and
`hit_i` in bearing order. The front-stop activity has one ray and leaves
unobserved left/right sector fields empty. Control records include goal error,
supervisor and pose-controller phases, final-goal errors, active-target pose and
errors, `advance_allowed`, and wheel scaling. `target_*` always identifies the
final goal; `active_target_*` identifies the pose currently assigned to Lesson 3. A control sample
uses the scan available before the physics step whose resulting pose is logged.

Compare the same scene and duration with **reference** after your own attempt.
Use exported data to explain where a nominal command was blocked, when a turn
was authorized and whether the goal tolerance was reached. The implementations
for comparison mirror the exercise filenames:

| Exercise | Reference implementation |
| --- | --- |
| `rangefinder.py` | [references/rangefinder.py](../../sim/references/rangefinder.py): `ReferenceRangeStop.command` |
| `avoidance.py` | [references/avoidance.py](../../sim/references/avoidance.py): `ReferenceSectors.summarize`, `ReferenceObstacleSupervisor.command` |
| `estimation.py` (additional activity) | [references/estimation.py](../../sim/references/estimation.py): `ReferenceOdometry.update`, `ReferenceInertial.update` |

The simulator's **reference** mode uses these implementations. Their method
signatures match the student methods. See the
[instructor guide](../../sim/references/README.md) for implementation details.
