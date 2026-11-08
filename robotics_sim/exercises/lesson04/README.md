# Lesson 4: Sensors and obstacle control

The simulator supplies the robot pose throughout this lesson, just as in
Lesson 3. Rangefinder and LiDAR measurements describe the environment and
supervise the same goal controller. Work through the following activities in
**Lesson 4 - Sensors and obstacle control**.

## Part I: stop with a front rangefinder

Choose **Reach a goal: front rangefinder stop** and complete
`StudentRangeStop.command` in [rangefinder.py](rangefinder.py). The goal is
`(3, 0) m`; an isolated box at `(1.5, 0) m` blocks the straight path.

The method receives `pose`, `goal`, `distance`, `hit`, `max_range`, and `age`,
and returns `(v, omega)` in m/s and rad/s. Validate finite distances, range and
age first. Invalid or stale data require zero motion. A valid miss observes
to `max_range`; it does not identify an obstacle at that distance. Use the
front distance to interrupt nominal motion and retain STOP until the release
threshold is met. Inside the goal tolerance, set DONE and return zero.

The editable settings are `ENTER_DISTANCE = 0.65 m`, `EXIT_DISTANCE = 0.8 m`
and `MAX_SCAN_AGE = 0.25 s`. Distances are measured from the centre of the robot,
so the stopping margin must include its footprint and motion between samples.
One ray supports this aligned scene; it does not observe lateral clearance.

For authorized motion, call `self.goal_command(pose, goal)`. This reuses the
completed `StudentPoseController` from Lesson 3 through a supplied wheel/body
adapter. The same instance, gains, position tolerance and `WHEEL_MAX` are used.
Call `self.reset_goal_controller()` when restarting after an interruption.
The runtime owns sampling, command conversion and common wheel saturation;
the student method owns the decision and `self.phase`.

Save, click **Reload exercise**, select **student** under **STOP CONTROLLER
MODEL**, and start with **Sensor mode: ideal**. The Lesson 3 student controller
must already be complete. Predict the stopping point, then inspect the front
distance, scan age, phase and wheel commands with **Pause** and **Step**.
The robot should stop before the box and remain there. The experiment ends
with **COMPLETED**, which records the end of this observation rather than
arrival at the goal. Repeat with noisy distances and compare phase changes.

## Part II: observe directions, then choose a detour

Choose **Scan obstacles with 2D lidar**. The robot rotates with 37 rays spanning
-90 to +90 degrees at 5-degree intervals and a maximum range of 4 m. Green rays
hit geometry; gray rays are valid misses. Complete `StudentLidar.directions`
in [lidar.py](lidar.py), using the robot yaw to rotate each sensor bearing into
a world-space unit vector. The public exercise method is initially unfinished.
Select **Sensor mode: student** to test those directions with ideal ranges;
compare the centre ray, endpoints and scan orientation with the reference.
The sampler performs ray casting; exercise code needs no PyBullet calls.

Next choose **Reach a goal and avoid an obstacle**. Set `AVOIDANCE_STAGE`
at the top of [avoidance.py](avoidance.py), saving and reloading at each step:

1. `sectors`: implement `StudentSectors.summarize`. Compute minima for front,
   left and right; front includes bearings within 30 degrees of the forward
   axis. Return `None` for invalid data or missing coverage. A valid miss
   observes to maximum range. The robot rotates for inspection; neither the
   supervisor nor the goal controller is called. Select **SECTOR MODEL**.
2. `stop`: reuse the completed `StudentRangeStop.command` on the front-sector
   minimum. Only the input observation changes from Part I. The sector summary
   and Lesson 3 controller must be complete; the detour method may remain
   unfinished. This blocked experiment also ends with **COMPLETED**.
3. `avoid` (default): implement `StudentObstacleSupervisor.command` to add a
   detour around the isolated box. Use side observations to choose a turn,
   retain the turn direction and bypass progress, then return to nominal
   goal control. Set DONE only inside the position tolerance. Arrival gives
   **DONE**; expiration of the duration gives **TIMEOUT**.

Use **student** under **SUPERVISOR MODEL** for stop/avoid. Each student method
raises an explicit error until implemented. The supplied runtime provides
state and the Lesson 3 adapter, without supplying the student's decision.
The task requires position only, so arrival precedes final-heading alignment.
Reloading Lesson 4 also reloads saved Lesson 3 controller changes.

`avoidance.py` exposes `ENTER_DISTANCE`, `EXIT_DISTANCE`, `MAX_SCAN_AGE`,
`TURN_ANGLE`, `TURN_TOLERANCE`, `TURN_GAIN`, `BYPASS_DISTANCE`, and `BYPASS_SPEED`.
Both models use these settings. `LIDAR_MODEL` selects `reference` (default) or
the completed `student` directions independently of ideal/noisy **Sensor mode**.
This lets you reuse your geometry with noisy range measurements.

Compare front-only stopping with LiDAR stopping before implementing avoidance.
Then predict and check each detour phase using the sector readings and scan
age. Repeat identical conditions with noise. As an extension, edit the scene
in [gui.py](gui.py) to replace the box with a wide wall and explain the result.
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
phase, scan age, nominal-controller phase and wheel scaling. A control sample
uses the scan available before the physics step whose resulting pose is logged.

Compare the same scene and duration with **reference** after your own attempt.
Use exported data to explain where a nominal command was blocked, when a turn
was authorized and whether the goal tolerance was reached. The implementations
for comparison are in [references/sensors.py](../../sim/references/sensors.py).
