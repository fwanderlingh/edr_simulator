# Lesson 4: Sensors and noise

Choose **Lesson 4 - Sensors and noise**. Start with these three measurement activities:

- **Wheel encoders and forward range:** drive toward a wall and compare encoder
  increments with wheel speed times the sampling interval. The front rangefinder
  initially reads 2.9 m. With the default 16-second activity the robot stops
  before reaching the wall; longer runs let you observe a blocked chassis.
- **Scan obstacles with 2D lidar:** rotate in place with 37 rays spanning -90°
  to +90°, in 5° increments. Green rays hit geometry; gray rays are misses
  within the 4 m maximum range. The centre ray is the forward range.
- **IMU during changing motion:** observe yaw rate and forward/lateral
  acceleration while the robot follows a curve with smoothly changing forward
  speed.

The **Sensor mode** selector offers `ideal`, `noisy`, and `student`. Selecting
a mode resets the activity and is remembered in `settings.json`. Student mode
uses ideal encoders/IMU and [StudentLidar](lidar.py). Replace its working
ray-direction formula to implement the rotation from sensor bearings to
world-space unit vectors, then restart the application. The inherited sampler
calls the world's ray-casting interface and converts hit fractions to
distances. Sensor and exercise code do not need PyBullet API calls.

Sensor readings are cached: reading, rendering, or pausing does not consume
noise or generate new measurements. Sensors sample every `ceil(0.1 / dt)`
physics steps (10 Hz at the default timestep), plus initial and final samples.
For a timestep that does not divide 0.1 seconds, use exported timestamps as
the actual interval. The displayed scan stays at its acquisition pose until
the next sample.

| Sensor | Measurement | Noisy-mode model |
| --- | --- | --- |
| Encoder | Left/right shaft-angle increments, rad | 360 ticks/turn; Gaussian angle noise of 1/4 tick before cumulative-angle quantization |
| Rangefinder/lidar | Range, m, plus explicit hit flag | Gaussian 0.02 m standard deviation on hits; clipped to 0–4 m; misses remain 4 m |
| IMU | Yaw rate, rad/s; body x/y acceleration, m/s² | Gyro bias 0.01 rad/s, noise 0.005 rad/s; acceleration biases +0.02/-0.02 m/s² and noise 0.03 m/s² |

Each sensor uses its own fixed random seed, so resetting repeats the same noise
sequence. The models and constants are in
[sensors/](sensors/). Encoder angles integrate ideal shaft commands; they are
not measured wheel joints, and still increase if the chassis is blocked. The
IMU reads physics velocity and computes finite-difference world acceleration
before expressing it in the current body frame. It excludes gravity, roll/pitch
sensing, and a full inertial sensor model. Initial acceleration is zero;
starting an ideal velocity source can produce a large first-sample
acceleration. The comparison activities below reconstruct pose from these
measurements. Sensor fusion and Kalman filtering remain later topics.

**Export CSV** writes ground-truth poses to the chosen file and sensor samples
to a companion file, e.g. `run.csv` and `run_sensors.csv`. Existing files are
overwritten. The sensor file includes timestamps, measured and ideal
encoder/IMU values, and `range_i_m`, `true_range_i_m`, `hit_i` for each ray in
bearing order. Encoder deltas cover the interval since the preceding sensor
sample. The initial ideal deltas are zero. Compare measured and ideal columns
to study quantization, bias, noise, and the distinction between sensor rate
and physics rate.

Public interfaces: `EncoderSensor.read()`, `LidarSensor.scan()`, and
`ImuSensor.read()` return the latest sampled measurements. `SensorSuite` handles
advancing shaft angles and sampling on the simulation clock.

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

`setup_sensors(robot, ...)` attaches the sensor suite to that robot. The
session samples it after each physics step according to its sampling rate.

## Part I: guided encoder/IMU comparison

Start with the questions at the end of Part I in the
[course notes](../../../../dispense/EDR.04_Sensori_e_percezione.tex).
Predict which information is needed to reconstruct position from each sensor,
then edit [estimation.py](estimation.py):

- `StudentOdometry.update`: update `x`, `y`, `yaw`, `speed`, and `yaw_rate`
  from measured left/right angle increments over the supplied interval.
- `StudentInertial.update`: update `x`, `y`, `yaw`, `vx`, and `vy` from gyro
  and body-frame accelerations. Use `truth_yaw` only when explicitly supplied;
  the standard experiment integrates the gyro instead of substituting true yaw.

Both methods are intentionally unfinished. The application identifies the
missing method instead of silently replacing your work. Initial pose,
orientation, and velocity are zero. Accelerations are already in m/s² without
gravity; do not convert them from g or remove gravity again.

Save the file and restart the simulator. In **Lesson 4 - Sensors and noise**,
set **ESTIMATOR MODEL** to **student**, then choose:

1. **Compare encoder and IMU: straight motion** to verify direction, speed
   integration and initial conditions.
2. **Compare encoder and IMU: curved motion** to compare trajectories while
   the body axes rotate.
3. **Compare estimates: blocked robot** to observe commanded shaft motion
   while the chassis stops at a wall.

Press **Start**. **Pause** and **Step** let you inspect the same simulation
instant; estimates change only at sensor acquisition times. **Sensor mode**
selects **ideal** or **noisy** measurements independently of the estimator
model. Changing the activity or model resets the experiment. Repeat identical
conditions with noise, and use **Export CSV**: the main file contains the true
pose plus encoder/IMU estimates, velocities and errors; the companion
`_sensors.csv` contains sensor readings. Align sensor timestamps when comparing
files. The comparison does not correct the estimates
with ground truth during the run.

Plot errors rather than expecting every numerical integrator to match the
simulation exactly. Keep encoder and IMU reconstructions separate: this exercise
prepares a discussion of sensor fusion without implementing it yet.

## Part II: guided reactive supervisor

Draw your state diagram and stopping conditions before editing
[avoidance.py](avoidance.py). Complete:

- `StudentSectors.summarize`: minima for `front`, `left`, and `right`, or
  `None` for invalid/missing coverage. A valid miss observes to maximum range;
  it is not an occupied point at that range.
- `StudentObstacleSupervisor.command`: return `(v, omega)` and update `phase`.
  Old or invalid data must stop the robot. Keep enough state to avoid turning
  immediately back toward the same box. Report completion as `DONE`.

Save and restart the simulator, choose **Reach a goal and avoid an obstacle**,
set **SUPERVISOR MODEL** to **student** and **Sensor mode** to **ideal**, then
press **Start**. The target is `(3, 0) m`, with an isolated box centred at
`(1.5, 0) m`. Predict the initial path and stopping condition before running it.

The activity supplies sensor scheduling, conversion to wheel speeds and a
common 5 rad/s wheel-speed limit. Ground-truth pose isolates perception and
action selection. Inspect the phase and target error in the measurements panel;
use **Pause**, **Step** and **Export CSV** to connect decisions to sector
readings and scan age. Repeat with **noisy** data and compare arrival time and
phase changes. `DONE` and `TIMEOUT` are different outcomes; both stop the robot.

The scenes for the estimation and avoidance activities are in
[gui.py](gui.py). As an extension, replace the isolated box with a wide wall,
save, restart and explain the observed failure. This local exercise is not a
general path planner.

## Instructor comparison after attempting the exercises

The complete implementations are separate in [reference.py](reference.py),
corresponding to the solutions appendix of the notes. After your own attempt,
keep the activity and sensor mode unchanged, select **reference** under
**ESTIMATOR MODEL** or **SUPERVISOR MODEL**, then restart the activity and
compare exported measurements.

The reference detour is designed for the supplied isolated-box scene; it does
not guarantee success in every geometry. All of these practical activities are
available from the application's activity selector.
