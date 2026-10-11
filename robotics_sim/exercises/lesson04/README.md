# Lesson 4: Sensors and noise

Choose **Lesson 4 - Sensors and noise**. Its three activities are:

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
acceleration. There is no odometry or filtering yet.

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
