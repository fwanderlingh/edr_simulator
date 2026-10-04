# Robotics teaching simulator

A Python laboratory for the course, currently covering **lessons 1–4**:
simulation basics, coordinate frames, mobile-base kinematics, and sensors.
Estimation, control, planning, and manipulation are planned for later lessons.

## Contents

- [Student quick start](#student-quick-start)
- [Using the desktop app](#using-the-desktop-app)
- [Headless runs and CSV](#headless-runs-and-csv)
- [Developer setup](#developer-setup)
- [Minimal API](#minimal-api)
- [Lessons](#lessons)
  - [Lesson 1: System overview](#lesson-1-system-overview)
  - [Lesson 2: Frames and transformations](#lesson-2-frames-and-transformations)
  - [Lesson 3: Mobile base kinematics](#lesson-3-mobile-base-kinematics)
  - [Lesson 4: Sensors and noise](#lesson-4-sensors-and-noise)
- [Next increments](#next-increments)
- [Saved settings](#saved-settings-windows-and-ubuntu)
- [Developer notes: viewport](#developer-notes-viewport)

## Student quick start

Keep the complete `simulator` folder in a writable location. The first launch
downloads a private Python 3.12 runtime and precompiled dependencies; later runs
can work offline. This is self-contained after setup, **not an offline installer**.
Allow roughly 600 MB of free disk space.

**Ubuntu Linux 22.04+:** open a terminal in the folder and run:

```bash
bash start-simulator.sh
```

**Windows 10/11, 64-bit:** double-click `start-simulator.cmd`.

No separately installed Python, Conda, compiler, administrator access, or
environment activation is required. Ubuntu needs a desktop session for the GUI;
use headless mode on servers. For a class without internet, first run
`bash start-simulator.sh --setup-only` on Ubuntu or
`& './start-simulator.cmd' -SetupOnly` in PowerShell on each Windows machine.
Do not move or copy a created `.conda` runtime; distribute the source folder and
let each computer create its own environment.

For the notes' Module 1, choose **Lesson 2 - Frames and transformations** and
then **Module 1: Planar transforms**.

## Using the desktop app

Choose a lesson and task, then press **Start**. **Pause**, **Step**,
**Reset / Apply length**, and **Export CSV** control the activity. It starts
Ready and remains open when complete. Changing the task or resetting clears its
measurements, so export first if you need to keep them. The duration applies on
reset or when selecting another task. Close the window or choose **File > Exit**
to quit.

Drag to orbit the 3D view, scroll to zoom, or choose **View > Reset camera**.
**Labels...** controls label size and colors. Axes are x red, y green, z blue;
the ground grid is 1 m × 1 m.

## Headless runs and CSV

From a terminal in `simulator/`, use `--headless` to run an activity without a
display, `--csv` to export poses, or `--test` to run headless tests:

```bash
bash start-simulator.sh --lesson 2 --headless --csv output/frames.csv
bash start-simulator.sh --test
```

On Windows, use `& './start-simulator.cmd' -Lesson 2 -Headless -Csv output/frames.csv`
or `& './start-simulator.cmd' -Test`. CSV paths are relative to `simulator/`;
existing files are overwritten. Headless runs finish as fast as possible and
do not access desktop preferences. Both modes use a fixed timestep (`--dt`,
default 1/240 s); duration rounds up to a whole number of steps. Pose CSVs
contain ground-truth state after each step, with units in column names.

## Developer setup

Development targets Python 3.12 (the code requires Python 3.10 or newer). From
the repository root, create the supplied Conda environment and install the
package in editable mode:

```bash
conda env create -f simulator/environment.yml
conda activate edr-simulator
conda install pytest
python -m pip install --no-build-isolation --no-deps -e ./simulator
python -m robotics_sim --lesson 2
python -m pytest simulator/tests
```

The environment supplies a precompiled PyBullet package, avoiding a native
build. Student exercise edits take effect on the next launch; no reinstall is
needed.

The Ubuntu and x86-64 Windows launchers use checked-in package locks; other
detected platforms resolve `environment.yml`. Regenerate the locks when
dependencies change. GitHub Actions tests the launchers on Windows and Ubuntu.

The desktop GUI tests are opt-in with `ROBOTICS_SIM_GUI_TEST=1` (Ubuntu CI uses
Xvfb); the remaining tests run without a display.

## Minimal API

```python
from robotics_sim import Pose, Robot, Simulator, World

with World(gui=True, dt=1 / 240) as world:
    robot = Robot(world, Pose(x=0, y=0, yaw=0))
    sim = Simulator(world, robot)
    robot.set_velocity(vx=0.2, vy=0.0, omega=0.1, frame="world")
    for _ in range(240):
        pose = sim.step()
    print(pose)
```

Manual `sim.step()` calls do not sleep. Each `World` owns an isolated physics
connection; the context manager disconnects it even if an exercise raises an
exception. All backend calls belong to `sim/` or `robots/`. Geometry and exercise
code do not call PyBullet.

## Lessons

### Lesson 1: System overview

A first look at the environment, robot, commands, state, and simulation time.
The scripted activities preview the system that later lessons will extend with
sensing, estimation, planning, and feedback control.

The default exercise moves a body around a 2 m square using four constant
world-frame velocity commands, each held for 4 seconds. The central obstacle
is clear of the intended route. This is an **open-loop schedule**, not waypoint
feedback control; changing velocities or timing changes where the robot ends up.

Edit [SquareProgram.update](robotics_sim/exercises/lesson01/program.py), or pass
your own object with `update(time, dt) -> Velocity` to `Simulator(program=...)`.
Try a straight line, a diagonal, and a body-frame circular motion. Compare CSV
measurements with your predicted positions and investigate different timesteps.
Without a program, the last `robot.set_velocity(...)` command remains active.

The initial robot is a dynamic box with ideal imposed planar velocity, zero
damping, and **gravity disabled**. PyBullet still resolves collisions, so contact
can make measured motion differ from prescribed motion. This is not a wheeled
robot or a motor model. Wheel kinematics and actuation belong to lesson 3.

### Lesson 2: frames and transformations

Choose **Module 1: Planar transforms** for the activity that follows the notes'
Module 1. The robot and sensor frames are planar and use the same example values
as the notes' sensor-composition and navigation-goal exercises. Edit
[solve_planar_task](robotics_sim/exercises/lesson02/planar.py) to:

1. Compose `T_W_R @ T_R_S` and transform a sensor-frame point into the world.
2. Transform a direction with homogeneous coordinate `0`, so translation does
   not affect it.
3. Invert `T_W_R` to express a world-frame goal in robot coordinates.

The live readout shows your function's point, direction, and goal coordinates.
Check them against the expected values: world point `(2.8, 2.5) m`, world
direction `(0, 1)`, and robot-frame goal `(2, -2) m`. The diagrams distinguish
the transformed point, the direction, and the path to the world goal. This
planar activity is the core Module 1 exercise; the other Lesson 2 activities
are explicitly labelled as 3D extensions.

Choose **Extension: Rotating 3D frames** or **Extension: Fixed 3D transform**
to inspect the existing 3D World, Robot, and Sensor frame demonstrations. Edit
[point_in_world](robotics_sim/exercises/lesson02/frames.py) to implement the
sensor-point composition in the rotating demo. Reference geometry functions
are in `geometry/`.

Conventions:

- Metres, seconds, radians; right-handed frames; z points up.
- Column vectors: `T_A_B` maps coordinates expressed in B into A.
- Composition reads right to left: `T_W_S = T_W_R @ T_R_S`.
- A point uses homogeneous coordinate `1`; a direction uses `0`.
- The 3D extension uses fixed-axis RPY: `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`.
- Quaternions use `(x, y, z, w)` and are normalized on input.
- At RPY gimbal lock, the inverse conversion chooses roll zero.
- `Pose` is a body-centre pose; its default z is 0.16 m above the floor.
- Body-frame planar velocity uses the robot's current yaw.

`Visualizer` provides `draw_frame`, `draw_vector`, `draw_point`, `draw_path`, and
`draw_text`. Reusing a name updates the annotation. Annotations are no-ops in
headless mode. The geometry tests compare composition and rotations against
PyBullet's independent transform operations.

### Lesson 3: mobile base kinematics

Four tasks demonstrate equal wheel speeds (straight line), unequal speeds
(circle), opposite speeds (rotation in place), and inverse kinematics from a body
twist. The orange trail shows the measured path. The readout and CSV include
left/right wheel speeds, forward speed, and yaw rate, with explicit units.

The robot uses wheel radius `r = 0.12 m` and track width `L = 0.42 m`.
Positive wheel speed moves forward; positive yaw turns left:

```text
v = r * (w_right + w_left) / 2
omega = r * (w_right - w_left) / L
w_left  = (v - omega * L / 2) / r
w_right = (v + omega * L / 2) / r
```

Edit [StudentDifferentialDrive](robotics_sim/exercises/lesson03/kinematics.py)
and select **student** in the Lesson 3 kinematics selector. Restart the application
after editing. A working implementation is supplied so students can
replace one formula at a time and compare with **reference**. Both implement
`forward(w_left, w_right)` and `inverse(v, omega)`; geometry belongs to the model.
The robot API exposes `set_wheel_velocities(w_left, w_right)` and `set_twist(v, omega)`.
To replace the model in Python, assign `robot.motor_model` before issuing commands
(keep its geometry unchanged after creating the robot).

Predict straight-line distance `v*t`, circle radius `v/omega`, and spin angle
`omega*t`, then compare with exported ground-truth poses. Small circular-path
errors decrease with the fixed timestep. This is ideal no-slip planar kinematics:
PyBullet integrates the commanded chassis velocity; the cylindrical wheels are
visual geometry, without wheel joints, rolling contact, slip, or motor dynamics.
No feedback controller or odometry estimator is involved.

### Lesson 4: sensors and noise

Choose **Lesson 4 - Sensors and noise**. Its three activities are:

- **Wheel encoders and forward range:** drive toward a wall and compare encoder
  increments with wheel speed times the sampling interval. The front rangefinder
  initially reads 2.9 m. With the default 16-second activity the robot stops before
  reaching the wall; longer runs let you observe a blocked chassis.
- **Scan obstacles with 2D lidar:** rotate in place with 37 rays spanning -90° to
  +90°, in 5° increments. Green rays hit geometry; gray rays are misses within
  the 4 m maximum range. The centre ray is the forward range.
- **IMU during changing motion:** observe yaw rate and forward/lateral acceleration
  while the robot follows a curve with smoothly changing forward speed.

The **Sensor mode** selector offers `ideal`, `noisy`, and `student`. Selecting a
mode resets the activity and is remembered in `settings.json`. Student mode uses
ideal encoders/IMU and [StudentLidar](robotics_sim/exercises/lesson04/lidar.py).
Replace its working ray-direction formula to implement the rotation from sensor
bearings to world-space unit vectors, then restart the application. The inherited
sampler calls the world's ray-casting interface and converts hit fractions to
distances. Sensor and exercise code do not need PyBullet API calls.

Sensor readings are cached: reading, rendering, or pausing does not consume noise
or generate new measurements. Sensors sample every `ceil(0.1 / dt)` physics steps
(10 Hz at the default timestep), plus initial and final samples. For a timestep
that does not divide 0.1 seconds, use exported timestamps as the actual interval.
The displayed scan stays at its acquisition pose until the next sample.

| Sensor | Measurement | Noisy-mode model |
| --- | --- | --- |
| Encoder | Left/right shaft-angle increments, rad | 360 ticks/turn; Gaussian angle noise of 1/4 tick before cumulative-angle quantization |
| Rangefinder/lidar | Range, m, plus explicit hit flag | Gaussian 0.02 m standard deviation on hits; clipped to 0–4 m; misses remain 4 m |
| IMU | Yaw rate, rad/s; body x/y acceleration, m/s² | Gyro bias 0.01 rad/s, noise 0.005 rad/s; acceleration biases +0.02/-0.02 m/s² and noise 0.03 m/s² |

Each sensor uses its own fixed random seed, so resetting repeats the same noise
sequence. The models and constants are in `robotics_sim/sensors/`. Encoder angles
integrate ideal shaft commands; they are not measured wheel joints, and still
increase if the chassis is blocked. The IMU reads physics velocity and computes
finite-difference world acceleration before expressing it in the current body
frame. It excludes gravity, roll/pitch sensing, and a full inertial sensor model.
Initial acceleration is zero; starting an ideal velocity source can produce a
large first-sample acceleration. There is no odometry or filtering yet.

**Export CSV** writes ground-truth poses to the chosen file and sensor samples to
a companion file, e.g. `run.csv` and `run_sensors.csv`. Existing files are
overwritten. The sensor file includes timestamps, measured and ideal encoder/IMU
values, and `range_i_m`, `true_range_i_m`, `hit_i` for each ray in bearing order.
Encoder deltas cover the interval since the preceding sensor sample. The initial
ideal deltas are zero. Compare measured and ideal columns to study quantization,
bias, noise, and the distinction between sensor rate and physics rate.

Public interfaces: `EncoderSensor.read()`, `LidarSensor.scan()`, and
`ImuSensor.read()` return the latest sampled measurements. `SensorSuite` handles
advancing shaft angles and sampling on the simulation clock.

## Next increments

| Lesson | Planned addition |
| --- | --- |
| 5 | Separate ground-truth and odometry estimates |
| 6 | Velocity and pose controllers |
| 7 | Guidance and path following |
| 8 | Occupancy grid and A* planning |
| 9 | A small 2–3 DOF arm |
| 10 | Integrated sense–plan–act challenge |

Each new block should expose a small Python interface so a student implementation
can replace the reference without editing the simulation loop. The first such
interface is `VelocityProgram`; later controller/estimator interfaces will be
introduced with their lessons rather than represented by empty implementations.

Backend reference: [official PyBullet quickstart guide](https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html).

## Saved settings (Windows and Ubuntu)

Both launchers use the same Python application and the same editable
**`simulator/settings.json`** file. No platform-specific settings file or Windows
registry entries are used. Settings are saved shortly after a change and again
when the application closes. Window resizing and camera dragging are debounced
so saving does not slow down rendering.

Close the application before editing the JSON, then reopen it. You can copy the
file between Windows and Ubuntu. [`settings.example.json`](settings.example.json)
contains all supported keys with default values:

| Keys | Meaning |
| --- | --- |
| `width`, `height`, `x`, `y`, `maximized` | Normal window size/position in pixels and maximized state |
| `lesson`, `task`, `duration` | Last selected activity and length in seconds |
| `kinematics` | Lesson 3 model: `reference` or `student` |
| `sensor_mode` | Lesson 4 mode: `ideal`, `noisy`, or `student` |
| `show_frames` | Display coordinate-frame annotations |
| `label_size` | Text size, 8–36 points |
| `label_color`, `label_background`, `label_background_color` | Text color, background enabled, and background color |
| `label_background_transparency` | Background transparency percentage: 0 is opaque, 100 is clear |
| `camera_yaw`, `camera_pitch`, `camera_distance` | Camera angles in degrees and distance in metres |

Colors can be hexadecimal RGB strings, such as `"#172d3d"`. Task IDs are
`square`, `straight`, `circle` for lesson 1; `planar`, `frames`, `static` for
lesson 2; `drive_line`, `drive_circle`, `drive_spin`, `drive_inverse` for lesson 3;
and `sensor_encoders`, `sensor_lidar`, `sensor_imu` for lesson 4. Missing or
invalid values fall back to defaults. Windows
are kept within the current display bounds when restored. The simulator reopens
in Ready state; it does not restore a running simulation or previous measurements.
Explicit `--lesson`/`--duration` options (or `-Lesson`/`-Duration` on Windows)
override saved choices. Headless runs do not read or modify desktop preferences.
Delete `settings.json` to restore defaults; this personal file is ignored by Git.

## Developer notes: viewport

The interactive renderer targets the opaque boxes and cylinders used by current
lessons; it does not support mesh materials, transparency, shadows, or general
intersecting-surface rendering. `Camera.render()` is available for explicit
raster captures, but the app uses the canvas renderer.

Run `tools/benchmark_viewport.py --compare-raster` with the simulator's environment
to compare canvas paint times with raster capture. It requires a desktop or Xvfb;
timings depend on the machine and display.
