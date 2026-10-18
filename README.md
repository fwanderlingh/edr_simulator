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
- [Math to NumPy cheat sheet](#math-to-numpy-cheat-sheet)
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

### Exercises you complete

Lesson 1 provides ready-to-run motion previews, including the square; no code
changes are needed. Select **Preview: square motion** and press **Start**.
Lessons 2–4 contain exercises with `TODO` markers; the simulator reads your
changes on the next launch. Until you complete an exercise, the app shows
"not implemented yet" with a yellow background instead of results.

| Lesson | File | Complete |
|---|---|---|
| 2 | `robotics_sim/exercises/lesson02/planar.py` | `solve_planar_task` (Module 1) |
| 2 | `robotics_sim/exercises/lesson02/spatial.py` | `solve_spatial_task` (Module 2, E2.3) |
| 2 | `robotics_sim/exercises/lesson02/rpy.py` | `solve_rpy_task` (Module 2, E2.1) |
| 2 | `robotics_sim/exercises/lesson02/quaternions.py` | `hamilton`, `solve_quaternion_task` (Module 2, E2.2; wxyz) |
| 2 | `robotics_sim/exercises/lesson02/frames.py` | `point_in_world` (3D extension) |
| 3 | `robotics_sim/exercises/lesson03/kinematics.py` | `forward`, `inverse` (select **student**) |
| 4 | `robotics_sim/exercises/lesson04/lidar.py` | `StudentLidar.directions` (select **student**) |

Finished solutions are provided by the instructor.
## Using the desktop app

Choose a lesson and task, then press **Start**. **Pause**, **Step**,
**Reset / Apply length**, and **Export CSV** control the activity. It starts
Ready and remains open when complete. Changing the task or resetting clears its
measurements, so export first if you need to keep them. The duration applies on
reset or when selecting another task. Close the window or choose **File > Exit**
to quit.

Left-drag to orbit the 3D view, **Shift + left-drag** to pan, and scroll to zoom.
Choose **View > Reset camera** to restore the activity's original centre, angles, and zoom.
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

## Math to NumPy cheat sheet

Naming convention: `a_from_b` is the transform $T_{ab}$ that converts coordinates
from frame `b` to frame `a` (the pose of `b` expressed in `a`); `p_in_a` is point
`p` expressed in frame `a`. Frame names cancel along a chain:
`world_from_robot @ robot_from_sensor` gives `world_from_sensor`.

```python
import numpy as np
```

| Math | Python (NumPy) | Notes |
|---|---|---|
| $\mathbf{p}=(x,y)^T$ | `p = np.array([x, y])` | 1D `float` array |
| homogeneous point $(x,y,1)^T$ | `np.append(p, 1)` | translation applies |
| homogeneous direction $(v_x,v_y,0)^T$ | `np.append(v, 0)` | free vector ignores translation |
| back to Cartesian | `ph[:2]` | drop the last component |
| $\cos\theta,\ \sin\theta$ | `np.cos(theta)`, `np.sin(theta)` | radians (`np.deg2rad(degrees)`) |
| $R(\theta)=\begin{bmatrix}c&-s\\ s&c\end{bmatrix}$ | `R = np.array([[c, -s], [s, c]])` | |
| $T=\begin{bmatrix}R&t\\0&1\end{bmatrix}$ | `T = np.eye(3); T[:2, :2] = R; T[:2, 2] = t` | block assignment by slicing |
| rotation block of $T$ | `T[:2, :2]` | |
| translation of $T$ | `T[:2, 2]` | |
| matrix-vector product $A\mathbf{x}$ | `A @ x` | **not** `*` (element-wise) |
| composition $T_{ac}=T_{ab}\,T_{bc}$ | `a_from_c = a_from_b @ b_from_c` | order matters: $T_{ab}T_{bc}\ne T_{bc}T_{ab}$ |
| transpose $R^T$ | `R.T` | |
| rotation inverse $R^{-1}=R^T$ | `R.T` | rotations only |
| homogeneous inverse $T^{-1}$ | `np.linalg.inv(T)` | in general $T^{-1}\ne T^T$ |
| $T_{ba}=T_{ab}^{-1}$ | `b_from_a = np.linalg.inv(a_from_b)` | |

Worked examples:

| Math | Python |
|---|---|
| $T_{ws}=T_{wr}T_{rs}$ | `world_from_sensor = world_from_robot @ robot_from_sensor` |
| point: $\tilde{p}_w=T_{ws}\begin{bmatrix}p_s\\1\end{bmatrix}$ | `point_in_world = world_from_sensor @ np.append(point_in_sensor, 1)` |
| direction: $\tilde{d}_w=T_{ws}\begin{bmatrix}d_s\\0\end{bmatrix}$ | `direction_in_world = world_from_sensor @ np.append(direction_in_sensor, 0)` |

The result of a homogeneous transform has one extra component (3 elements in
2D); take `[:2]` for Cartesian coordinates, e.g.
`point_in_world = (world_from_sensor @ np.append(point_in_sensor, 1))[:2]`.

Sanity check: the "from" frame of the matrix must match the frame of the vector
(`_from_sensor` with `_in_sensor`), and the result is in the "to" frame
(`world_from_` gives `_in_world`).
## Lessons

### Lesson 1: System overview

[Lesson 1 exercise details](robotics_sim/exercises/lesson01/README.md) cover
the scripted motion previews and the optional velocity-schedule extension.

### Lesson 2: frames and transformations

[Lesson 2 exercise details](robotics_sim/exercises/lesson02/README.md) cover
the planar transformation activity, the standalone Example 3.5 script,
Module 2 exercises on spatial transforms (E2.3), RPY (E2.1), and quaternions
(E2.2), and the optional 3D extensions.

### Lesson 3: mobile base kinematics

[Lesson 3 exercise details](robotics_sim/exercises/lesson03/README.md) describe
the differential-drive tasks, equations, reference model, and expected
ideal-kinematics assumptions.

### Lesson 4: sensors and noise

[Lesson 4 exercise details](robotics_sim/exercises/lesson04/README.md) cover
the encoder, lidar, and IMU activities, sensor modes, sampling behavior, noise
models, and exported sensor data.

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
interface is `setup(world)` plus `update(robot, time, dt)`; later controller and
estimator interfaces will be introduced with their lessons rather than
represented by empty implementations.

All four lessons now declare robots and issue commands inside their lesson
folders. Lesson 1 has one file per preview (`straight.py`, `circle.py`,
`square.py`); Lessons 2, 3, and 4 each use `activity.py`, whose `task` argument
selects the UI example. Lesson 4 also declares obstacles and attaches sensors
there. The UI lists these paths in bold alongside the formula exercise file.
Edit and save the source, then restart the application to load changes.

The simulation core already supports `sim.register_robot(name, robot,
update=callback)` before the first step. Robots must belong to the same world
and have unique names and bodies. Each callback receives its robot, simulation
time, and timestep; all callbacks run before commands are applied and the shared
world advances once. `sim.stop()` clears commands for every registered robot.
`sim.robot` remains the primary robot used by today's UI and CSV output;
`sim.robots` exposes a read-only registry. Multi-robot selection, labels, and
measurements remain future UI work. Existing `program=` command sources remain
supported for compatibility, but new examples use explicit `update` callbacks.

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
`straight`, `circle`, `square` for lesson 1; `planar`, `frames`, `static` for
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
