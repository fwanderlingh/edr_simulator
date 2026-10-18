# Lesson 2: Frames and transformations

Open the simulator with the usual launcher and select **Lesson 2 - Frames and
transformations**. This directory contains the planar exercise, a standalone
calculation for worked Example 3.5, three Module 2 exercises, and optional 3D extensions.

## Module 1: Planar transforms

Select **Module 1: Planar transforms** to explore worked Example 3.5 from the
notes. The sensor offset is `(0.5, 0)` and the sensor-frame point is `(1, 0)`.

In the student version, complete
[solve_planar_task](planar.py) to:

1. Compose `world_from_robot @ robot_from_sensor` and transform the point
   using homogeneous coordinate `1`.
2. Transform the direction using homogeneous coordinate `0`.
3. Return the two Cartesian components of the point and direction.

Save the file, close and reopen the simulator, and select the same activity.
Unimplemented exercises display a yellow warning. Compare the **World**,
**Robot**, and **Sensor** frames with the notes' diagram. The sensor origin is
`(2, 1.5) m`; **Sensor point** and the live `point in world` readout should show
`(2, 2.5) m`. **Direction (no translation)** and `direction in world` should
show `(0, 1)`. The direction arrow starts at the sensor for visualization;
its components contain no translation.

This planar activity is the core Module 1 exercise; the other Lesson 2
core activities are listed below under Module 2; optional activities are labelled as extensions.

### Standalone Example 3.5 script

[example_3_5.py](example_3_5.py) is an optional terminal script that executes
the calculations from Example 3.5 and checks their numerical results,
including the distinction between a point and a direction. It does not open
the graphical interface or replace the simulator activity; it is a bridge
between the calculation on paper and the interactive scene.

## Module 2: spatial transforms, RPY and quaternions

Select the corresponding **Module 2** activity in Lesson 2. Edit its exercise
file, save and restart the simulator. The public student version contains TODO
stubs and displays a warning until implemented. The private version includes
worked solutions. All three scenes are static: inspect them immediately or use
Start, Pause and Reset normally. Results appear below the scene.

- **Spatial transforms (E2.3)** — [spatial.py](spatial.py): return the composed
  4x4 pose and transformed point. Inputs are `Rz(90 deg)`, base origin `(1,2,0)`,
  sensor rotation `Rx(90 deg)`, sensor offset `(0.2,0,0.5)` and point `(0,1,0)`.
  Check sensor origin `(1,2.2,0.5)` and point `(1,2.2,1.5)` metres. The composed
  rotation is `[[0,0,1],[1,0,0],[0,1,0]]`. The mathematical base origin is on
  the ground, distinct from the rendered robot's body centre.
- **RPY, order and gimbal lock (E2.1)** — [rpy.py](rpy.py): return `Rz Ry Rx`
  and `Rx Ry Rz`. For `(roll,pitch,yaw)=(90,0,90)` degrees, the x axis maps
  to `(0,1,0)` and `(0,0,1)` respectively. The two frames are offset only to
  make comparison easier. The readout also checks that `(0,90,0)` and
  `(30,90,30)` degrees produce the same orientation. Inputs to the function
  are radians; gimbal lock affects the angle representation, not the matrix.
- **Quaternions (E2.2)** — [quaternions.py](quaternions.py): implement Hamilton
  multiplication, conjugate/inverse, composition, sandwich rotation and the
  sign-invariant angular distance. With `a=sqrt(2)/2`, `q=(a,0,0,a)` and
  `v=(0,1,0)`, expect inverse `(a,0,0,-a)`, square `(0,0,0,1)`, rotated vector
  `(-1,0,0)` and distance between `q` and `-q` equal to zero radians.

These exercises use **wxyz (scalar first)**, exactly as the notes. The lower-level
geometry/PyBullet API uses **xyzw**; convert explicitly using `q[[1,2,3,0]]`
when passing a notes-order quaternion to `quaternion_to_matrix`. Both conventions
are common; changing component order does not change the Hamilton product.
As a cross-check for E2.3, use `hamilton((a,0,0,a), (a,a,0,0))`: the result
`(0.5,0.5,0.5,0.5)` yields the same composed rotation matrix.

Scene inputs and annotation construction are in [module2.py](module2.py).

## 3D extensions

The 3D extensions are optional. **Extension: Fixed 3D transform** only shows
the World, Robot, and Sensor frames; it has no code to complete.

In **Extension: Rotating 3D frames**, edit
[point_in_world](frames.py), which receives 4x4 matrices
`world_from_robot` and `robot_from_sensor` and a 3D `point_in_sensor`:

1. Compose `world_from_sensor = world_from_robot @ robot_from_sensor`.
2. Append homogeneous coordinate `1` to the point and multiply.
3. Return the 3D world coordinates (drop the last component).

Until the function is implemented the sensor point is not drawn; afterwards a
purple line from the sensor origin to the point appears. Reference geometry
functions are in [`../../geometry/`](../../geometry/).

## Conventions and implementation notes

- Metres, seconds, radians; right-handed frames; z points up.
- Column vectors: `T_A_B` maps coordinates expressed in B into A.
- Composition reads right to left: `T_W_S = T_W_R @ T_R_S`.
- A point uses homogeneous coordinate `1`; a direction uses `0`.
- The 3D extension uses fixed-axis RPY:
  `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`.
- Geometry API quaternions use `(x, y, z, w)` and are normalized on input;
  Module 2 exercise quaternions use the notes' `(w, x, y, z)` order.
- At RPY gimbal lock, the inverse conversion chooses roll zero.
- `Pose` is a body-centre pose; its default z is 0.16 m above the floor.
- Body-frame planar velocity uses the robot's current yaw.

`Visualizer` provides `draw_frame`, `draw_vector`, `draw_point`, `draw_path`,
and `draw_text`. Reusing a name updates the annotation. Annotations are
no-ops in headless mode. The geometry tests compare composition and rotations
against PyBullet's independent transform operations.

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

The planar scene data (`PLANAR_WORLD_FROM_ROBOT`, `PLANAR_ROBOT_FROM_SENSOR`,
the point, and the direction) are defined in **activity.py**. The first matrix
sets the robot's initial pose. `setup_planar_scene(robot)` attaches the sensor
frame and local observations to that specific robot. Both the drawing and the
exercise inputs use its current simulated pose, so the sensor follows any
translation or rotation you add in `update`. Reset recreates the attachment.
**planar.py** contains only the mathematical exercise, with its inputs supplied
by the scene. Editing the initial scene data requires restarting the app.
