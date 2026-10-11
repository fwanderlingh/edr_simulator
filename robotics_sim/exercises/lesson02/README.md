# Lesson 2: Frames and transformations

Open the simulator with the usual launcher and select **Lesson 2 - Frames and
transformations**. This directory contains the planar exercise, a standalone
calculation for worked Example 3.5, and optional 3D extensions.

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
activities are explicitly labelled as 3D extensions.

### Standalone Example 3.5 script

[example_3_5.py](example_3_5.py) is an optional terminal script that executes
the calculations from Example 3.5 and checks their numerical results,
including the distinction between a point and a direction. It does not open
the graphical interface or replace the simulator activity; it is a bridge
between the calculation on paper and the interactive scene.

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
- Quaternions use `(x, y, z, w)` and are normalized on input.
- At RPY gimbal lock, the inverse conversion chooses roll zero.
- `Pose` is a body-centre pose; its default z is 0.16 m above the floor.
- Body-frame planar velocity uses the robot's current yaw.

`Visualizer` provides `draw_frame`, `draw_vector`, `draw_point`, `draw_path`,
and `draw_text`. Reusing a name updates the annotation. Annotations are
no-ops in headless mode. The geometry tests compare composition and rotations
against PyBullet's independent transform operations.
