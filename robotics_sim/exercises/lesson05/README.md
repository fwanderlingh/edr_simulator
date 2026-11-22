# Lesson 5: planar arm kinematics and control

All practical activities are in the same desktop application. Select **Lesson 5 -
Planar arm** and choose an activity. Use **Student** for your implementation and
**reference** for comparison after your own attempt. Save source changes and
click **Reload exercise** before repeating the experiment.

## Part I: predict, implement and inspect

1. Solve the guided forward-kinematics exercise in the notes. Compose the
   homogeneous transforms, then read position and orientation from the result.
   `q2` is relative to link 1.
2. Complete `StudentPlanarArm.forward` in [kinematics.py](kinematics.py). Choose
   **Forward kinematics: endpoint** and press **Step**. The teal arm follows an
   independent plant; the purple endpoint is your prediction. The result is
   `OK` or `CHECK`, with the numerical error in the measurements panel.
3. Repeat for the guided exercise's configurations by changing `DEFAULT_INITIAL`
   in `gui.py` and reloading the exercise. Export the CSV and compare predicted
   and measured endpoint positions.

Unfinished student methods report the method to complete; loading the exercise
still leaves the scene available for inspection. The app does not silently
substitute a reference solution for student code.

**Inverse kinematics: two branches** remains available as an optional extension.
Its analytic inverse and special cases are outside the introductory activity;
`StudentPlanarArm.inverse` is not required for forward kinematics or control.

## Part II: control the endpoint

Implement `StudentPlanarArm.jacobian` in [kinematics.py](kinematics.py), then
`StudentArmController.velocity` in [control.py](control.py). First derive the
Jacobian and reason about the allowed endpoint directions in a singular pose.
The controller computes Cartesian error and the desired endpoint velocity, then
uses the ready-made `joint_velocity` function from the notes' appendix to obtain
joint velocities. Copy that function into `control.py` before the controller
class; its numerical derivation is not part of the exercise. Apply a common
speed scale to respect `max_speed`.

Choose **Control the end-effector**, enter the desired target, apply Reset and
press **Start**. Pause or single-step to inspect the transient. The duration is
a timeout, not a prescribed arrival time. The initial joints are `(0.2, 0.5)`,
with a 1 mm Cartesian completion tolerance. Exported values include joint
configuration, actual endpoint, residual, joint speed and outcome.

Compare `DONE`, `TIMEOUT`, `JOINT_LIMIT` and `UNREACHABLE`; a stopped arm need not
have reached its target. Joints are limited to `[-pi, pi]` and speeds to the
controller's `max_speed`. Reduce `max_speed` in the `ArmActivity` configuration in
`gui.py`, save, click **Reload exercise** and repeat with the same target. Keep
the default `damping`. To observe a singular configuration, set `DEFAULT_INITIAL`
in [gui.py](gui.py) to `(0, 0)` and target `(1, 0)`, reload and reset. Compare the
nonzero residual with a repeat from a bent configuration.

The plant integrates prescribed joint velocities. The common application draws
the chain, goals and predictions; it does not simulate contact, grasp forces or
collision-free motion between inverse-kinematics branches. Instructor models are
kept separately in the simulator's [reference package](../../sim/references/planar_arm.py), and worked results are in the
notes' final appendix.
