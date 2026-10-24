# Lesson 5: planar arm kinematics and control

All practical activities are in the same desktop application. Select **Lesson 5 -
Planar arm** and choose an activity. Use **Student** for your implementation and
**reference** for comparison after your own attempt. Save source changes and
restart the application before repeating the experiment.

## Part I: predict, implement and inspect

1. Solve the guided geometric exercise in the notes, including units and joint
   conventions. `q2` is relative to link 1.
2. Complete `StudentPlanarArm.forward` in [kinematics.py](kinematics.py). Choose
   **Forward kinematics: endpoint** and press **Step**. The teal arm follows an
   independent plant; the purple endpoint is your prediction. The result is
   `OK` or `CHECK`, with the numerical error in the measurements panel.
3. Implement `StudentPlanarArm.inverse`. Choose **Inverse kinematics: two
   branches**, set target x/y to `(1, 1)`, apply Reset, and press **Step**. Inspect
   both branches and their residuals. Change the target to `(3, 0)` for the
   unreachable case and `(0, 0)` for the degenerate folded-arm case.
4. Export the CSV and compare the branch count, coordinates and residuals.

Unfinished student methods report the method to complete; loading the exercise
still leaves the scene available for inspection. The app does not silently
substitute a reference solution for student code. The inverse check distinguishes
an actually unreachable target from a student implementation that returns no
solution for a reachable target.

## Part II: control the endpoint

Implement `StudentPlanarArm.jacobian` in [kinematics.py](kinematics.py), then
`StudentArmController.velocity` in [control.py](control.py). First derive the
Jacobian and reason about the allowed endpoint directions in a singular pose.
The controller converts Cartesian error into damped joint velocities and applies
a common speed scale.

Choose **Control the end-effector**, enter the desired target, apply Reset and
press **Start**. Pause or single-step to inspect the transient. The duration is
a timeout, not a prescribed arrival time. The initial joints are `(0.2, 0.5)`,
with a 1 mm Cartesian completion tolerance. Exported values include joint
configuration, actual endpoint, residual, joint speed and outcome.

Compare `DONE`, `TIMEOUT`, `JOINT_LIMIT` and `UNREACHABLE`; a stopped arm need not
have reached its target. Joints are limited to `[-pi, pi]` and speeds to the
controller's `max_speed`. Adjust `damping` or `max_speed` in the `ArmActivity` configuration in
`gui.py`, save, restart and repeat with the same target. `DEFAULT_INITIAL` in [gui.py](gui.py)
sets the initial configuration for additional experiments.

The plant integrates prescribed joint velocities. The common application draws
the chain, goals and predictions; it does not simulate contact, grasp forces or
collision-free motion between inverse-kinematics branches. Instructor models are
kept separately in [reference.py](reference.py), and worked results are in the
notes' final appendix.
