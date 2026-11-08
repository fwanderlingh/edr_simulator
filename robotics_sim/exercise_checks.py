"""Checks for the selected activity, independent of presentation and student state.

Sensor references replay the same measurements, including noise. Checks describe
the current activity; they do not certify a solution for every possible input.
"""

import math

import numpy as np

from .exercises.lesson02 import activity as frames_activity
from .exercises.lesson02 import module2
from .sim.references.sensors import ReferenceInertial, ReferenceOdometry, ReferenceSectors
from .robots.differential_drive import ReferenceDifferentialDrive


def close(actual, expected):
    return np.shape(actual) == np.shape(expected) and bool(np.allclose(actual, expected, atol=1e-6, rtol=0))


def verified(probe):
    """An unfinished or invalid numerical result is a failed check, not a UI error."""
    try:
        return bool(probe())
    except Exception:
        return False


class ExerciseChecks:
    def __init__(self, session):
        self.session = session
        self.odom = ReferenceOdometry()
        self.odom_midpoint = ReferenceOdometry(integration="midpoint")
        self.imu = ReferenceInertial()
        self._sensor_row = 1  # The initial capture has no elapsed interval.
        self._sensor_time = 0.0

    def evaluate(self, *, module2_output=None, planar_output=None, frame_output=None):
        session = self.session
        task = session.task
        ran = session.sim.steps > 0
        completed = session.state == "Completed"
        if task in module2.TASKS:
            return (module2_output.checks if module2_output is not None else
                    tuple((label, False) for label in module2.CHECK_LABELS[task]))
        if task == "planar":
            scene = session.planar_scene
            transform = scene.world_from_robot() @ scene.robot_from_sensor
            return (
                ("Point includes rotation and translation", planar_output is not None and
                 verified(lambda: close(planar_output[0], (transform @ np.r_[scene.point_in_sensor, 1])[:2]))),
                ("Direction uses rotation only", planar_output is not None and
                 verified(lambda: close(planar_output[1], transform[:2, :2] @ scene.direction_in_sensor))),
            )
        if task in ("frames", "static"):
            transform = session.robot.pose().matrix() @ frames_activity.ROBOT_FROM_SENSOR
            return (("Sensor point maps into the world frame", frame_output is not None and
                     verified(lambda: close(frame_output, (transform @ np.r_[frames_activity.POINT_IN_SENSOR, 1])[:3]))),)
        if session.controller is not None:
            ctrl = session.controller
            arrivals = session.arrivals
            all_arrived = len(arrivals) == len(ctrl.goals)
            checks = [
                ("All waypoints reached", all_arrived and session.outcome == "DONE"),
                (f"Arrival position errors ≤ {ctrl.eps_pos:g} m", all_arrived and
                 all(a["position_error_m"] <= ctrl.eps_pos for a in arrivals)),
            ]
            if task != "drive_proportional":
                checks.append((f"Arrival heading errors ≤ {ctrl.eps_angle:g} rad", all_arrived and
                               all(abs(a["angle_error_rad"]) <= ctrl.eps_angle for a in arrivals)))
            checks.append((f"Wheel speeds stay within {ctrl.wheel_max:g} rad/s", ran and
                           all(max(abs(row[7]), abs(row[8])) <= ctrl.wheel_max + 1e-9
                               for row in session.samples)))
            return tuple(checks)
        if session.lesson == "3":
            model = session.robot.motor_model
            reference = ReferenceDifferentialDrive(model.wheel_radius, model.track_width)
            return (
                ("Run completed", completed),
                ("Forward wheel kinematics match the reference", ran and verified(lambda:
                 all(close(model.forward(*w), reference.forward(*w))
                     for w in ((2, 2), (1.3, 2.7), (-1, 1))))),
                ("Inverse wheel kinematics match the reference", ran and verified(lambda:
                 all(close(model.inverse(*v), reference.inverse(*v))
                     for v in ((0.24, 0.4), (0, -0.5), (-0.2, 0))))),
            )
        if session.lesson == "5":
            lab = session.laboratory
            if task == "arm_forward":
                return (("Model evaluated", lab.evaluated),
                        ("Forward endpoint error ≤ 0.000001 m", lab.model_error is not None and lab.model_error <= 1e-6))
            if task == "arm_inverse":
                special = lab.outcome in ("UNREACHABLE", "DEGENERATE")
                return (
                    ("All distinct IK branches or target edge case handled", lab.outcome in ("OK", "UNREACHABLE", "DEGENERATE")),
                    ("Each branch reaches the target (≤ 0.000001 m)", lab.evaluated and (special or
                     bool(lab.branch_solutions) and all(np.linalg.norm(lab.plant.forward(q) - lab.target_xy) <= 1e-6
                                                       for q in lab.branch_solutions))),
                    ("Forward model agrees with the plant", lab.evaluated and (special or
                     lab.model_error is not None and lab.model_error <= 1e-6)),
                )
            return (
                ("Controller reached DONE", lab.outcome == "DONE"),
                ("Endpoint residual ≤ 0.001 m", lab.evaluated and lab.residual <= 1e-3),
                ("Joint positions and speeds stay within limits", ran and all(
                    max(abs(row[7]), abs(row[8])) <= math.pi and
                    max(abs(row[14]), abs(row[15])) <= lab.controller.max_speed + 1e-9
                    for row in session.samples)),
            )
        if session.lesson == "4":
            return self._sensor_checks(ran, completed)
        return ()  # Lesson 1 contains motion previews, not implementation exercises.

    def _sensor_checks(self, ran, completed):
        session = self.session
        lab = session.laboratory
        sensors = session.sensors
        if lab is not None and not lab.has_goal:
            sample_yaws = {row[0]: row[6] for row in session.samples} if lab.truth_orientation else {}
            for row in sensors.rows[self._sensor_row:]:
                interval = row[0] - self._sensor_time
                self.odom.update(*row[1:3], interval)
                self.odom_midpoint.update(*row[1:3], interval)
                self.imu.update(*row[5:8], interval,
                                truth_yaw=sample_yaws[row[0]] if lab.truth_orientation else None)
                self._sensor_time = row[0]
            self._sensor_row = len(sensors.rows)
            sampled = self._sensor_row > 1
            odom_check = (
                "Encoder estimate matches arc or midpoint integration", sampled and verified(lambda: any(close(
                    [lab.odom.x, lab.odom.y, lab.odom.yaw, lab.odom.speed, lab.odom.yaw_rate],
                    [ref.x, ref.y, ref.yaw, ref.speed, ref.yaw_rate])
                    for ref in (self.odom, self.odom_midpoint))))
            imu_check = (
                ("IMU estimate matches the sampled-data reference", sampled and verified(lambda: close(
                    [lab.imu.x, lab.imu.y, lab.imu.yaw, lab.imu.vx, lab.imu.vy],
                    [self.imu.x, self.imu.y, self.imu.yaw, self.imu.vx, self.imu.vy]))))
            return (("Run completed", completed),
                    *((odom_check,) if lab.stage in ("encoder", "both") else ()),
                    *((imu_check,) if lab.stage in ("imu", "both") else ()))
        if lab is not None:
            lidar = sensors.lidar
            if lab.front_only:
                data = [dict(zip(session.columns, row)) for row in session.samples]
                stopped = any(row["front_m"] != "" and row["front_m"] < lab.controller.enter
                              and row["left_rad_s"] == row["right_rad_s"] == 0 for row in data)
                return (
                    ("Rangefinder stop experiment completed", completed),
                    ("Robot stopped for a nearby obstacle", stopped),
                    ("Only the front ray is used", len(lidar.angles) == 1 and lidar.angles[0] == 0),
                    (f"Wheel speeds stay within {lab.wheel_max:g} rad/s", ran and all(
                        max(abs(row[-3]), abs(row[-2])) <= lab.wheel_max + 1e-9 for row in session.samples)),
                )
            expected = ReferenceSectors().summarize(lidar.angles, lidar.scan(), lidar.hits, lidar.max_range)
            sector_check = (
                "Scan sectors match the reference", ran and verified(lambda:
                    lab.summary.summarize(lidar.angles, lidar.scan(), lidar.hits, lidar.max_range) == expected
                    and expected is not None))
            wheel_check = (
                f"Wheel speeds stay within {lab.wheel_max:g} rad/s", ran and all(
                    max(abs(row[-3]), abs(row[-2])) <= lab.wheel_max + 1e-9 for row in session.samples))
            if lab.stage == "sectors":
                return (("Sector observation completed", completed), sector_check, wheel_check)
            if lab.stage == "stop":
                data = [dict(zip(session.columns, row)) for row in session.samples]
                stopped = any(row["front_m"] != "" and row["front_m"] < lab.controller.enter
                              and row["left_rad_s"] == row["right_rad_s"] == 0 for row in data)
                return (("Stop experiment completed", completed),
                        ("Robot stopped for a nearby obstacle", stopped), sector_check, wheel_check)
            return (
                ("Supervisor reached DONE", lab.outcome == "DONE"),
                (f"Goal distance ≤ {lab.controller.eps_pos:g} m", ran and math.hypot(
                    lab.target[0] - session.robot.pose().x, lab.target[1] - session.robot.pose().y) <= lab.controller.eps_pos),
                sector_check, wheel_check,
            )
        lidar = sensors.lidar
        yaw = session.robot.pose().yaw
        expected = np.column_stack((np.cos(yaw + lidar.angles), np.sin(yaw + lidar.angles),
                                    np.zeros(len(lidar.angles))))
        return (
            ("Run completed", completed),
            ("Sensors recorded a new sample", sensors.time > 0),
            ("Lidar directions match the scan angles and robot yaw", verified(lambda: close(lidar.directions(yaw), expected))),
            ("Ranges are finite and within sensor limits", bool(np.all(np.isfinite(lidar.ranges)) and
             np.all((lidar.ranges >= 0) & (lidar.ranges <= lidar.max_range)))),
        )
