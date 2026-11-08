"""Rangefinder and LiDAR control activities in the desktop application.

Edit rangefinder.py, lidar.py and avoidance.py, then click Reload exercise.
This adapter supplies the shared scene, sampling, recording and run lifecycle.
"""

import math

from robotics_sim import DifferentialDriveRobot, Pose
from robotics_sim.exercises.lesson03 import control as pose_exercise
from robotics_sim.exercises.lesson03._control_runtime import controller_parameters
from robotics_sim.sim.references.control import ReferencePoseController
from . import avoidance, estimation
from ._runtime import rangefinder_parameters, supervisor_parameters
from .rangefinder import StudentRangeStop
from .avoidance import StudentObstacleSupervisor, StudentSectors
from .estimation import StudentInertial, StudentOdometry
from robotics_sim.sim.references.sensors import (
    ReferenceInertial, ReferenceObstacleSupervisor, ReferenceOdometry, ReferenceRangeStop, ReferenceSectors,
)
from .sensors.suite import SensorSuite


SENSOR_EXERCISES = (
    "sensor_estimation_straight", "sensor_estimation_curve",
    "sensor_estimation_blocked", "sensor_front_stop", "sensor_avoidance",
)


class SensorActivity:
    """Own one robot/sensor suite; ActivitySession owns and advances its clock."""

    def __init__(self, world, task, *, model="reference", mode="ideal", dt=1 / 240,
                 target=(3.0, 0.0), truth_orientation=False):
        if task not in SENSOR_EXERCISES or model not in ("student", "reference"):
            raise ValueError("Choose a supported sensor exercise and model.")
        if mode not in ("ideal", "noisy"):
            raise ValueError("Estimation and supervision use ideal or noisy sensor readings.")
        self.task, self.model_name = task, model
        self.robot = DifferentialDriveRobot(world, Pose())
        self.target = tuple(target)
        if len(self.target) != 2 or not all(math.isfinite(v) for v in self.target):
            raise ValueError("Target must contain finite x and y coordinates.")
        self.front_only = task == "sensor_front_stop"
        self.has_goal = self.front_only or task == "sensor_avoidance"
        self.stage = "stop" if self.front_only else (
            avoidance.AVOIDANCE_STAGE if self.has_goal else estimation.ESTIMATION_STAGE)
        stages = ("sectors", "stop", "avoid") if self.has_goal else ("encoder", "imu", "both")
        if self.stage not in stages:
            raise ValueError(f"Choose an exercise stage from {stages} in the exercise file.")
        self.truth_orientation = truth_orientation
        self.outcome = "RUNNING"
        self.done = False
        self.last_time = 0.0
        self.used_sectors = None
        self.used_age = 0.0
        self.wheel_scale = 1.0
        self.traces = {"encoder": [(0, 0, 0.025)], "imu": [(0, 0, 0.03)]}
        if task == "sensor_estimation_blocked":
            world.add_box(Pose(x=1.3, z=0.5), size=(0.2, 3, 1))
        elif self.has_goal:
            world.add_box(Pose(x=1.5, z=0.5), size=(0.4, 0.6, 1))
        self.sensors = SensorSuite(self.robot, mode, dt,
                                   rangefinder=self.front_only,
                                   motion_sensors=not self.has_goal,
                                   lidar_model="reference" if self.front_only else (
                                       avoidance.LIDAR_MODEL if self.has_goal else None))
        if self.has_goal:
            options = controller_parameters()
            self.wheel_max = options["wheel_max"]
            pose_class = pose_exercise.StudentPoseController if model == "student" else ReferencePoseController
            self.pose_controller = pose_class(**options)
            self.summary = None if self.front_only else (
                StudentSectors if model == "student" else ReferenceSectors)()
            controller_class = (StudentRangeStop if model == "student" else ReferenceRangeStop) if self.front_only or self.stage == "stop" else (
                StudentObstacleSupervisor if model == "student" else ReferenceObstacleSupervisor)
            self.controller = controller_class(
                                   pose_controller=self.pose_controller, stage=self.stage,
                                   radius=self.robot.motor_model.wheel_radius,
                                   width=self.robot.motor_model.track_width,
                                   **(rangefinder_parameters() if self.front_only else supervisor_parameters()))
        else:
            self.odom_model = model if self.stage in ("encoder", "both") else "reference"
            self.imu_model = model if self.stage in ("imu", "both") else "reference"
            self.odom = (StudentOdometry if self.odom_model == "student" else ReferenceOdometry)()
            self.imu = (StudentInertial if self.imu_model == "student" else ReferenceInertial)()

    def update(self, robot, time, dt):
        if self.done:
            self.stop()
            return
        if not self.has_goal:
            robot.set_twist(0.2 + 0.1 * math.sin(time)
                            if self.task == "sensor_estimation_curve" else 0.2,
                            0.4 if self.task == "sensor_estimation_curve" else 0.0)
            return
        lidar = self.sensors.lidar
        self.used_age = time - self.sensors.time
        if self.front_only:
            distance, hit = float(lidar.scan()[0]), bool(lidar.hits[0])
            self.used_sectors = {"front": distance if hit else lidar.max_range,
                                 "left": None, "right": None}
            command = tuple(self.controller.command(robot.pose(), self.target,
                                                     distance, hit, lidar.max_range, self.used_age))
        else:
            self.used_sectors = self.summary.summarize(lidar.angles, lidar.scan(),
                                                       lidar.hits, lidar.max_range)
        if not self.front_only and self.stage == "sectors":
            # Inspect only the summary; neither the supervisor nor Lesson 3 is called.
            self.controller.phase = "SECTORS"
            command = (0.0, 0.3)
        elif not self.front_only and self.stage == "stop":
            # Reuse the first exercise on the front sector; no detour code is needed.
            front = self.used_sectors["front"] if self.used_sectors is not None else float("nan")
            command = tuple(self.controller.command(robot.pose(), self.target,
                                                     front, True, lidar.max_range, self.used_age))
        elif not self.front_only:
            command = tuple(self.controller.command(robot.pose(), self.target,
                                                     self.used_sectors, self.used_age))
        if len(command) != 2 or not all(math.isfinite(v) for v in command):
            raise ValueError("Supervisor.command must return finite (v, omega).")
        wl, wr = robot.motor_model.inverse(*command)
        peak = max(abs(wl), abs(wr))
        self.wheel_scale = min(1.0, self.wheel_max / peak) if peak else 1.0
        robot.set_wheel_velocities(self.wheel_scale * wl, self.wheel_scale * wr)

    def after_step(self, time, step, dt, *, final=False):
        self.sensors.advance(time, step, dt, final=final)
        if self.has_goal:
            if self.controller.phase == "DONE":
                self.outcome, self.done = "DONE", True
            return
        if self.sensors.time <= self.last_time:
            return
        interval = self.sensors.time - self.last_time
        pose = self.robot.pose()
        self.odom.update(*self.sensors.encoder.read(), interval)
        self.imu.update(*self.sensors.imu.read(), interval,
                        truth_yaw=pose.yaw if self.truth_orientation else None)
        self.last_time = self.sensors.time
        self.traces["encoder"].append((self.odom.x, self.odom.y, 0.025))
        self.traces["imu"].append((self.imu.x, self.imu.y, 0.03))

    def stop(self):
        self.robot.set_twist(0.0, 0.0)

    def timeout(self):
        self.outcome = "TIMEOUT" if self.has_goal and self.stage == "avoid" else "COMPLETED"
        self.done = True
        self.stop()

    @property
    def extra_columns(self):
        if self.has_goal:
            return ("target_x_m", "target_y_m", "goal_error_m", "phase", "front_m",
                    "left_m", "right_m", "scan_age_s", "stage", "nominal_phase", "wheel_scale",
                    "left_rad_s", "right_rad_s", "outcome")
        return ("enc_x_m", "enc_y_m", "enc_yaw_rad", "enc_speed_m_s", "enc_yaw_rate_rad_s",
                "imu_x_m", "imu_y_m", "imu_yaw_rad", "imu_vx_m_s", "imu_vy_m_s",
                "enc_position_error_m", "imu_position_error_m", "sensor_time_s",
                "stage", "encoder_model", "imu_model", "outcome")

    @property
    def extra_values(self):
        pose = self.robot.pose()
        if self.has_goal:
            ranges = tuple(self.used_sectors[key] if self.used_sectors and self.used_sectors[key] is not None else ""
                           for key in ("front", "left", "right"))
            return (*self.target, math.hypot(self.target[0] - pose.x, self.target[1] - pose.y),
                    self.controller.phase, *ranges, self.used_age,
                    self.stage, self.pose_controller.phase, self.wheel_scale,
                    *self.robot.wheel_velocities, self.outcome)
        return (self.odom.x, self.odom.y, self.odom.yaw, self.odom.speed, self.odom.yaw_rate,
                self.imu.x, self.imu.y, self.imu.yaw, self.imu.vx, self.imu.vy,
                math.hypot(self.odom.x - pose.x, self.odom.y - pose.y),
                math.hypot(self.imu.x - pose.x, self.imu.y - pose.y), self.sensors.time,
                self.stage, self.odom_model, self.imu_model, self.outcome)

    @property
    def readout(self):
        values = self.extra_values
        if self.has_goal:
            return (f"Goal ({self.target[0]:.2f}, {self.target[1]:.2f}) m\n" +
                    ("Sensor: front rangefinder\n" if self.front_only else
                     f"Stage: {self.stage} | Lidar: {avoidance.LIDAR_MODEL}\n") +
                    f"Phase: {self.controller.phase}\nGoal error: {values[2]:.3f} m\n"
                    f"Lesson 3 controller: {self.model_name} | Wheels limit: {self.wheel_max:g} rad/s\n"
                    f"Outcome: {self.outcome}")
        return (f"Encoder ({self.odom.x:.3f}, {self.odom.y:.3f}) m\n"
                f"IMU ({self.imu.x:.3f}, {self.imu.y:.3f}) m\n"
                f"Stage: {self.stage} | Encoder: {self.odom_model} | IMU: {self.imu_model}\n"
                f"Encoder error: {values[10]:.3f} m\nIMU error: {values[11]:.3f} m\n"
                "Trails: encoder orange, IMU purple\n"
                f"Orientation: {'ground-truth assisted' if self.truth_orientation else 'gyro integration'}")
