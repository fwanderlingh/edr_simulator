"""Rangefinder and LiDAR control activities in the desktop application.

Edit rangefinder.py, lidar.py and avoidance.py, then click Reload exercise.
This adapter supplies the shared scene, sampling, recording and run lifecycle.
"""

import math

from robotics_sim import DifferentialDriveRobot, Pose
from robotics_sim.exercises.lesson03 import control as pose_exercise
from robotics_sim.exercises.lesson03._control_runtime import controller_parameters, wrap
from robotics_sim.sim.references.control import ReferencePoseController
from . import avoidance, estimation, rangefinder
from ._runtime import rangefinder_parameters, supervisor_parameters
from .rangefinder import StudentRangeStop
from .avoidance import StudentObstacleSupervisor, StudentSectors
from .estimation import StudentInertial, StudentOdometry
from robotics_sim.sim.references.avoidance import ReferenceObstacleSupervisor, ReferenceSectors
from robotics_sim.sim.references.estimation import ReferenceInertial, ReferenceOdometry
from robotics_sim.sim.references.rangefinder import ReferenceRangeStop
from .sensors.suite import SensorSuite


SENSOR_EXERCISES = (
    "sensor_estimation_straight", "sensor_estimation_curve",
    "sensor_estimation_blocked", "sensor_front_stop", "sensor_avoidance",
)

AVOIDANCE_STAGES = {
    "sectors": "Inspect sector distances. A supplied command rotates the robot at 0.3 rad/s; the supervisor command is inactive.",
    "stop": "Test stopping with the front-sector minimum and the completed rangefinder controller. No detour is executed.",
    "avoid": "Run the complete supervisor: reach the goal, stop for obstacles and execute the detour.",
}
ESTIMATION_STAGES = {
    "encoder": "Test encoder odometry; the IMU estimate uses the reference implementation.",
    "imu": "Test inertial integration; encoder odometry uses the reference implementation.",
    "both": "Run both estimates with the selected model and compare them with the robot pose.",
}


class SensorActivity:
    """Own one robot/sensor suite; ActivitySession owns and advances its clock."""

    def __init__(self, world, task, *, model="reference", mode="ideal", dt=1 / 240,
                 target=(3.0, 0.0, 0.0), truth_orientation=False, stage=None, lidar_model=None):
        if task not in SENSOR_EXERCISES or model not in ("student", "reference"):
            raise ValueError("Choose a supported sensor exercise and model.")
        if mode not in ("ideal", "noisy"):
            raise ValueError("Estimation and supervision use ideal or noisy sensor readings.")
        self.task, self.model_name = task, model
        self.robot = DifferentialDriveRobot(world, Pose())
        self.target = tuple(target)
        if len(self.target) != 3 or not all(math.isfinite(v) for v in self.target):
            raise ValueError("Target must contain a finite (x_m, y_m, yaw_rad) pose.")
        self.front_only = task == "sensor_front_stop"
        self.has_goal = self.front_only or task == "sensor_avoidance"
        self.stage = stage if stage is not None else (
            "stop" if self.front_only else
            avoidance.AVOIDANCE_STAGE if self.has_goal else estimation.ESTIMATION_STAGE)
        stages = ("sectors", "stop", "avoid") if self.has_goal else ("encoder", "imu", "both")
        if self.stage not in stages or self.front_only and self.stage != "stop":
            raise ValueError(f"Choose an exercise stage from {('stop',) if self.front_only else stages}.")
        self.lidar_model = "reference" if self.front_only else (
            lidar_model if lidar_model is not None else avoidance.LIDAR_MODEL if self.has_goal else None)
        self.truth_orientation = truth_orientation
        self.outcome = "RUNNING"
        self.done = False
        self.last_time = 0.0
        self.used_sectors = None
        self.wheel_scale = 1.0
        self.traces = {"encoder": [(0, 0, 0.025)], "imu": [(0, 0, 0.03)]}
        if task == "sensor_estimation_blocked":
            world.add_box(Pose(x=1.3, z=0.5), size=(0.2, 3, 1))
        elif self.has_goal:
            scene = rangefinder if self.front_only else avoidance
            obstacle_x, obstacle_y = scene.OBSTACLE_POSITION
            obstacle_size = scene.OBSTACLE_SIZE
            world.add_box(Pose(x=obstacle_x, y=obstacle_y, z=obstacle_size[2] / 2),
                          size=obstacle_size)
        self.sensors = SensorSuite(self.robot, mode, dt,
                                   rangefinder=self.front_only,
                                   motion_sensors=not self.has_goal,
                                   lidar_model=self.lidar_model)
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
            if self.stage == "sectors":
                self.controller.phase = "SECTORS"
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
        if self.front_only:
            distance, hit = float(lidar.scan()[0]), bool(lidar.hits[0])
            self.used_sectors = {"front": distance if hit else lidar.max_range,
                                 "left": None, "right": None}
            command = tuple(self.controller.command(robot.pose(), self.target,
                                                     distance, hit, lidar.max_range))
        else:
            self.used_sectors = self.summary.summarize(lidar.angles, lidar.scan(),
                                                       lidar.hits, lidar.max_range)
        if not self.front_only and self.stage == "sectors":
            # Inspect only the summary; neither the supervisor nor Lesson 3 is called.
            self.controller.phase = "SECTORS"
            command = (0.0, 0.3)
        elif not self.front_only and self.stage == "stop":
            # Reuse the first exercise on the front sector; no detour code is needed.
            front = self.used_sectors["front"]
            command = tuple(self.controller.command(robot.pose(), self.target,
                                                     front, True, lidar.max_range))
        elif not self.front_only:
            command = tuple(self.controller.command(robot.pose(), self.target,
                                                     self.used_sectors))
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
                    "left_m", "right_m", "stage", "nominal_phase", "wheel_scale",
                    "target_yaw_rad", "goal_heading_error_rad",
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
            return (self.target[0], self.target[1], math.hypot(self.target[0] - pose.x, self.target[1] - pose.y),
                    self.controller.phase, *ranges,
                    self.stage, self.pose_controller.phase, self.wheel_scale,
                    self.target[2], wrap(self.target[2] - pose.yaw),
                    *self.robot.wheel_velocities, self.outcome)
        return (self.odom.x, self.odom.y, self.odom.yaw, self.odom.speed, self.odom.yaw_rate,
                self.imu.x, self.imu.y, self.imu.yaw, self.imu.vx, self.imu.vy,
                math.hypot(self.odom.x - pose.x, self.odom.y - pose.y),
                math.hypot(self.imu.x - pose.x, self.imu.y - pose.y), self.sensors.time,
                self.stage, self.odom_model, self.imu_model, self.outcome)

    @property
    def stage_description(self):
        if self.front_only:
            return "Reach the goal with a front rangefinder stop and hysteresis."
        return (AVOIDANCE_STAGES if self.has_goal else ESTIMATION_STAGES)[self.stage]

    @property
    def implementation_hint(self):
        if self.stage == "sectors":
            return "Complete StudentSectors.summarize in avoidance.py. The scan rotation is supplied; command is not called."
        if self.stage == "stop":
            return "Reuse StudentRangeStop.command in rangefinder.py with the front-sector minimum from StudentSectors.summarize."
        return "Complete StudentSectors.summarize and StudentObstacleSupervisor.command in avoidance.py. Nominal motion reuses Lesson 3."

    @property
    def motion_feedback(self):
        if self.done:
            return {"DONE": "The controller reports completion.",
                    "TIMEOUT": "Maximum duration reached before goal completion.",
                    "COMPLETED": "Component test finished; goal arrival was not required."}[self.outcome]
        if self.stage == "sectors":
            return "Supplied scan rotation; supervisor command is inactive."
        phase = self.controller.phase
        if phase == "GO" and self.pose_controller.phase == "TURN":
            return "The nominal controller is turning toward the goal."
        if phase == "GO" and self.pose_controller.phase == "ALIGN":
            return "Final orientation alignment by the Lesson 3 pose controller."
        if phase == "STOP" and self.used_sectors is not None:
            front = self.used_sectors["front"]
            if front < self.controller.enter:
                return "Front below the stop threshold."
            if self.stage == "stop" and front < self.controller.exit:
                return "STOP retained by hysteresis: front is below the release threshold."
            if not self.front_only and max(self.used_sectors["left"], self.used_sectors["right"]) < self.controller.exit:
                return "Neither lateral sector reaches the release threshold."
        return {"GO": "Nominal control toward the goal.",
                "STOP": "Stopped; the controller evaluates whether to resume or turn.",
                "TURN": "Detour rotation toward the stored heading.",
                "PASS": "Bypass segment before returning to the goal.",
                "DONE": "The controller reports completion."}.get(phase, "Phase defined by the selected controller.")

    @property
    def readout(self):
        values = self.extra_values
        if self.has_goal:
            command = self.robot.command
            lines = [f"Goal pose: ({self.target[0]:.2f} m, {self.target[1]:.2f} m, {self.target[2]:.3f} rad)",
                     "Sensor: front rangefinder" if self.front_only else
                     f"Stage: {self.stage} | Lidar: {self.lidar_model}",
                     f"Phase: {self.controller.phase}", self.motion_feedback,
                     f"Command: v={command.vx:.3f} m/s | omega={command.omega:.3f} rad/s"]
            if not self.front_only and self.used_sectors is not None:
                lines.append("Sectors [m]: " + " | ".join(
                    f"{name}={self.used_sectors[name]:.3f}" for name in ("front", "left", "right")))
            elif not self.front_only:
                lines.append("Sectors: press Start or Step to compute the first summary.")
            if self.stage != "sectors":
                lines.append(f"Stop below {self.controller.enter:g} m | release from {self.controller.exit:g} m")
            if self.controller.phase == "TURN":
                lines.append(f"Turn target: {self.controller.turn_yaw:.3f} rad")
            elif self.controller.phase == "PASS":
                pose = self.robot.pose()
                start = self.controller.pass_start
                progress = math.hypot(pose.x - start[0], pose.y - start[1])
                lines.append(f"Bypass progress: {progress:.3f} / {self.controller.bypass_distance:g} m")
            lines.extend([f"Goal error: {values[2]:.3f} m",
                          f"Heading error: {wrap(self.target[2] - self.robot.pose().yaw):.3f} rad",
                          "Lesson 3 controller: inactive in sectors" if self.stage == "sectors" else
                          f"Lesson 3 controller: {self.model_name} | phase: {self.pose_controller.phase}"
                          if self.controller.phase == "GO" else "Lesson 3 controller: inactive",
                          f"Wheels limit: {self.wheel_max:g} rad/s", f"Outcome: {self.outcome}"])
            return "\n".join(lines)
        return (f"Encoder ({self.odom.x:.3f}, {self.odom.y:.3f}) m\n"
                f"IMU ({self.imu.x:.3f}, {self.imu.y:.3f}) m\n"
                f"Stage: {self.stage} | Encoder: {self.odom_model} | IMU: {self.imu_model}\n"
                f"Encoder error: {values[10]:.3f} m\nIMU error: {values[11]:.3f} m\n"
                "Trails: encoder orange, IMU purple\n"
                f"Orientation: {'ground-truth assisted' if self.truth_orientation else 'gyro integration'}")
