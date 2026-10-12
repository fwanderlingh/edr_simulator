"""Persistent activity lifecycle shared by the desktop app and batch runner."""

import csv
import math
from pathlib import Path
from functools import partial

from . import Simulator, World
from .exercises.lesson01 import circle, square, straight
from .exercises.lesson02 import activity as frames_activity
from .exercises.lesson03 import activity as drive_activity
from .exercises.lesson04 import activity as sensors_activity


LESSONS = {"1": "Lesson 1 - System overview", "2": "Lesson 2 - Frames and transformations",
           "3": "Lesson 3 - Mobile base kinematics", "4": "Lesson 4 - Sensors and noise"}
TASKS = {
    "1": {"straight": "Preview: straight-line motion", "circle": "Preview: circular motion", "square": "Preview: square motion"},
    "2": {"planar": "Module 1: Planar transforms",
          "frames": "Extension: Rotating 3D frames",
          "static": "Extension: Fixed 3D transform"},
    "3": {"drive_line": "Equal wheel speeds: straight line",
          "drive_circle": "Unequal wheel speeds: circle",
          "drive_spin": "Opposite wheel speeds: spin",
          "drive_inverse": "Inverse kinematics: body twist"},
    "4": {"sensor_encoders": "Wheel encoders and forward range",
          "sensor_lidar": "Scan obstacles with 2D lidar",
          "sensor_imu": "IMU during changing motion"},
}
DESCRIPTIONS = {
    "square": "A first look at the world, robot, commands, and measured state. Watch a scripted square motion; later lessons will build sensing, estimation, planning, and feedback control.",
    "straight": "Explore the simulator with a scripted straight-line motion. Observe the robot's pose and simulation time as a preview of the system we will develop.",
    "circle": "Explore a scripted circular motion and the robot's changing orientation. This is a system preview; feedback control will be introduced in a later lesson.",
    "planar": "Module 1 (planar frames and transformations): edit solve_planar_task. Compose the sensor pose to map a point into the world, rotate a direction without translating it. Compare your output with the checks in the simulator README.",
    "frames": "Optional 3D extension: a sensor is attached to a rotating 3D robot. Edit point_in_world (4x4 matrices): compose world_from_sensor = world_from_robot @ robot_from_sensor, append homogeneous coordinate 1 to the sensor point, and return its 3D world coordinates. The purple line shows your point once implemented. For the Module 1 exercise, choose Planar transforms.",
    "static": "Optional 3D extension: inspect fixed 3D World, Robot, and Sensor frames. This view has no code to complete; the sensor-point exercise is point_in_world, shown in Rotating 3D frames. For the Module 1 exercise, choose Planar transforms.",
    "drive_line": "Equal wheel speeds produce straight motion. Compare distance with v * time. Wheel radius r = 0.12 m; track width L = 0.42 m.",
    "drive_circle": "Unequal wheel speeds produce a circle of radius v / omega. Positive yaw turns left; the right wheel travels farther.",
    "drive_spin": "Equal and opposite wheel speeds rotate the robot about its centre. Forward velocity is zero.",
    "drive_inverse": "Convert v = 0.24 m/s and omega = 0.4 rad/s to wheel speeds, then apply forward kinematics. Compare with the circle task.",
    "sensor_encoders": "Compare wheel angular increments with speed times sample interval. The forward rangefinder measures distance to the wall; encoder shafts keep turning if blocked.",
    "sensor_lidar": "A rotating robot scans a 180-degree field of view. Green rays hit obstacles; gray rays have no return within 4 m. Student mode uses your ray-direction formula.",
    "sensor_imu": "Observe yaw rate and body-frame acceleration during smoothly changing motion. Compare ideal and noisy readings; this planar IMU excludes gravity.",
}


# Paths are relative to the simulator folder; previews need no implementation.
ACTIVITY_FILES = {
    "straight": ("Optional: edit setup and update; save and restart.", "robotics_sim/exercises/lesson01/straight.py"),
    "circle": ("Optional: edit setup and update; save and restart.", "robotics_sim/exercises/lesson01/circle.py"),
    "square": ("Optional: edit setup and update; save and restart.", "robotics_sim/exercises/lesson01/square.py"),
    "planar": ("Edit solve_planar_task; save and restart the simulator.", "robotics_sim/exercises/lesson02/planar.py"),
    "frames": ("Edit point_in_world; save and restart the simulator.", "robotics_sim/exercises/lesson02/frames.py"),
    "static": ("Optional: customize ROBOT_FROM_SENSOR and POINT_IN_SENSOR.", "robotics_sim/exercises/lesson02/activity.py"),
    "drive_line": ("Edit StudentDifferentialDrive.forward; select student kinematics and restart after editing.", "robotics_sim/exercises/lesson03/kinematics.py"),
    "drive_circle": ("Edit StudentDifferentialDrive.forward; select student kinematics and restart after editing.", "robotics_sim/exercises/lesson03/kinematics.py"),
    "drive_spin": ("Edit StudentDifferentialDrive.forward; select student kinematics and restart after editing.", "robotics_sim/exercises/lesson03/kinematics.py"),
    "drive_inverse": ("Edit StudentDifferentialDrive.forward and inverse; select student kinematics and restart after editing.", "robotics_sim/exercises/lesson03/kinematics.py"),
    "sensor_encoders": ("Optional: inspect or customize the encoder model; restart after editing.", "robotics_sim/exercises/lesson04/sensors/encoder.py"),
    "sensor_lidar": ("Edit StudentLidar.directions; select student sensor mode and restart after editing.", "robotics_sim/exercises/lesson04/lidar.py"),
    "sensor_imu": ("Optional: inspect or customize the IMU model; restart after editing.", "robotics_sim/exercises/lesson04/sensors/imu.py"),
}


class ActivitySession:
    """Completion freezes the world; only close() releases it."""

    def __init__(self, lesson="1", task=None, *, duration=16.0, dt=1 / 240, kinematics="reference", sensor_mode="ideal"):
        if lesson not in LESSONS:
            raise ValueError("Unknown lesson.")
        task = task or next(iter(TASKS[lesson]))
        if task not in TASKS[lesson]:
            raise ValueError("This task does not belong to the selected lesson.")
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("Activity length must be finite and positive.")
        if kinematics not in ("reference", "student"):
            raise ValueError("Kinematics must be reference or student.")
        if sensor_mode not in ("ideal", "noisy", "student"):
            raise ValueError("Sensor mode must be ideal, noisy, or student.")
        self.sensor_mode = sensor_mode
        self.sensors = None
        self.lesson, self.task = lesson, task
        self.duration, self.dt = duration, dt
        self.world = World(gui=False, dt=dt)
        try:
            self._build_activity(kinematics)
        except Exception:
            self.world.close()
            raise
        self.total_steps = math.ceil(duration / dt)
        self.state = "Ready"
        self.samples = []

    def _build_activity(self, kinematics):
        if self.lesson == "1":
            example = {"straight": straight, "circle": circle, "square": square}[self.task]
            self.robot = example.setup(self.world)
            update = example.update
        else:
            example = {"2": frames_activity, "3": drive_activity,
                       "4": sensors_activity}[self.lesson]
            options = {"task": self.task}
            if self.lesson == "3":
                options["kinematics"] = kinematics
            self.robot = example.setup(self.world, **options)
            update = partial(example.update, task=self.task)
        self.sim = Simulator(self.world, self.robot, update=update)
        if self.lesson == "4":
            self.sensors = example.setup_sensors(
                self.robot, task=self.task, mode=self.sensor_mode, dt=self.dt)

    def start(self):
        if self.state in ("Ready", "Paused"):
            self.state = "Running"

    def pause(self):
        if self.state == "Running":
            self.state = "Paused"

    def step(self, *, single=False):
        if self.state in ("Completed", "Closed"):
            return
        if self.state != "Running" and not single:
            return
        pose = self.sim.step()
        if self.sensors is not None:
            self.sensors.advance(self.sim.time, self.sim.steps, self.dt,
                                 final=self.sim.steps >= self.total_steps)
        self.samples.append((self.sim.time, pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw))
        if self.lesson == "3":
            self.samples[-1] += (*self.robot.wheel_velocities,
                                 self.robot.command.vx, self.robot.command.omega)
        if self.sim.steps >= self.total_steps:
            self.state = "Completed"
            self.sim.stop()
        elif single:
            self.state = "Paused"

    def export_csv(self, destination):
        path = Path(destination)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            columns = ("time_s", "x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")
            if self.lesson == "3":
                columns += ("left_rad_s", "right_rad_s", "forward_m_s", "yaw_rate_rad_s")
            writer.writerow(columns)
            writer.writerows(self.samples)
        if self.sensors is not None:
            sensor_path = path.with_name(path.stem + "_sensors.csv")
            with sensor_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(self.sensors.columns)
                writer.writerows(self.sensors.rows)

    def close(self):
        self.world.close()
        self.state = "Closed"
