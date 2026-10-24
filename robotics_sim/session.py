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
from .exercises.lesson04.gui import SENSOR_EXERCISES, SensorActivity
from .exercises.lesson05.gui import ArmActivity


LESSONS = {"1": "Lesson 1 - System overview", "2": "Lesson 2 - Frames and transformations",
           "3": "Lesson 3 - Mobile base kinematics", "4": "Lesson 4 - Sensors and noise",
           "5": "Lesson 5 - Planar arm"}
TASKS = {
    "1": {"straight": "Preview: straight-line motion", "circle": "Preview: circular motion", "square": "Preview: square motion"},
    "2": {"planar": "Module 1: Planar transforms",
          "spatial": "Module 2: Spatial transforms",
          "rpy": "Module 2: RPY sensor to world",
          "quaternions": "Module 2: Quaternions",
          "frames": "Extension: Rotating 3D frames",
          "static": "Extension: Fixed 3D transform"},
    "3": {"drive_line": "Equal wheel speeds: straight line",
          "drive_circle": "Unequal wheel speeds: circle",
          "drive_spin": "Opposite wheel speeds: spin",
          "drive_inverse": "Inverse kinematics: body twist",
          "drive_goal": "Reach a goal: pose control",
          "drive_waypoints": "Follow two waypoints",
          "drive_goal_behind": "Reach a goal behind the robot"},
    "4": {"sensor_encoders": "Wheel encoders and forward range",
          "sensor_lidar": "Scan obstacles with 2D lidar",
          "sensor_imu": "IMU during changing motion",
          "sensor_estimation_straight": "Compare encoder and IMU: straight motion",
          "sensor_estimation_curve": "Compare encoder and IMU: curved motion",
          "sensor_estimation_blocked": "Compare estimates: blocked robot",
          "sensor_avoidance": "Reach a goal and avoid an obstacle"},
    "5": {"arm_forward": "Forward kinematics: endpoint",
          "arm_inverse": "Inverse kinematics: two branches",
          "arm_control": "Control the end-effector"},
}
DESCRIPTIONS = {
    "spatial": "Compose T_world_base and T_base_sensor, then transform P_sensor=(0,1,0). Base: Rz(90 deg), (1,2,0) m; sensor: Rx(90 deg), (0.2,0,0.5) m. Expected sensor origin (1,2.2,0.5), point (1,2.2,1.5) m. Edit solve_spatial_task. The base frame is at ground level as in the notes; the robot body centre is above it.",
    "rpy": "Assignment: sensor angles (roll,pitch,yaw)=(pi/6,-pi/9,2pi/9) rad and v_s=(1,0,0). Implement the expanded RPY matrix and Rz Ry Rx in solve_rpy_task. The sensor frame and purple vector show your result for this assignment. Compare v_w with (0.719846,0.604023,0.342020). Checks compare the full matrix with the expected rotation and your product, and test orthogonality and determinant. The drawing origin is offset vertically for visibility; directions are not translated.",
    "quaternions": "q=(sqrt(2)/2,0,0,sqrt(2)/2) in wxyz order, v=(0,1,0). Compute inverse, q*q, q*(0,v)*q_inverse and distance(q,-q). Expected rotated vector (-1,0,0), distance 0 rad. Edit solve_quaternion_task and hamilton. The exercise uses scalar-first quaternions, exactly as in the notes.",
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
    "drive_goal": "Reach (x, y, yaw) = (1 m, 0.5 m, pi/2 rad). Turn toward the goal, drive within position tolerance, then align the final orientation. Select reference to inspect the working controller or student to run your nominal_command implementation.",
    "drive_waypoints": "Reach (1 m, 0.5 m, pi/2 rad), then (1 m, 1.5 m, 0 rad). Complete position and orientation at each waypoint before advancing. The controller selector switches between the reference model and your implementation; DONE and TIMEOUT are distinct outcomes.",
    "drive_goal_behind": "Reach (-1 m, 0 m, 0 rad), starting with a goal behind the robot. Predict the initial turn, then observe TURN, DRIVE and ALIGN. Use student to run nominal_command from control.py after completing the guided exercise.",
    "sensor_encoders": "Compare wheel angular increments with speed times sample interval. The forward rangefinder measures distance to the wall; encoder shafts keep turning if blocked.",
    "sensor_lidar": "A rotating robot scans a 180-degree field of view. Green rays hit obstacles; gray rays have no return within 4 m. Student mode uses your ray-direction formula.",
    "sensor_imu": "Observe yaw rate and body-frame acceleration during smoothly changing motion. Compare ideal and noisy readings; this planar IMU excludes gravity.",
}


# Paths are relative to the simulator folder; previews need no implementation.
ACTIVITY_FILES = {
    "spatial": ("Edit solve_spatial_task; save and restart.", "robotics_sim/exercises/lesson02/spatial.py"),
    "rpy": ("Edit solve_rpy_task; save and restart.", "robotics_sim/exercises/lesson02/rpy.py"),
    "quaternions": ("Edit solve_quaternion_task and hamilton (wxyz); save and restart.", "robotics_sim/exercises/lesson02/quaternions.py"),
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
    "drive_goal": ("Complete StudentWaypointController.nominal_command; select student controller and restart after editing.", "robotics_sim/exercises/lesson03/control.py"),
    "drive_waypoints": ("Complete StudentWaypointController.nominal_command; advance only after both tolerances are met.", "robotics_sim/exercises/lesson03/control.py"),
    "drive_goal_behind": ("Complete StudentWaypointController.nominal_command; test a target behind the robot without commanding lateral motion.", "robotics_sim/exercises/lesson03/control.py"),
    "sensor_encoders": ("Optional: inspect or customize the encoder model; restart after editing.", "robotics_sim/exercises/lesson04/sensors/encoder.py"),
    "sensor_lidar": ("Edit StudentLidar.directions; select student sensor mode and restart after editing.", "robotics_sim/exercises/lesson04/lidar.py"),
    "sensor_imu": ("Optional: inspect or customize the IMU model; restart after editing.", "robotics_sim/exercises/lesson04/sensors/imu.py"),
}

for task in SENSOR_EXERCISES:
    DESCRIPTIONS[task] = (
        "Complete the sector summary and supervisor. Reach the visible goal around one isolated box; "
        "compare DONE and TIMEOUT. The reference is a local detour, not a general path planner."
        if task == "sensor_avoidance" else
        "Complete encoder odometry and inertial integration. Compare both estimates with the actual "
        "robot on the same sensor stream; orange is encoder odometry, purple is the IMU estimate. "
        "Use ideal data first, then noisy data. The blocked-robot task exposes wheel/chassis disagreement.")
    ACTIVITY_FILES[task] = (
        "Complete StudentSectors.summarize and StudentObstacleSupervisor.command; select student and restart."
        if task == "sensor_avoidance" else
        "Complete StudentOdometry.update and StudentInertial.update; select student and restart.",
        "robotics_sim/exercises/lesson04/avoidance.py" if task == "sensor_avoidance" else
        "robotics_sim/exercises/lesson04/estimation.py")
for task, method in (("arm_forward", "forward"), ("arm_inverse", "inverse"), ("arm_control", "jacobian")):
    DESCRIPTIONS[task] = {
        "arm_forward": "Predict the endpoint from the joint configuration. The teal arm follows the independent plant; compare your predicted purple point with the real endpoint.",
        "arm_inverse": "Find both joint configurations for the target. Inspect the two branches and their residuals; also try unreachable and degenerate targets.",
        "arm_control": "Complete the Jacobian and the joint-velocity controller. Move the endpoint toward the target with damping and joint-speed limits; distinguish DONE, TIMEOUT and JOINT_LIMIT.",
    }[task]
    ACTIVITY_FILES[task] = (
        "Complete StudentArmController.velocity and StudentPlanarArm.jacobian; save and restart."
        if task == "arm_control" else f"Complete StudentPlanarArm.{method}; save and restart.",
        "robotics_sim/exercises/lesson05/control.py" if task == "arm_control" else
        "robotics_sim/exercises/lesson05/kinematics.py")



class ActivitySession:
    """Completion freezes the world; only close() releases it."""

    def __init__(self, lesson="1", task=None, *, duration=16.0, dt=1 / 240, kinematics="reference", sensor_mode="ideal",
                 exercise_model="reference", goal=None):
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
        if exercise_model not in ("reference", "student"):
            raise ValueError("Exercise model must be reference or student.")
        if goal is not None and (len(goal) != 3 or not all(math.isfinite(v) for v in goal)):
            raise ValueError("Goal must contain finite x, y and heading.")
        self.exercise_model, self.goal = exercise_model, goal
        self.laboratory = None
        self.sensor_mode = sensor_mode
        self.sensors = None
        self.controller = None
        self.outcome = None
        self.arrivals = []
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
        if self.lesson == "5":
            options = {"target": self.goal[:2]} if self.goal is not None else {}
            self.laboratory = ArmActivity(self.world, self.task, model=self.exercise_model, **options)
            self.robot = self.laboratory.robot
            update = lambda robot, time, dt: self.laboratory.step(dt)
        elif self.task in SENSOR_EXERCISES:
            options = {"target": self.goal[:2]} if self.goal is not None else {}
            self.laboratory = SensorActivity(self.world, self.task, model=self.exercise_model,
                                             mode=self.sensor_mode, dt=self.dt, **options)
            self.robot = self.laboratory.robot
            self.sensors = self.laboratory.sensors
            update = self.laboratory.update
        elif self.lesson == "1":
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
        self.controller = getattr(self.robot, "waypoint_controller", None)
        if self.controller is not None:
            if self.goal is not None and self.task != "drive_waypoints":
                self.controller.goals = (tuple(self.goal),)
            self.outcome = "RUNNING"
        elif self.laboratory is not None:
            self.outcome = self.laboratory.outcome
        self.planar_scene = (frames_activity.setup_planar_scene(self.robot)
                             if self.task == "planar" else None)
        self.sim = Simulator(self.world, self.robot, update=update)
        if self.lesson == "4" and self.laboratory is None:
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
        goal_index = self.controller.index if self.controller is not None else None
        pose = self.sim.step()
        if self.task in SENSOR_EXERCISES:
            self.laboratory.after_step(self.sim.time, self.sim.steps, self.dt,
                                       final=self.sim.steps >= self.total_steps)
        elif self.sensors is not None:
            self.sensors.advance(self.sim.time, self.sim.steps, self.dt,
                                 final=self.sim.steps >= self.total_steps)
        if self.laboratory is not None:
            if not self.laboratory.done and self.sim.steps >= self.total_steps:
                self.laboratory.timeout()
            self.outcome = self.laboratory.outcome
        self.samples.append((self.sim.time, pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw))
        if self.lesson == "3":
            self.samples[-1] += (*self.robot.wheel_velocities,
                                 self.robot.command.vx, self.robot.command.omega)
        if self.controller is not None:
            gx, gy, gt = self.controller.goals[goal_index]
            position_error = math.hypot(gx - pose.x, gy - pose.y)
            angle_error = (gt - pose.yaw + math.pi) % (2 * math.pi) - math.pi
            if self.controller.index > goal_index:
                self.arrivals.append({"index": goal_index, "time_s": self.sim.time,
                                      "position_error_m": position_error,
                                      "angle_error_rad": angle_error})
            if self.controller.done:
                self.outcome = "DONE"
            elif self.sim.steps >= self.total_steps:
                self.outcome = "TIMEOUT"
            self.samples[-1] += (goal_index, self.controller.index, self.controller.phase,
                                 position_error, angle_error, self.outcome, self.controller.scale)
        if self.laboratory is not None:
            self.samples[-1] += tuple(self.laboratory.extra_values)
        if (self.outcome == "DONE" or self.sim.steps >= self.total_steps
                or (self.laboratory is not None and self.laboratory.done)):
            self.state = "Completed"
            self.sim.stop()
            if self.laboratory is not None:
                self.laboratory.stop()
        elif single:
            self.state = "Paused"

    @property
    def columns(self):
        columns = ("time_s", "x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")
        if self.lesson == "3":
            columns += ("left_rad_s", "right_rad_s", "forward_m_s", "yaw_rate_rad_s")
        if self.controller is not None:
            columns += ("goal_index", "completed_waypoints", "phase", "position_error_m",
                        "angle_error_rad", "outcome", "wheel_scale")
        if self.laboratory is not None:
            columns += tuple(self.laboratory.extra_columns)
        return columns

    def export_csv(self, destination):
        path = Path(destination)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(self.columns)
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
