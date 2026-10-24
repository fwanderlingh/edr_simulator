"""Compare encoder odometry and inertial integration on the same sensor stream.

From simulator/: python -m robotics_sim.exercises.lesson04.estimation --model reference
Complete the two Student.update methods after the guided exercise in the notes.
"""

import argparse
import csv
import math
import time as clock
from pathlib import Path

from robotics_sim import DifferentialDriveRobot, Pose, Simulator, World
from robotics_sim.sim.visualizer import Visualizer
from .reference import ReferenceInertial, ReferenceOdometry
from .sensors.suite import SensorSuite


class StudentOdometry(ReferenceOdometry):
    def update(self, left_delta, right_delta, dt):
        """Update x,y,yaw,speed,yaw_rate using measured wheel increments and dt."""
        # TODO: wheel increments -> ds, da -> pose; never use ground truth here.
        raise NotImplementedError("Complete StudentOdometry.update in lesson04/estimation.py")


class StudentInertial(ReferenceInertial):
    def update(self, gyro, ax_body, ay_body, dt, *, truth_yaw=None):
        """Update yaw,x,y,vx,vy; body acceleration is already in m/s² without gravity.

        truth_yaw=None: propagate yaw from gyro. Otherwise use supplied yaw only
        for the explicitly orientation-assisted comparison. All states start at 0.
        """
        # TODO: yaw -> acceleration in world -> position and velocity integration.
        raise NotImplementedError("Complete StudentInertial.update in lesson04/estimation.py")


def run(args):
    odom = (ReferenceOdometry if args.model == "reference" else StudentOdometry)()
    imu = (ReferenceInertial if args.model == "reference" else StudentInertial)()
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with World(gui=args.gui, dt=args.dt) as world:
        robot = DifferentialDriveRobot(world, Pose())
        visualizer = Visualizer(world)
        visualizer.draw_point((0, 0, 0.2), name="Initial position")
        if args.task == "blocked":
            world.add_box(Pose(x=1.3, z=0.5), size=(0.2, 3, 1))

        def update(robot, time, dt):
            robot.set_twist(0.2 + 0.1 * math.sin(time) if args.task == "curve" else 0.2,
                            0.4 if args.task == "curve" else 0.0)

        sim = Simulator(world, robot, update=update)
        sensors = SensorSuite(robot, args.mode, args.dt)
        last_time = sensors.time
        with args.csv.open("w", newline="", encoding="utf-8") as stream:
            out = csv.writer(stream)
            out.writerow(["time_s", "true_x_m", "true_y_m", "true_yaw_rad",
                          "enc_x_m", "enc_y_m", "enc_yaw_rad", "enc_speed_m_s",
                          "imu_x_m", "imu_y_m", "imu_yaw_rad", "imu_vx_m_s", "imu_vy_m_s",
                          "enc_position_error_m", "imu_position_error_m"])
            out.writerow([0.0] * 15)
            steps = math.ceil(args.duration / args.dt)
            try:
                for step in range(1, steps + 1):
                    pose = sim.step()
                    sensors.advance(sim.time, step, args.dt, final=step == steps)
                    if args.gui:
                        clock.sleep(args.dt)
                    if sensors.time == last_time:
                        continue
                    dt = sensors.time - last_time
                    last_time = sensors.time
                    odom.update(*sensors.encoder.read(), dt)
                    imu.update(*sensors.imu.read(), dt,
                               truth_yaw=pose.yaw if args.orientation == "truth" else None)
                    enc_error = math.hypot(odom.x - pose.x, odom.y - pose.y)
                    imu_error = math.hypot(imu.x - pose.x, imu.y - pose.y)
                    visualizer.draw_point((odom.x, odom.y, 0.25), name="Encoder estimate", color=(0.1, 0.5, 0.5))
                    visualizer.draw_point((imu.x, imu.y, 0.35), name="IMU estimate", color=(0.6, 0.2, 0.8))
                    out.writerow([sim.time, pose.x, pose.y, pose.yaw,
                                  odom.x, odom.y, odom.yaw, odom.speed,
                                  imu.x, imu.y, imu.yaw, imu.vx, imu.vy, enc_error, imu_error])
            finally:
                sim.stop()
    print(f"{args.task}, {args.mode}, orientation={args.orientation}: "
          f"encoder error={enc_error:.4f} m; IMU error={imu_error:.4f} m; CSV={args.csv}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=("student", "reference"), default="student")
    parser.add_argument("--task", choices=("straight", "curve", "blocked"), default="straight")
    parser.add_argument("--mode", choices=("ideal", "noisy"), default="ideal")
    parser.add_argument("--orientation", choices=("gyro", "truth"), default="gyro")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--dt", type=float, default=1 / 240)
    parser.add_argument("--csv", type=Path, default=Path("output/estimation.csv"))
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.duration, args.dt)):
        parser.error("duration and dt must be finite and positive")
    try:
        run(args)
    except NotImplementedError as error:
        parser.exit(2, f"Exercise not implemented: {error}. Use --model reference to compare.\n")


if __name__ == "__main__":
    main()
