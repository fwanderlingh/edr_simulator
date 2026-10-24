"""Guided reactive supervisor for one isolated obstacle; this is not a planner.

From simulator/: python -m robotics_sim.exercises.lesson04.avoidance --model reference
The student sector summary and supervisor command are deliberately unfinished.
"""

import argparse
import csv
import math
import time as clock
from pathlib import Path

from robotics_sim import DifferentialDriveRobot, Pose, Simulator, World
from robotics_sim.sim.visualizer import Visualizer
from .reference import ReferenceObstacleSupervisor, ReferenceSectors
from .sensors.suite import SensorSuite


class StudentSectors(ReferenceSectors):
    def summarize(self, angles, ranges, hits, max_range):
        """Return dict(front,left,right) of minima, or None for invalid/missing data.

        Front: |bearing|<=30 degrees. Misses are valid observations to max_range.
        """
        raise NotImplementedError("Complete StudentSectors.summarize in lesson04/avoidance.py")


class StudentObstacleSupervisor(ReferenceObstacleSupervisor):
    def command(self, pose, goal, sectors, age):
        """Return (v,omega), updating self.phase; stale data must stop the robot.

        Keep a turn direction and bypass progress; do not immediately point back
        at the box after a clear scan. Finishing and timeout are separate outcomes.
        """
        raise NotImplementedError("Complete StudentObstacleSupervisor.command in lesson04/avoidance.py")


def run(args):
    summary = (ReferenceSectors if args.model == "reference" else StudentSectors)()
    controller = (ReferenceObstacleSupervisor if args.model == "reference" else StudentObstacleSupervisor)()
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with World(gui=args.gui, dt=1 / 240) as world:
        robot = DifferentialDriveRobot(world, Pose())
        world.add_box(Pose(x=1.5, z=0.5), size=(0.4, 0.6, 1))
        sensors = SensorSuite(robot, args.mode, 1 / 240)
        goal = (3.0, 0.0)
        visualizer = Visualizer(world)
        visualizer.draw_point((*goal, 0.15), name="Target")
        observed = {"sectors": None, "age": 0.0}

        def update(robot, time, dt):
            sectors = summary.summarize(sensors.lidar.angles, sensors.lidar.scan(),
                                        sensors.lidar.hits, sensors.lidar.max_range)
            age = time - sensors.time
            observed.update(sectors=sectors, age=age)
            v, omega = controller.command(robot.pose(), goal, sectors, age)
            wl, wr = robot.motor_model.inverse(v, omega)
            peak = max(abs(wl), abs(wr))
            scale = min(1.0, 5.0 / peak) if peak else 1.0
            robot.set_wheel_velocities(scale * wl, scale * wr)

        sim = Simulator(world, robot, update=update)
        with args.csv.open("w", newline="", encoding="utf-8") as stream:
            out = csv.writer(stream)
            out.writerow(["time_s", "phase", "x_m", "y_m", "yaw_rad", "goal_error_m",
                          "front_m", "left_m", "right_m", "scan_age_s", "left_rad_s", "right_rad_s"])
            steps = math.ceil(args.duration / sim.dt)
            try:
                for step in range(1, steps + 1):
                    pose = sim.step()
                    sensors.advance(sim.time, step, sim.dt, final=step == steps)
                    if args.gui:
                        clock.sleep(sim.dt)
                    sectors = observed["sectors"]
                    error = math.hypot(goal[0] - pose.x, goal[1] - pose.y)
                    if step % sensors.period_steps == 0 or controller.phase == "DONE" or step == steps:
                        out.writerow([sim.time, controller.phase, pose.x, pose.y, pose.yaw, error,
                                      *(sectors[key] if sectors else "" for key in ("front", "left", "right")),
                                      observed["age"], *robot.wheel_velocities])
                    if controller.phase == "DONE":
                        break
            finally:
                sim.stop()
    result = "DONE" if controller.phase == "DONE" else "TIMEOUT"
    print(f"{result}: target error={error:.4f} m; CSV={args.csv}")
    return 0 if result == "DONE" else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=("student", "reference"), default="student")
    parser.add_argument("--mode", choices=("ideal", "noisy"), default="ideal")
    parser.add_argument("--duration", type=float, default=35.0)
    parser.add_argument("--csv", type=Path, default=Path("output/avoidance.csv"))
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("duration must be finite and positive")
    try:
        parser.exit(run(args))
    except NotImplementedError as error:
        parser.exit(2, f"Exercise not implemented: {error}. Use --model reference to compare.\n")


if __name__ == "__main__":
    main()
