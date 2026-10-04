"""Open the desktop laboratory by default; retain an explicit batch mode."""

import argparse
import math
from pathlib import Path

from .session import ActivitySession, LESSONS


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("Expected a finite positive number.")
    return number


def main():
    parser = argparse.ArgumentParser(description="Robotics Laboratory (metres, seconds, radians).")
    parser.add_argument("--lesson", choices=tuple(LESSONS), help="Override the remembered lesson.")
    parser.add_argument("--headless", action="store_true", help="Run one activity without the desktop application.")
    parser.add_argument("--duration", type=positive_float, help="Override the remembered activity length (headless default: 16 s).")
    parser.add_argument("--dt", type=positive_float, default=1 / 240, help="Fixed physics timestep in seconds.")
    parser.add_argument("--csv", type=Path, help="Export measured poses when the activity completes.")
    args = parser.parse_args()
    if not args.headless:
        from .app import run_app
        run_app(lesson=args.lesson, duration=args.duration, dt=args.dt, csv_path=args.csv)
        return
    session = ActivitySession(args.lesson or "1", duration=args.duration or 16, dt=args.dt)
    try:
        session.start()
        while session.state == "Running":
            session.step()
        pose = session.robot.pose()
        print(f"Completed {session.sim.steps} steps ({session.sim.time:.3f} s). "
              f"Position: ({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f}) m; yaw: {pose.yaw:.3f} rad.")
    except KeyboardInterrupt:
        print("Simulation stopped.")
    finally:
        try:
            if args.csv:
                session.export_csv(args.csv)
        finally:
            session.close()


if __name__ == "__main__":
    main()
