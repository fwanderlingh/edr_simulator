"""Guided waypoint-control exercise for Lesson 3, Part II.

Complete StudentWaypointController.nominal_command using the course notes.
Run from simulator/: python -m robotics_sim.exercises.lesson03.control
Use --model reference to compare after making your own prediction.
"""

import argparse
import csv
import math
from pathlib import Path
import time

from robotics_sim import DifferentialDriveRobot, Pose, Simulator, World
from robotics_sim.sim.visualizer import Visualizer


DEFAULT_GOALS = ((1.0, 0.5, math.pi / 2), (1.0, 1.5, 0.0))


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class WaypointController:
    """Supplied state, wheel conversion and saturation; poses are ground truth."""

    def __init__(self, goals=DEFAULT_GOALS, *, kp=0.8, ka=2.0, kt=1.5,
                 eps_p=0.03, eps_a=0.03, angle_in=0.08, angle_out=0.6,
                 wheel_max=5.0):
        self.goals = tuple(tuple(goal) for goal in goals)
        if not self.goals or any(len(g) != 3 or not all(math.isfinite(x) for x in g)
                                 for g in self.goals):
            raise ValueError("Supply at least one finite (x_m, y_m, yaw_rad) goal.")
        positive = (kp, ka, kt, eps_p, eps_a, wheel_max)
        if not all(math.isfinite(x) and x > 0 for x in positive):
            raise ValueError("Gains, tolerances and wheel limit must be positive and finite.")
        if not 0 < angle_in < angle_out < math.pi / 2:
            raise ValueError("Require 0 < angle_in < angle_out < pi/2.")
        self.kp, self.ka, self.kt = kp, ka, kt
        self.eps_p, self.eps_a = eps_p, eps_a
        self.angle_in, self.angle_out = angle_in, angle_out
        self.wheel_max = wheel_max
        self.index, self.phase = 0, "TURN"
        self.nominal = (0.0, 0.0)
        self.scale = 1.0

    @property
    def done(self):
        return self.index == len(self.goals)

    def nominal_command(self, pose):
        raise NotImplementedError("Choose the student or reference controller.")

    def update(self, robot, time, dt):
        """Called before each physics step. The exercise returns body (v, omega)."""
        if self.done:
            self.nominal = (0.0, 0.0)
        else:
            self.nominal = tuple(self.nominal_command(robot.pose()))
        if len(self.nominal) != 2 or not all(math.isfinite(x) for x in self.nominal):
            raise ValueError("nominal_command must return finite (v_m_s, omega_rad_s).")
        wl, wr = robot.motor_model.inverse(*self.nominal)
        peak = max(abs(wl), abs(wr))
        self.scale = min(1.0, self.wheel_max / peak) if peak else 1.0
        robot.set_wheel_velocities(self.scale * wl, self.scale * wr)


class StudentWaypointController(WaypointController):
    def nominal_command(self, pose):
        """Return (v, omega), updating self.index and self.phase when necessary.

        Guided steps (the complete working comparison is in control_reference.py):
        1. Read gx, gy, gt = self.goals[self.index]. Compute distance first.
        2. Within eps_p choose ALIGN; otherwise use TURN or DRIVE with hysteresis.
        3. In ALIGN use the wrapped final-angle error. Within eps_a, increment
           index, set TURN or DONE, and return a zero command for that step.
        4. Else compute the bearing error. TURN has zero forward speed;
           DRIVE uses kp*distance. Both use ka*bearing_error for omega.
        5. Return the nominal command. update already applies the wheel limit.
        """
        # TODO: implement the phase transitions and the two nominal velocities.
        raise NotImplementedError(
            "Complete StudentWaypointController.nominal_command in "
            "robotics_sim/exercises/lesson03/control.py (Lesson 3, Part II)."
        )


def run(*, model="student", goals=DEFAULT_GOALS, csv_path="output/control.csv",
        gui=False, duration=60.0, dt=1 / 240, wheel_max=5.0):
    """Run the exercise and return evidence for each reached waypoint."""
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("Duration must be positive and finite.")
    if model == "reference":
        from .control_reference import ReferenceWaypointController
        controller_class = ReferenceWaypointController
    elif model == "student":
        controller_class = StudentWaypointController
    else:
        raise ValueError("Choose model student or reference.")
    controller = controller_class(goals, wheel_max=wheel_max)
    csv_path = Path(csv_path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    arrivals = []
    with World(gui=gui, dt=dt) as world:
        robot = DifferentialDriveRobot(world, Pose(x=0, y=0, yaw=0))
        sim = Simulator(world, robot, update=controller.update)
        view = Visualizer(world)
        for i, (gx, gy, _) in enumerate(controller.goals):
            view.draw_point((gx, gy, 0.2), name=f"goal {i + 1}")
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            out = csv.writer(stream)
            out.writerow(("time_s", "goal_index", "completed_waypoints", "phase",
                          "x_m", "y_m", "yaw_rad", "position_error_m", "angle_error_rad",
                          "w_left_rad_s", "w_right_rad_s", "wheel_scale"))
            try:
                for _ in range(math.ceil(duration / dt)):
                    active = controller.index
                    sim.step()
                    pose = robot.pose()
                    gx, gy, gt = controller.goals[active]
                    ep = math.hypot(gx - pose.x, gy - pose.y)
                    ea = wrap(gt - pose.yaw)
                    wl, wr = robot.wheel_velocities
                    out.writerow((sim.time, active, controller.index, controller.phase,
                                  pose.x, pose.y, pose.yaw, ep, ea, wl, wr, controller.scale))
                    if controller.index > active:
                        arrivals.append({"index": active, "time_s": sim.time,
                                         "position_error_m": ep, "angle_error_rad": ea})
                    view.draw_text((pose.x, pose.y, 0.7),
                                   f"{controller.phase} {controller.index}/{len(controller.goals)}",
                                   name="controller")
                    if controller.done:
                        break
                    if gui:
                        time.sleep(dt)
            finally:
                sim.stop()
        result = {"status": "DONE" if controller.done else "TIMEOUT",
                  "completed": controller.index, "arrivals": arrivals,
                  "final_wheels": robot.wheel_velocities, "csv": str(csv_path)}
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--model", choices=("student", "reference"), default="student")
    parser.add_argument("--csv", default="output/control.csv")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--behind", action="store_true", help="Start with a target behind the base.")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--dt", type=float, default=1 / 240)
    parser.add_argument("--wheel-max", type=float, default=5.0)
    args = parser.parse_args(argv)
    goals = ((-1.0, 0.0, 0.0),) if args.behind else DEFAULT_GOALS
    try:
        result = run(model=args.model, goals=goals, csv_path=args.csv, gui=args.gui,
                     duration=args.duration, dt=args.dt, wheel_max=args.wheel_max)
    except NotImplementedError as error:
        print(f"NOT IMPLEMENTED: {error}")
        return 2
    except ValueError as error:
        parser.error(str(error))
    print(f"{result['status']}: {result['completed']}/{len(goals)} waypoints; {result['csv']}")
    for arrival in result["arrivals"]:
        print(f"  Goal {arrival['index'] + 1}: {arrival['position_error_m']:.5f} m, "
              f"{arrival['angle_error_rad']:.5f} rad at {arrival['time_s']:.2f} s")
    return 0 if result["status"] == "DONE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
