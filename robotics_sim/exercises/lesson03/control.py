"""Guided waypoint-control exercise for Lesson 3, Part II.

Complete StudentWaypointController.wheel_command using the course notes.
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


GOALS = ((1.0, 0.5, math.pi / 2), (1.0, 1.5, 0.0))  # (x [m], y [m], heading [rad])
GAIN = 0.8                  # 1/s; forward speed from position error
GAIN_BEARING = 2.0          # 1/s; turn toward the goal
GAIN_HEADING = 1.5          # 1/s; align the final heading
EPS_POS = 0.03              # m
EPS_ANGLE = 0.03            # rad
ANGLE_IN = 0.08             # rad; enter DRIVE
ANGLE_OUT = 0.6             # rad; return to TURN


def _controller_parameters():
    """Use the exercise settings for both models and both GUI/CLI entry points."""
    return dict(gain=GAIN, gain_bearing=GAIN_BEARING, gain_heading=GAIN_HEADING,
                eps_pos=EPS_POS, eps_angle=EPS_ANGLE, angle_in=ANGLE_IN, angle_out=ANGLE_OUT)


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class WaypointController:
    """Supplied state and actuator limits; the exercise computes wheel references."""

    def __init__(self, goals=GOALS, *, gain=GAIN, gain_bearing=GAIN_BEARING, gain_heading=GAIN_HEADING,
                 eps_pos=EPS_POS, eps_angle=EPS_ANGLE, angle_in=ANGLE_IN, angle_out=ANGLE_OUT,
                 wheel_max=5.0):
        self.goals = tuple(tuple(goal) for goal in goals)
        if not self.goals or any(len(g) != 3 or not all(math.isfinite(x) for x in g)
                                 for g in self.goals):
            raise ValueError("Supply at least one finite (x_m, y_m, yaw_rad) goal.")
        positive = (gain, gain_bearing, gain_heading, eps_pos, eps_angle, wheel_max)
        if not all(math.isfinite(x) and x > 0 for x in positive):
            raise ValueError("Gains, tolerances and wheel limit must be positive and finite.")
        if not 0 < angle_in < angle_out < math.pi / 2:
            raise ValueError("Require 0 < angle_in < angle_out < pi/2.")
        self.gain, self.gain_bearing, self.gain_heading = gain, gain_bearing, gain_heading
        self.eps_pos, self.eps_angle = eps_pos, eps_angle
        self.angle_in, self.angle_out = angle_in, angle_out
        self.wheel_max = wheel_max
        self.index, self.phase = 0, "TURN"
        self.nominal = (0.0, 0.0)
        self.scale = 1.0

    @property
    def done(self):
        return self.index == len(self.goals)

    def wheel_command(self, pose, r, L):
        raise NotImplementedError("Choose the student or reference controller.")

    def update(self, robot, time, dt):
        """Apply the exercise's wheel references, with a common actuator limit."""
        if self.done:
            references = (0.0, 0.0)
        else:
            references = tuple(self.wheel_command(
                robot.pose(), robot.motor_model.wheel_radius, robot.motor_model.track_width))
        if len(references) != 2 or not all(math.isfinite(x) for x in references):
            raise ValueError("wheel_command must return two finite wheel speeds in rad/s.")
        self.nominal = robot.motor_model.forward(*references)
        wl, wr = references
        peak = max(abs(wl), abs(wr))
        self.scale = min(1.0, self.wheel_max / peak) if peak else 1.0
        robot.set_wheel_velocities(self.scale * wl, self.scale * wr)


class StudentWaypointController(WaypointController):
    def wheel_command(self, pose, r, L):
        """Return (w_left, w_right) in rad/s; update self.index and self.phase.

        The robot supplies r (wheel radius, m) and L (track width, m).
        Read self.goals, self.gain, self.gain_bearing, self.gain_heading,
        self.eps_pos, self.eps_angle, self.angle_in and self.angle_out.

        Guided steps (derive the controller from the notes before comparison):
        1. Read gx, gy, gt = self.goals[self.index]. Compute distance first.
        2. Within eps_pos choose ALIGN; otherwise use TURN or DRIVE with hysteresis.
        3. In ALIGN use gain_heading * wrapped final-angle error. Within eps_angle, increment
           index, set TURN or DONE, and return a zero command for that step.
        4. Else compute the bearing error with
           wrap(math.atan2(gy - pose.y, gx - pose.x) - pose.yaw).
           TURN has zero forward speed;
           DRIVE uses gain*distance. Both use gain_bearing*bearing_error for omega.
        5. Convert (v, omega) with both complete inverse-kinematics equations,
           using r and L as in proportional.py. Return the two wheel references;
           update applies them with the common wheel-speed limit.
        """
        # TODO: errors -> phase and (v, omega) -> complete wheel-reference equations.
        raise NotImplementedError(
            "Complete StudentWaypointController.wheel_command in "
            "robotics_sim/exercises/lesson03/control.py (Lesson 3, Part II)."
        )


def run(*, model="student", goals=None, csv_path="output/control.csv",
        gui=False, duration=60.0, dt=1 / 240, wheel_max=5.0):
    """Run the exercise and return evidence for each reached waypoint."""
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("Duration must be positive and finite.")
    if model == "reference":
        from robotics_sim.sim.references.waypoint_control import ReferenceWaypointController
        controller_class = ReferenceWaypointController
    elif model == "student":
        controller_class = StudentWaypointController
    else:
        raise ValueError("Choose model student or reference.")
    controller = controller_class(GOALS if goals is None else goals,
                                  wheel_max=wheel_max, **_controller_parameters())
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
    goals = ((-1.0, 0.0, 0.0),) if args.behind else GOALS
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
