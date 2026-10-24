"""Run with python -m robotics_sim.exercises.lesson05 --task forward|geometry|control."""

import argparse
import csv
import math
from pathlib import Path
import time

import numpy as np

from robotics_sim import World
from robotics_sim.sim.visualizer import Visualizer
from .kinematics import StudentPlanarArm
from .control import StudentArmController
from .reference import ReferencePlanarArm, ReferenceArmController


def draw_arm(view, model, q, target, *, name="arm", color=(0.04, 0.5, 0.5)):
    origin = np.array([0.0, 0.0, 0.15])
    elbow = origin + [model.l1*math.cos(q[0]), model.l1*math.sin(q[0]), 0]
    tip = origin + [*model.forward(q), 0]
    view.draw_vector(origin, elbow-origin, name=name+":link1", color=color)
    view.draw_vector(elbow, tip-elbow, name=name+":link2", color=color)
    view.draw_point(tip, name=name+":tip", color=color)
    view.draw_point([*target, 0.15], name="target", color=(0.9, 0.4, 0.1))


def finite_vector(value, size, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (size,) or not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must contain {size} finite values.")
    return value


def run(args):
    if not all(math.isfinite(x) and x > 0 for x in (args.dt, args.duration)):
        raise ValueError("dt and duration must be positive and finite.")
    target = finite_vector(args.target, 2, "Target")
    plant = ReferencePlanarArm()
    model = StudentPlanarArm() if args.model == "student" else ReferencePlanarArm()
    controller_type = StudentArmController if args.model == "student" else ReferenceArmController
    controller = controller_type(model, damping=args.damping, max_speed=args.max_speed)
    destination = Path(args.csv)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with World(gui=args.gui, dt=args.dt) as world, destination.open("w", newline="") as output:
        view = Visualizer(world)
        out = csv.writer(output)
        out.writerow(["t_s", "q1_rad", "q2_rad", "X_m", "Y_m", "error_m",
                      "q1_dot_rad_s", "q2_dot_rad_s", "status"])
        if args.task == "forward":
            q = finite_vector(args.initial, 2, "Joint configuration")
            prediction = finite_vector(model.forward(q), 2, "Forward endpoint")
            actual = plant.forward(q)
            error = np.linalg.norm(prediction-actual)
            status = "OK" if error <= 1e-6 else "CHECK"
            out.writerow([0, *q, *actual, error, 0, 0, status])
            draw_arm(view, plant, q, target)
            view.draw_point([*prediction, 0.15], name="student prediction", color=(0.55, 0.2, 0.7))
            print(f"Forward: predicted={prediction}, actual={actual}, error={error:.6g} m, {status}")
            if args.gui:
                time.sleep(3)
            return status
        if args.task == "geometry":
            try:
                solutions = model.inverse(target)
            except ValueError as error:
                if np.linalg.norm(target) >= 1e-12:
                    raise
                out.writerow([0, "", "", "", "", "", 0, 0, "DEGENERATE"])
                print("DEGENERATE:", error)
                return "DEGENERATE"
            expected = plant.inverse(target)
            all_ok = len(solutions) == len(expected)
            seen = []
            for index, q in enumerate(solutions):
                q = finite_vector(q, 2, "IK solution")
                actual = plant.forward(q)
                prediction = finite_vector(model.forward(q), 2, "Forward endpoint")
                error = np.linalg.norm(target-actual)
                status = "OK" if error <= 1e-6 and np.linalg.norm(prediction-actual) <= 1e-6 else "CHECK"
                wrapped = (q+math.pi) % (2*math.pi)-math.pi
                duplicate = any(np.linalg.norm((wrapped-old+math.pi) % (2*math.pi)-math.pi) < 1e-6 for old in seen)
                seen.append(wrapped)
                if duplicate:
                    status = "CHECK"
                all_ok = all_ok and status == "OK" and not duplicate
                out.writerow([0, *q, *actual, error, 0, 0, status])
                draw_arm(view, plant, q, target, name=f"branch{index}",
                         color=(0.04, 0.5, 0.5) if index == 0 else (0.55, 0.2, 0.7))
                print(f"Branch {index+1}: q={q}, tip={actual}, residual={error:.6g} m, {status}")
            if not solutions:
                status = "CHECK" if expected else "UNREACHABLE"
                out.writerow([0, "", "", "", "", "", 0, 0, status])
                print("CHECK: the target is reachable but no solution was returned." if expected
                      else "UNREACHABLE: no geometric solution returned.")
            if solutions and not all_ok:
                print("CHECK: verify branch count, distinct configurations and forward prediction.")
            if args.gui:
                time.sleep(3)
            return ("OK" if all_ok else "CHECK") if solutions else status

        q = finite_vector(args.initial, 2, "Initial configuration")
        if np.any(np.abs(q) > math.pi):
            raise ValueError("Initial joints must be within [-pi, pi].")
        radius = np.linalg.norm(target)
        status = "RUNNING" if radius <= plant.l1+plant.l2+1e-12 else "UNREACHABLE"
        for step in range(math.ceil(args.duration/args.dt)+1):
            actual = plant.forward(q)
            error = np.linalg.norm(target-actual)
            if status == "RUNNING" and error <= 1e-3:
                status = "DONE"
            if status == "RUNNING" and step == math.ceil(args.duration/args.dt):
                status = "TIMEOUT"
            dq = finite_vector(controller.velocity(q, target), 2, "Joint velocity") if status == "RUNNING" else np.zeros(2)
            if np.max(np.abs(dq)) > args.max_speed+1e-9:
                raise ValueError("Controller violated the configured joint-speed limit.")
            if status == "RUNNING" and np.any(np.abs(q+args.dt*dq) > math.pi):
                status, dq = "JOINT_LIMIT", np.zeros(2)
            out.writerow([step*args.dt, *q, *actual, error, *dq, status])
            draw_arm(view, plant, q, target)
            if status != "RUNNING":
                break
            q = q+args.dt*dq
            world.physics.step()
            if args.gui:
                time.sleep(args.dt)
        print(f"{status}: residual={error:.6g} m, time={step*args.dt:.3f} s; CSV: {destination}")
        if args.gui:
            time.sleep(2)
        return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=("forward", "geometry", "control"), default="geometry")
    parser.add_argument("--model", choices=("student", "reference"), default="student")
    parser.add_argument("--target", nargs=2, type=float, default=(1.0, 1.0))
    parser.add_argument("--initial", nargs=2, type=float, default=(0.2, 0.5))
    parser.add_argument("--dt", type=float, default=1/120)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--damping", type=float, default=0.05)
    parser.add_argument("--max-speed", type=float, default=1.0)
    parser.add_argument("--csv", default="output/arm.csv")
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    try:
        status = run(args)
    except NotImplementedError as error:
        print("NOT IMPLEMENTED:", error)
        print("Complete the indicated method; use --model reference for comparison after your attempt.")
        return 2
    except ValueError as error:
        parser.error(str(error))
    return 0 if status in ("OK", "DONE") else 1


if __name__ == "__main__":
    raise SystemExit(main())
