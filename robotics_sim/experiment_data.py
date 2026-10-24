"""Detached experiment records shared by plots and the activity's CSV schema."""

from copy import deepcopy

from .session import LESSONS, TASKS


def capture_experiment(session):
    """Copy a completed or partial run without retaining its physics world.

    Selecting/resetting an activity can close that world immediately afterwards.
    Sensor samples retain their own acquisition timestamps and sampling rate.
    """
    if not session.samples:
        return None
    metadata = {"dt": session.dt, "duration": session.duration,
                "elapsed": session.sim.time, "state": session.state}
    if session.sensors is not None:
        metadata["lidar_angles"] = tuple(float(a) for a in session.sensors.lidar.angles)
        metadata["lidar_max_range"] = float(session.sensors.lidar.max_range)
    model = getattr(session.robot, "motor_model", None)
    if model is not None:
        metadata["wheel_radius"] = model.wheel_radius
        metadata["track_width"] = model.track_width
    laboratory = session.laboratory
    if session.lesson == "5":
        for key, name in (("arm_points", "points"), ("arm_branches", "branches"),
                          ("arm_predictions", "predictions"), ("arm_target", "target")):
            value = getattr(laboratory, name)
            metadata[key] = value.tolist() if hasattr(value, "tolist") else deepcopy(value)
    goals = (tuple(session.controller.goals) if session.controller is not None else
             ((*laboratory.target, 0.0),) if laboratory is not None and session.lesson == "4"
             and laboratory.has_goal else ())
    return deepcopy({
        "lesson": session.lesson, "task": session.task,
        "title": f"{LESSONS[session.lesson]} · {TASKS[session.lesson][session.task]}",
        "columns": tuple(session.columns), "samples": tuple(tuple(row) for row in session.samples),
        "sensor_columns": tuple(session.sensors.columns) if session.sensors is not None else (),
        "sensor_samples": tuple(tuple(row) for row in session.sensors.rows)
                          if session.sensors is not None else (),
        "goals": goals, "outcome": session.outcome, "metadata": metadata,
    })
