"""Reload a lesson's saved source and the adapters that import its exercises."""

import importlib
from pathlib import Path
import tokenize


# Dependencies precede their importers, including the waypoint/pose relationship.
LESSON_MODULES = {
    "1": ("exercises.lesson01.straight", "exercises.lesson01.circle", "exercises.lesson01.square"),
    "2": ("exercises.lesson02.frames", "exercises.lesson02.planar", "exercises.lesson02.spatial",
          "exercises.lesson02.rpy", "exercises.lesson02.quaternions", "exercises.lesson02.module2",
          "exercises.lesson02.activity"),
    "3": ("exercises.lesson03._control_runtime", "exercises.lesson03._proportional_runtime",
          "exercises.lesson03.kinematics", "exercises.lesson03.proportional",
          "exercises.lesson03.control", "exercises.lesson03.waypoint",
          "sim.references.control", "sim.references.proportional_control",
          "sim.references.waypoint_control", "exercises.lesson03.activity"),
    "4": ("exercises.lesson04._runtime", "sim.references.sensors",
          "exercises.lesson04.sensors.encoder", "exercises.lesson04.sensors.imu",
          "exercises.lesson04.sensors.lidar", "exercises.lesson04.lidar",
          "exercises.lesson04.sensors.suite", "exercises.lesson04.estimation",
          "exercises.lesson04.rangefinder", "exercises.lesson04.avoidance",
          "exercises.lesson04.gui", "exercises.lesson04.activity"),
    "5": ("exercises.lesson05.kinematics", "exercises.lesson05.control", "exercises.lesson05.gui"),
    "8": ("exercises.lesson08.activity",),
}

# Lesson 4's nominal controller is the saved Lesson 3 exercise, not a duplicate.
LESSON_MODULES["4"] = LESSON_MODULES["3"] + LESSON_MODULES["4"]


def reload_lesson(lesson):
    """Reload saved Python files, restoring the previous modules if loading fails.

    Read and compile source directly: a rapid edit can preserve both the file
    size and its timestamp in Python's bytecode cache. Retain module identities
    so existing imports of module objects continue to see the new definitions.
    """
    if lesson not in LESSON_MODULES:
        raise ValueError("Unknown lesson.")
    importlib.invalidate_caches()
    modules = [importlib.import_module(f"robotics_sim.{name}") for name in LESSON_MODULES[lesson]]
    compiled = []
    for module in modules:
        path = Path(module.__file__)
        with tokenize.open(path) as stream:
            compiled.append(compile(stream.read(), str(path), "exec"))
    previous = [dict(module.__dict__) for module in modules]
    metadata = ("__name__", "__package__", "__file__", "__cached__", "__loader__", "__spec__")
    try:
        for module, code, saved in zip(modules, compiled, previous):
            # Clearing the namespace also removes globals deleted by the student.
            module.__dict__.clear()
            module.__dict__.update({key: saved[key] for key in metadata if key in saved})
            module.__dict__["__doc__"] = None
            exec(code, module.__dict__)
    except Exception:
        for module, saved in zip(modules, previous):
            module.__dict__.clear()
            module.__dict__.update(saved)
        raise
