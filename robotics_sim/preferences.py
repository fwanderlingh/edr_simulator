"""Small, validated, atomically written preferences file beside the application."""

import json
import math
import os
from pathlib import Path
import tempfile


DEFAULT_PATH = Path(__file__).resolve().parents[1] / "settings.json"


def load_preferences(path):
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    except (OSError, ValueError):
        return {}
    if not isinstance(data, dict):
        return {}
    result = {}
    for key in ("lesson", "task", "kinematics", "sensor_mode", "label_color", "label_background_color"):
        if isinstance(data.get(key), str):
            result[key] = data[key]
    for key in ("show_frames", "label_background", "maximized"):
        if isinstance(data.get(key), bool):
            result[key] = data[key]
    ranges = {"label_size": (8, 36), "label_background_transparency": (0, 100), "duration": (0.001, 86400),
              "camera_yaw": (-1e9, 1e9), "camera_pitch": (-89, -5), "camera_distance": (1.5, 14),
              "width": (900, 20000), "height": (620, 20000), "x": (-20000, 20000), "y": (-20000, 20000)}
    for key, (low, high) in ranges.items():
        value = data.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            continue
        if key in ("label_size", "width", "height", "x", "y") and int(value) != value:
            continue
        result[key] = value
    return result


def save_preferences(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=path.name + ".", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(data, stream, indent=2, allow_nan=False)
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
