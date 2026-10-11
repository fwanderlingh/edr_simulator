"""Run from simulator/: .conda/python.exe tools/benchmark_viewport.py.

Reports completed Canvas paint time, including geometry projection. Use
--compare-raster to measure the previous full-resolution image path as well.
Requires a desktop (or Xvfb); no fixed timing assertions belong in unit tests.
"""

import argparse
import json
from pathlib import Path
import statistics
import sys
import time
import tkinter as tk

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robotics_sim.app import enable_high_dpi
from robotics_sim.session import ActivitySession
from robotics_sim.sim.camera import Camera
from robotics_sim.sim.scene_renderer import SceneRenderer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compare-raster", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    enable_high_dpi()
    root = tk.Tk()
    root.title("Viewport performance check")
    canvas = tk.Canvas(root, highlightthickness=0)
    canvas.pack(fill="both", expand=True)
    session = ActivitySession(duration=600)
    session.start()
    camera, renderer = Camera(), SceneRenderer(canvas)
    results = []
    try:
        for width, height in ((640, 360), (1280, 720), (1920, 1080), (2560, 1440)):
            root.geometry(f"{width}x{height}")
            root.update()
            width, height = canvas.winfo_width(), canvas.winfo_height()
            times = []
            for frame in range(35):
                session.step()
                start = time.perf_counter()
                renderer.draw(session.world, camera, width, height)
                root.update_idletasks()
                if frame >= 5:
                    times.append((time.perf_counter() - start) * 1000)
            result = {"viewport": [width, height], "retained_median_ms": round(statistics.median(times), 3),
                      "retained_p95_ms": round(sorted(times)[int(len(times) * 0.95)], 3)}
            if args.compare_raster:
                times = []
                image_item = canvas.create_image(0, 0, anchor="nw")
                for _ in range(3):
                    start = time.perf_counter()
                    photo = tk.PhotoImage(data=camera.render(session.world, width, height), format="PPM")
                    canvas.itemconfigure(image_item, image=photo)
                    root.update_idletasks()
                    times.append((time.perf_counter() - start) * 1000)
                result["raster_median_ms"] = round(statistics.median(times), 3)
                canvas.delete(image_item)
            results.append(result)
            print(json.dumps(result), flush=True)
    finally:
        session.close()
        root.destroy()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
