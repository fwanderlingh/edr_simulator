import csv
from pathlib import Path
import subprocess
import sys

import pytest


def test_headless_entry_point_and_csv(tmp_path):
    destination = tmp_path / "nested" / "poses.csv"
    result = subprocess.run(
        [sys.executable, "-m", "robotics_sim", "--lesson", "4", "--headless",
         "--duration", "0.1", "--dt", "0.01", "--csv", str(destination)],
        capture_output=True, text=True, timeout=30, check=True,
        cwd=Path(__file__).resolve().parents[1],
    )
    assert "Completed 10 steps" in result.stdout
    with destination.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 10
    assert float(rows[-1]["time_s"]) == pytest.approx(0.1)
    assert float(rows[-1]["x_m"]) == pytest.approx(0.015, abs=1e-6)
    assert destination.with_name("poses_sensors.csv").exists()
