import csv
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from plot_obstacle_speed_comparison import (  # noqa: E402
    collect_runs,
    plot_obstacle_speed_comparisons,
)


FIELDS = [
    "TimeFromStart(s)",
    "North(m)",
    "East(m)",
    "NearestObstacleNorth(m)",
    "NearestObstacleEast(m)",
    "WebotsEnvironment",
    "SwitchCombination",
]


def write_run(logs_dir, name, speed, switch="ekf_on_cluster_on"):
    run_dir = logs_dir / name
    run_dir.mkdir()
    log_path = run_dir / f"log_{name.removeprefix('run_')}.csv"
    with log_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for time_s in (10.0, 11.0, 12.0):
            writer.writerow(
                {
                    "TimeFromStart(s)": time_s,
                    "North(m)": time_s - 10.0,
                    "East(m)": 0.0,
                    "NearestObstacleNorth(m)": 4.0,
                    "NearestObstacleEast(m)": speed,
                    "WebotsEnvironment": (
                        f"mr_webots_overtaking_small_ship_0_{int(speed * 10)}"
                        "_m_s.wbt"
                    ),
                    "SwitchCombination": switch,
                }
            )
    return run_dir


def test_collects_only_ekf_cluster_on_and_plots_combined_figure(tmp_path):
    logs_dir = tmp_path / "logs"
    logs_dir.mkdir()
    write_run(logs_dir, "run_20260101_000001", 0.2)
    write_run(logs_dir, "run_20260101_000002", 0.4)
    write_run(
        logs_dir,
        "run_20260101_000003",
        0.6,
        switch="ekf_on_cluster_off",
    )

    groups = collect_runs(logs_dir)
    records = groups["mr_webots_overtaking_small_ship"]
    assert [record.obstacle_speed_m_s for record in records] == [0.2, 0.4]
    assert np.allclose(records[0].time_s, [0.0, 1.0, 2.0])

    outputs = plot_obstacle_speed_comparisons(logs_dir, tmp_path / "figures")
    assert len(outputs) == 1
    assert outputs[0][0].is_file()
