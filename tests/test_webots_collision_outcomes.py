import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / "src" / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


switch_plot = load_module("switch_plot", "plot_switch_combination_trajectories.py")
distance_plot = load_module("distance_plot", "plot_three_strategy_distance.py")
postprocess_plot = load_module("postprocess_plot", "plot_obstacle_postprocess.py")


def test_collision_outcomes_use_webots_sensor_record(tmp_path):
    path = tmp_path / "webots_collision.json"
    path.write_text(
        json.dumps(
            {
                "source": "webots_touch_sensor",
                "obstacle_model": "ShipObstacle",
                "sensor_available": True,
                "detected": True,
                "first_contact_time_s": 3.25,
            }
        ),
        encoding="utf-8",
    )

    assert switch_plot.read_avoidance_outcome(tmp_path) is False
    assert distance_plot.read_collision_detected(tmp_path) is True


def test_missing_sensor_record_is_not_inferred_from_geometry(tmp_path):
    assert switch_plot.read_avoidance_outcome(tmp_path) is None


def test_trajectory_stops_at_first_arrival(tmp_path):
    log_path = tmp_path / "log_test.csv"
    log_path.write_text(
        "TimeFromStart(s),North(m),East(m),NavigationMode\n"
        "0,0,1,track\n"
        "1,15,1,arrived\n"
        "2,5e248,3e246,arrived\n",
        encoding="utf-8",
    )

    _, positions, _ = switch_plot.read_trajectory_series(log_path)

    assert positions.tolist() == [[0.0, 1.0], [15.0, 1.0]]


def test_environment_comparison_rejects_different_worlds():
    left = SimpleNamespace(webots_environment="mr_webots_overtaking_large_ship")
    right = SimpleNamespace(webots_environment="mr_webots_overtaking_small_ship")

    assert switch_plot.compare_environments(left, right, 10.0) is None


def test_distance_comparison_labels_come_from_webots_sensor(tmp_path):
    runs = []
    for name, detected in (("prediction", False), ("no_a", False), ("no_b", True)):
        run_dir = tmp_path / name
        run_dir.mkdir()
        (run_dir / "log_test.csv").write_text(
            "TimeFromStart(s),North(m),East(m),"
            "NearestObstacleNorth(m),NearestObstacleEast(m)\n"
            "0,0,0,1,0\n1,0.2,0,1,0\n",
            encoding="utf-8",
        )
        (run_dir / "webots_collision.json").write_text(
            json.dumps(
                {
                    "source": "webots_touch_sensor",
                    "obstacle_model": "ShipObstacle",
                    "sensor_available": True,
                    "detected": detected,
                    "first_contact_time_s": 0.5 if detected else None,
                }
            ),
            encoding="utf-8",
        )
        runs.append(run_dir)

    rows = postprocess_plot.plot_distance_comparison(
        prediction_source=runs[0],
        no_prediction_source_a=runs[1],
        no_prediction_source_b=runs[2],
        output_path=tmp_path / "comparison.png",
        safe_distance_m=1.0,
    )

    assert [row["collision_detected"] for row in rows] == [True, False, False]
    assert "collision detected" in rows[0]["label"]
    assert all("succeeded" not in row["label"] and "failed" not in row["label"] for row in rows)
