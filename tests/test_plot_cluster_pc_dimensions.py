import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from plot_cluster_pc_dimensions import (  # noqa: E402
    load_cluster_dimensions,
    plot_cluster_dimensions,
    plot_large_small_pc_pair,
)


class ClusterPcDimensionPlotTest(unittest.TestCase):
    def test_loads_cluster_dimensions_and_uses_relative_time(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            snapshots = [
                {
                    "t": 10.0,
                    "clusters": [
                        {"track_id": 1, "pc1_m": 1.0, "pc2_m": 0.4}
                    ],
                },
                {
                    "t": 12.5,
                    "clusters": [
                        {"track_id": 1, "pc1_m": 1.2, "pc2_m": 0.5}
                    ],
                },
            ]
            for index, payload in enumerate(snapshots):
                path = run_dir / f"obstacle_{index}.json"
                path.write_text(json.dumps(payload), encoding="utf-8")

            grouped = load_cluster_dimensions(run_dir)

            self.assertEqual(list(grouped), ["1"])
            self.assertEqual(
                [sample["time_s"] for sample in grouped["1"]],
                [0.0, 2.5],
            )

    def test_plot_reports_means_and_creates_png(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            payload = {
                "t": 3.0,
                "clusters": [
                    {"track_id": 4, "pc1_m": 1.4, "pc2_m": 0.6}
                ],
            }
            (run_dir / "obstacle_1.json").write_text(
                json.dumps(payload),
                encoding="utf-8",
            )
            output_path = run_dir / "pc_dimensions.png"

            output, pc1_mean, pc2_mean, count = plot_cluster_dimensions(
                run_dir,
                output_path,
            )

            self.assertEqual(output, output_path)
            self.assertTrue(output_path.is_file())
            self.assertAlmostEqual(pc1_mean, 1.4)
            self.assertAlmostEqual(pc2_mean, 0.6)
            self.assertEqual(count, 1)

    def test_plots_large_and_small_pc_dimensions_together(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            records = []
            for size_label, pc1_m, pc2_m in (
                ("Small", 0.4, 0.2),
                ("Large", 0.8, 0.3),
            ):
                run_dir = root / size_label.lower()
                run_dir.mkdir()
                for index, time_s in enumerate((10.0, 12.0)):
                    payload = {
                        "t": time_s,
                        "clusters": [
                            {
                                "track_id": 1,
                                "pc1_m": pc1_m,
                                "pc2_m": pc2_m,
                                "centre_ne": [1.0, 1.0],
                            }
                        ],
                        "robot_pos": [0.0, 0.0],
                    }
                    (run_dir / f"obstacle_{index}.json").write_text(
                        json.dumps(payload), encoding="utf-8"
                    )
                records.append(
                    SimpleNamespace(
                        run_dir=run_dir,
                        size_label=size_label,
                        webots_pair_key="mr_webots_head_on_size_ship",
                    )
                )

            output_path = root / "large_small_pc1_pc2.png"

            output = plot_large_small_pc_pair(records, output_path)

            self.assertEqual(output, output_path)
            self.assertTrue(output_path.is_file())


if __name__ == "__main__":
    unittest.main()
