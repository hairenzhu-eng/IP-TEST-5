import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from plot_cluster_pc_dimensions import (  # noqa: E402
    load_cluster_dimensions,
    plot_cluster_dimensions,
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


if __name__ == "__main__":
    unittest.main()
