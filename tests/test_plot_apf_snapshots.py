import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from plot_apf_snapshots import (  # noqa: E402
    attractive_potential,
    classic_repulsive_potential,
    ellipse_potential,
    obstacle_ellipse_geometry,
    run_bounds,
    segment_potential,
    select_snapshot_indices,
)


class ApfSnapshotTest(unittest.TestCase):
    def test_selects_nearest_snapshot_every_five_seconds(self):
        snapshots = [
            {"relative_time_s": value}
            for value in (0.0, 1.0, 4.9, 5.2, 9.8, 10.3)
        ]
        self.assertEqual(select_snapshot_indices(snapshots, 5.0), [0, 2, 4])

    def test_attractive_potential_matches_notebook_equation(self):
        north = np.array([[0.0, 2.0]])
        east = np.array([[0.0, 2.0]])
        potential = attractive_potential(north, east, [0.0, 0.0], 1.0)
        np.testing.assert_allclose(potential, [[0.0, 4.0]])

    def test_ellipse_potential_uses_notebook_exponential_shape(self):
        north = np.array([[0.0, 2.25]])
        east = np.zeros_like(north)
        obstacle = {
            "centre_ne": [0.0, 0.0],
            "pc1_m": 2.0,
            "pc2_m": 1.0,
            "length_axis_ne": [1.0, 0.0],
        }
        settings = {
            "avoidance_pc_scale": 2.0,
            "own_equivalent_radius_m": 0.25,
        }
        potential = ellipse_potential(
            north,
            east,
            obstacle,
            settings,
            k_obstacle=150.0,
        )
        self.assertAlmostEqual(float(potential[0, 0]), 150.0)
        self.assertAlmostEqual(float(potential[0, 1]), 150.0 / np.e)

    def test_virtual_segment_contributes_potential(self):
        north = np.array([[0.0, 0.0]])
        east = np.array([[1.0, 3.0]])
        obstacle = {
            "segment_start_ne": [0.0, 0.0],
            "segment_end_ne": [0.0, 2.0],
            "pc1_m": 0.2,
            "pc2_m": 0.5,
        }
        settings = {
            "virtual_pc_scale": 2.0,
            "own_equivalent_radius_m": 0.0,
        }
        potential = segment_potential(north, east, obstacle, settings)
        self.assertAlmostEqual(float(potential[0, 0]), 150.0)
        self.assertLess(float(potential[0, 1]), 1.0)

    def test_classic_potential_is_truncated_at_qstar(self):
        potential = classic_repulsive_potential(
            np.array([[1.0, 3.0, 3.1]]),
            3.0,
            150.0,
        )
        self.assertGreater(float(potential[0, 0]), 0.0)
        self.assertAlmostEqual(float(potential[0, 1]), 0.0)
        self.assertAlmostEqual(float(potential[0, 2]), 0.0)

    def test_classic_snapshot_field_ignores_cluster_dimensions(self):
        north = np.array([[0.0, 0.0]])
        east = np.array([[1.0, 3.1]])
        obstacle = {
            "centre_ne": [0.0, 0.0],
            "pc1_m": 100.0,
            "pc2_m": 50.0,
            "length_axis_ne": [1.0, 0.0],
        }
        settings = {
            "cluster_range_enabled": False,
            "classic_influence_distance_m": 3.0,
        }
        potential = ellipse_potential(
            north,
            east,
            obstacle,
            settings,
            k_obstacle=150.0,
        )
        self.assertGreater(float(potential[0, 0]), 0.0)
        self.assertAlmostEqual(float(potential[0, 1]), 0.0)

    def test_cluster_size_ellipse_uses_pc1_pc2_and_e_frame_axis(self):
        geometry = obstacle_ellipse_geometry(
            {
                "centre_ne": [4.0, 2.0],
                "pc1_m": 1.8,
                "pc2_m": 0.5,
                "length_axis_ne": [1.0, 0.0],
            },
            {},
        )
        centre, pc1_m, pc2_m, angle_deg = geometry
        np.testing.assert_allclose(centre, [4.0, 2.0])
        self.assertAlmostEqual(pc1_m, 1.8)
        self.assertAlmostEqual(pc2_m, 0.5)
        self.assertAlmostEqual(angle_deg, 90.0)

    def test_map_extent_is_fixed_twenty_by_twenty_metres(self):
        snapshots = [
            {"payload": {"robot_pos": [0.0, 0.0]}},
            {"payload": {"robot_pos": [10.0, 2.0]}},
        ]
        bounds = run_bounds(snapshots, 20.0)
        self.assertAlmostEqual(bounds[1] - bounds[0], 20.0)
        self.assertAlmostEqual(bounds[3] - bounds[2], 20.0)

    def test_map_extent_stays_twenty_metres_with_long_prediction(self):
        snapshots = [
            {
                "payload": {
                    "robot_pos": [0.0, 0.0],
                    "tracks": [
                        {
                            "position_ne": [0.0, 0.0],
                            "prediction_ne": [[0.0, 0.0], [30.0, 0.0]],
                        }
                    ],
                }
            }
        ]
        bounds = run_bounds(snapshots, 20.0)
        self.assertAlmostEqual(bounds[1] - bounds[0], 20.0)
        self.assertAlmostEqual(bounds[3] - bounds[2], 20.0)


if __name__ == "__main__":
    unittest.main()
