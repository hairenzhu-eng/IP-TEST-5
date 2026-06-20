import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from laptop import (  # noqa: E402
    LaptopController,
    classic_apf_repulsive_gradient_weight,
    cluster_principal_dimensions,
    ellipse_level_and_away,
    linear_field_weight,
)


class ApfGeometryTest(unittest.TestCase):
    def test_pc1_pc2_follow_cluster_principal_axes(self):
        points = np.array(
            [
                [-2.0, -0.5],
                [-2.0, 0.5],
                [2.0, -0.5],
                [2.0, 0.5],
            ]
        )
        centre, pc1_m, pc2_m, length_axis = cluster_principal_dimensions(
            points,
            0.1,
            0.1,
        )
        np.testing.assert_allclose(centre, [0.0, 0.0])
        self.assertAlmostEqual(pc1_m, 4.0)
        self.assertAlmostEqual(pc2_m, 1.0)
        self.assertAlmostEqual(abs(float(length_axis[0])), 1.0)

    def test_repulsion_weight_decreases_linearly_to_field_edge(self):
        centre_level, _ = ellipse_level_and_away(
            [0.0, 0.0],
            [1.0, 0.0],
            4.0,
            2.0,
        )
        halfway_level, _ = ellipse_level_and_away(
            [2.0, 0.0],
            [1.0, 0.0],
            4.0,
            2.0,
        )
        edge_level, _ = ellipse_level_and_away(
            [4.0, 0.0],
            [1.0, 0.0],
            4.0,
            2.0,
        )
        self.assertAlmostEqual(linear_field_weight(centre_level), 1.0)
        self.assertAlmostEqual(linear_field_weight(halfway_level), 0.5)
        self.assertAlmostEqual(linear_field_weight(edge_level), 0.0)

    def test_classic_apf_gradient_is_zero_outside_qstar(self):
        self.assertGreater(classic_apf_repulsive_gradient_weight(1.0, 3.0), 0.0)
        self.assertAlmostEqual(
            classic_apf_repulsive_gradient_weight(3.0, 3.0),
            0.0,
        )
        self.assertAlmostEqual(
            classic_apf_repulsive_gradient_weight(3.1, 3.0),
            0.0,
        )

    def test_classic_range_ignores_cluster_dimensions(self):
        controller = LaptopController.__new__(LaptopController)
        controller.apf_cluster_range_enabled = False
        controller.apf_classic_influence_distance_m = 3.0
        controller.apf_own_equivalent_radius_m = 0.25
        controller.obstacle_min_pc1_m = 0.3
        controller.obstacle_min_pc2_m = 0.16
        controller.p_robot = np.array([[0.0], [0.0], [0.0]])
        obstacle = {
            "centre_body": [1.5, 0.0],
            "pc1_m": 100.0,
            "pc2_m": 50.0,
            "length_axis_ne": [1.0, 0.0],
        }

        level, away = controller.apf_obstacle_level_and_away(obstacle, 20.0)

        self.assertAlmostEqual(level, 0.5)
        np.testing.assert_allclose(away, [-1.0, 0.0])
        self.assertAlmostEqual(controller.apf_direction_clearance_m(obstacle), 3.0)

    def test_prediction_ends_at_no_apf_collision_time(self):
        controller = LaptopController.__new__(LaptopController)
        controller.obstacle_ekf_prediction_enabled = True
        controller.latest_lidar_received_s = 0.0
        controller.North = 0.0
        controller.East = 0.0
        controller.p_robot = np.array([[0.0], [0.0], [0.0]])
        controller.apf_virtual_obstacles = []
        controller.apf_track_timeout_s = 1.0
        controller.apf_collision_horizon_s = 10.0
        controller.apf_risk_pc_scale = 2.5
        controller.apf_own_equivalent_radius_m = 0.1
        controller.obstacle_min_pc1_m = 0.2
        controller.obstacle_min_pc2_m = 0.1
        controller.obstacle_prediction_step_s = 0.5
        controller.obstacle_prediction_accel_min_samples = 10
        controller.apf_activation_front_half_angle_rad = np.deg2rad(150.0)
        controller.apf_dynamic_speed_threshold_m_s = 0.05
        controller.own_prediction_velocity_ne = lambda: np.array([1.0, 0.0])
        controller.obstacle_track_motion_is_stable = lambda track: True

        track = {
            "id": 1,
            "state": np.array([5.0, 5.0, 0.0, -1.0]),
            "pos_ne": np.array([5.0, 5.0]),
            "vel_ne": np.array([0.0, -1.0]),
            "velocity_mean_ne": np.array([0.0, -1.0]),
            "heading_axis_ne": np.array([0.0, -1.0]),
            "length_axis_ne": np.array([0.0, -1.0]),
            "pc1_m": 1.0,
            "pc2_m": 0.4,
            "pc1_mean_m": 1.0,
            "pc2_mean_m": 0.4,
            "stats_sample_count": 3,
            "last_seen_s": 0.0,
            "stamp_s": 0.0,
            "lidar_history_ne": [np.array([5.0, 5.0])],
            "prediction_model": "ekf_constant_velocity",
        }
        controller.apf_obstacle_tracks = [track]

        virtual = controller.update_apf_virtual_obstacles()

        self.assertEqual(len(virtual), 1)
        self.assertAlmostEqual(track["collision_time_s"], 5.0)
        np.testing.assert_allclose(virtual[0]["segment_start_ne"], [5.0, 5.0])
        np.testing.assert_allclose(virtual[0]["segment_end_ne"], [5.0, 0.0])
        np.testing.assert_allclose(track["prediction_ne"][-1], [5.0, 0.0])

    def test_colreg_classification_sets_avoidance_side(self):
        controller = LaptopController.__new__(LaptopController)
        controller.apf_dynamic_speed_threshold_m_s = 0.05

        encounter, side, _ = controller.apf_classify_encounter(
            [5.0, 0.0],
            [-1.0, 0.0],
            [1.0, 0.0],
        )
        self.assertEqual((encounter, side), ("head_on", -1.0))

        encounter, side, _ = controller.apf_classify_encounter(
            [5.0, -5.0],
            [0.0, 1.0],
            [1.0, 0.0],
        )
        self.assertEqual((encounter, side), ("crossing_from_starboard", -1.0))

        encounter, side, _ = controller.apf_classify_encounter(
            [5.0, 5.0],
            [0.0, -1.0],
            [1.0, 0.0],
        )
        self.assertEqual((encounter, side), ("crossing_from_port", 0.0))


if __name__ == "__main__":
    unittest.main()
