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

    def test_prediction_runs_to_30_seconds_independent_of_collision_time(self):
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
        controller.obstacle_prediction_horizon_s = 30.0
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
        np.testing.assert_allclose(track["prediction_ne"][-1], [5.0, -25.0])

    def test_static_cluster_world_centre_does_not_inherit_ownship_motion(self):
        controller = LaptopController.__new__(LaptopController)
        controller.lidar_dbscan_eps_m = 0.3
        controller.lidar_dbscan_min_points = 3
        controller.lidar_x_bl = 0.0
        controller.lidar_y_bl = 0.0
        controller.lidar_gamma_bl = 0.0
        controller.obstacle_min_pc1_m = 0.1
        controller.obstacle_min_pc2_m = 0.1
        controller.obstacle_ekf_accel_std_m_s2 = 0.2
        controller.obstacle_ekf_measurement_std_m = 0.08

        cluster_ne = np.array(
            [
                [4.95, 0.95],
                [5.05, 0.95],
                [5.00, 1.05],
            ]
        )

        def observe_from_pose(pose):
            controller.p_robot = np.asarray(pose, dtype=float).reshape(3, 1)
            c = np.cos(pose[2])
            s = np.sin(pose[2])
            points_body = (cluster_ne - np.asarray(pose[:2])) @ np.array(
                [[c, -s], [s, c]]
            )
            controller.lidar_data_rb = np.column_stack(
                [
                    np.linalg.norm(points_body, axis=1),
                    np.arctan2(points_body[:, 1], points_body[:, 0]),
                ]
            )
            controller.update_lidar_obstacle_clusters()
            return np.asarray(controller.lidar_obstacles[0]["centre_ne"])

        first_centre_ne = observe_from_pose([0.0, 0.0, 0.0])
        second_centre_ne = observe_from_pose([1.0, 0.5, np.deg2rad(35.0)])

        np.testing.assert_allclose(first_centre_ne, np.mean(cluster_ne, axis=0))
        np.testing.assert_allclose(second_centre_ne, first_centre_ne)

        state = np.r_[first_centre_ne, 0.0, 0.0]
        covariance = np.eye(4, dtype=float)
        predicted_state, predicted_covariance = controller.obstacle_ekf_predict(
            state,
            covariance,
            1.0,
        )
        corrected_state, _ = controller.obstacle_ekf_update(
            predicted_state,
            predicted_covariance,
            second_centre_ne,
        )
        np.testing.assert_allclose(corrected_state[2:4], [0.0, 0.0], atol=1e-12)

    def test_turning_noise_grows_with_range_and_yaw_rate(self):
        controller = LaptopController.__new__(LaptopController)
        controller.obstacle_ekf_measurement_std_m = 0.08
        controller.lidar_time_sync_std_s = 0.02
        controller.lidar_scan_time_s = 0.1
        controller.lidar_reference_pose = np.zeros(3)
        controller.p_robot = np.zeros((3, 1))
        controller.v_robot = np.zeros((3, 1))
        controller.Sigma = np.zeros((6, 6))
        controller.Sigma[2, 2] = np.deg2rad(2.0) ** 2
        controller.sensed_imu_yaw_rate_rad_s = 0.0

        near_covariance = controller.obstacle_measurement_covariance([1.0, 0.0])
        far_covariance = controller.obstacle_measurement_covariance([5.0, 0.0])
        controller.sensed_imu_yaw_rate_rad_s = 1.0
        turning_covariance = controller.obstacle_measurement_covariance([5.0, 0.0])

        self.assertGreater(far_covariance[1, 1], near_covariance[1, 1])
        self.assertGreater(turning_covariance[1, 1], far_covariance[1, 1])
        self.assertAlmostEqual(turning_covariance[0, 0], 0.08 ** 2)

    def test_track_heading_uses_ekf_velocity_and_holds_at_low_speed(self):
        controller = LaptopController.__new__(LaptopController)
        controller.obstacle_min_pc1_m = 0.1
        controller.obstacle_min_pc2_m = 0.1
        controller.obstacle_heading_hold_speed_m_s = 0.03
        controller.obstacle_ekf_prediction_enabled = False
        track = {
            "state": np.array([0.0, 0.0, 1.0, 1.0]),
            "velocity_mean_ne": np.array([0.0, -5.0]),
            "pc1_m": 1.0,
            "pc2_m": 0.4,
            "length_axis_ne": np.array([1.0, 0.0]),
        }

        controller.sync_obstacle_track_fields(track)
        self.assertAlmostEqual(track["heading_rad"], np.pi / 4.0)

        track["state"][2:4] = [0.001, -0.001]
        controller.sync_obstacle_track_fields(track)
        self.assertAlmostEqual(track["heading_rad"], np.pi / 4.0)

    def test_lidar_motion_compensation_uses_beam_time_and_extrinsics(self):
        controller = LaptopController.__new__(LaptopController)
        controller.lidar_dbscan_eps_m = 0.3
        controller.lidar_dbscan_min_points = 3
        controller.lidar_x_bl = 0.4
        controller.lidar_y_bl = -0.2
        controller.lidar_gamma_bl = np.deg2rad(8.0)
        controller.lidar_pose_extrapolation_limit_s = 1.0
        controller.lidar_time_sync_std_s = 0.01
        controller.lidar_scan_time_s = 0.2
        controller.lidar_timestamp_s = 0.0
        controller.last_nav_t = 0.0
        controller.p_robot = np.zeros((3, 1))
        controller.v_robot = np.array([[0.0], [0.0], [1.0]])
        controller.sensed_imu_yaw_rate_rad_s = 1.0
        controller.Sigma = np.zeros((6, 6))
        controller.obstacle_min_pc1_m = 0.1
        controller.obstacle_min_pc2_m = 0.1
        controller.obstacle_ekf_measurement_std_m = 0.08

        cluster_ne = np.array(
            [
                [4.95, 0.95],
                [5.05, 0.95],
                [5.00, 1.05],
            ]
        )
        stamps_s = np.array([0.0, 0.1, 0.2])
        measurements = []
        lidar_offset = np.array([controller.lidar_x_bl, controller.lidar_y_bl])
        for point_ne, stamp_s in zip(cluster_ne, stamps_s):
            yaw = stamp_s
            rotation = np.array(
                [
                    [np.cos(yaw), -np.sin(yaw)],
                    [np.sin(yaw), np.cos(yaw)],
                ]
            )
            lidar_origin_ne = rotation @ lidar_offset
            sensor_yaw = yaw + controller.lidar_gamma_bl
            sensor_rotation = np.array(
                [
                    [np.cos(sensor_yaw), -np.sin(sensor_yaw)],
                    [np.sin(sensor_yaw), np.cos(sensor_yaw)],
                ]
            )
            point_lidar = sensor_rotation.T @ (point_ne - lidar_origin_ne)
            measurements.append(
                [np.linalg.norm(point_lidar), np.arctan2(point_lidar[1], point_lidar[0])]
            )

        controller.lidar_data_rb = np.asarray(measurements)
        controller.lidar_beam_stamps_s = stamps_s
        controller.update_lidar_obstacle_clusters()

        np.testing.assert_allclose(
            controller.lidar_obstacles[0]["centre_ne"],
            np.mean(cluster_ne, axis=0),
            atol=1e-12,
        )

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
