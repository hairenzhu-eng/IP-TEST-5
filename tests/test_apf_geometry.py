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
    generate_papf_trajectory,
    linear_field_weight,
    papf_attractive_vector,
    papf_repulsive_vector,
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

    def test_papf_attraction_is_saturated_at_katt(self):
        np.testing.assert_allclose(
            papf_attractive_vector([0.0, 0.0], [3.0, 4.0], 2.0),
            [1.2, 1.6],
        )

    def test_papf_repulsion_uses_strongest_future_prediction(self):
        original = np.column_stack([np.arange(5, dtype=float), np.zeros(5)])
        obstacle = np.array(
            [
                [10.0, 10.0],
                [10.0, 10.0],
                [2.0, 0.5],
                [10.0, 10.0],
                [10.0, 10.0],
            ]
        )

        repulsion = papf_repulsive_vector(
            1,
            original,
            obstacle,
            k_rep=2.0,
            r_min_m=1.0,
            r_max_m=1.0,
        )

        np.testing.assert_allclose(
            repulsion,
            [0.0, -np.exp(-0.25)],
            atol=1e-12,
        )

    def test_papf_trajectory_avoids_future_obstacle_without_virtual_field(self):
        reference = np.column_stack([np.arange(5, dtype=float), np.zeros(5)])
        obstacle = np.array(
            [
                [10.0, 10.0],
                [10.0, 10.0],
                [2.0, 0.5],
                [10.0, 10.0],
                [10.0, 10.0],
            ]
        )

        trajectory, _, first_repulsive = generate_papf_trajectory(
            reference,
            [{
                "trajectory_ne": obstacle,
                "r_min_m": 1.0,
                "r_max_m": 1.0,
            }],
            k_att=1.0,
            k_rep=2.0,
            buffer_size=3,
            max_iterations=8,
            convergence_m=1e-6,
        )

        self.assertLess(first_repulsive[1], 0.0)
        self.assertLess(trajectory[1, 1], 0.0)

    def test_papf_obstacle_trajectory_uses_ekf_velocity_and_direction(self):
        controller = LaptopController.__new__(LaptopController)
        controller.apf_prediction_dt_s = 0.5
        controller.latest_lidar_received_s = 0.0
        controller.apf_track_timeout_s = 1.0
        controller.obstacle_ekf_prediction_enabled = True
        controller.obstacle_min_pc1_m = 0.2
        controller.obstacle_min_pc2_m = 0.1
        controller.apf_own_equivalent_radius_m = 0.1
        controller.apf_avoidance_pc_scale = 10.0
        track = {
            "id": 7,
            "pos_ne": np.array([1.0, 2.0]),
            "vel_ne": np.array([1.0, 2.0]),
            "pc1_m": 1.0,
            "pc2_m": 0.4,
            "last_seen_s": 0.0,
            "stamp_s": 0.0,
        }
        controller.apf_obstacle_tracks = [track]
        controller.lidar_obstacles = [{
            "track_id": 7,
            "centre_ne": [1.0, 2.0],
            "pc1_m": 1.0,
            "pc2_m": 0.4,
        }]
        controller.obstacle_track_motion_is_stable = lambda _: True
        controller.obstacle_track_state_at = lambda item, dt: (
            item["pos_ne"] + item["vel_ne"] * dt,
            item["vel_ne"],
        )

        trajectories = controller.papf_obstacle_trajectories(2)

        self.assertEqual(len(trajectories), 1)
        self.assertTrue(trajectories[0]["dynamic"])
        np.testing.assert_allclose(
            trajectories[0]["trajectory_ne"],
            [[1.0, 2.0], [1.5, 3.0], [2.0, 4.0]],
        )

    def test_robot_pose_at_lidar_time_interpolates_recorded_trajectory(self):
        controller = LaptopController.__new__(LaptopController)
        controller.robot_pose_history = [
            np.array([10.0, 1.0, 2.0, np.deg2rad(170.0)]),
            np.array([11.0, 3.0, 6.0, np.deg2rad(-170.0)]),
        ]
        controller.p_robot = np.array([[3.0], [6.0], [np.deg2rad(-170.0)]])
        controller.v_robot = np.zeros((3, 1))
        controller.last_nav_t = 11.0
        controller.lidar_pose_extrapolation_limit_s = 0.5

        pose = controller.robot_pose_at_time(10.5)

        np.testing.assert_allclose(pose[:2], [2.0, 4.0])
        self.assertAlmostEqual(abs(pose[2]), np.pi)

    def test_papf_keeps_original_avoidance_side_lock(self):
        controller = LaptopController.__new__(LaptopController)
        controller.lidar_obstacles = [{
            "centre_body": [2.0, 0.0],
            "centre_ne": [2.0, 0.0],
            "pc1_m": 1.0,
            "pc2_m": 0.4,
            "length_axis_ne": [1.0, 0.0],
        }]
        controller.apf_obstacle_tracks = []
        controller.obstacle_ekf_prediction_enabled = False
        controller.apf_activation_front_half_angle_rad = np.deg2rad(150.0)
        controller.apf_priority_front_half_angle_rad = np.deg2rad(90.0)
        controller.apf_direction_pc_scale = 10.0
        controller.apf_avoidance_pc_scale = 10.0
        controller.apf_dynamic_speed_threshold_m_s = 0.05
        controller.apf_cluster_range_enabled = True
        controller.apf_own_equivalent_radius_m = 0.1
        controller.obstacle_min_pc1_m = 0.2
        controller.obstacle_min_pc2_m = 0.1
        controller.apf_track_association_m = 0.5
        controller.lidar_dbscan_eps_m = 0.2
        controller.latest_lidar_received_s = 0.0
        controller.p_robot = np.zeros((3, 1))
        controller.current_velocity_body = lambda: np.array([1.0, 0.0])
        controller.left_clearance_m = 2.0
        controller.right_clearance_m = 1.0
        controller.timefromstart = 0.0
        controller.apf_side_lock_sign = 0.0
        controller.apf_side_lock_until_s = 0.0
        controller.apf_side_lock_s = 5.0
        controller.apf_side_lock_exit_level = 1.15
        controller.apf_side_lock_active = False
        controller.apf_encounter_mode = "none"
        controller.apf_colreg_rule = "none"
        controller.apf_avoidance_side_sign = 0.0
        controller.apf_colreg_dcpa_m = np.nan
        controller.apf_colreg_tcpa_s = np.nan
        controller.apf_colreg_active = False

        first_side = controller.papf_update_avoidance_side(True)
        controller.left_clearance_m = 0.5
        controller.right_clearance_m = 2.0
        controller.timefromstart = 1.0
        locked_side = controller.papf_update_avoidance_side(True)

        self.assertEqual(first_side, 1.0)
        self.assertEqual(locked_side, 1.0)

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

    def test_obstacle_pc_dimensions_do_not_floor_small_sizes(self):
        controller = LaptopController.__new__(LaptopController)
        controller.obstacle_min_pc1_m = 0.3
        controller.obstacle_min_pc2_m = 0.16

        pc1_m, pc2_m = controller.obstacle_pc_dimensions({
            "pc1_m": 0.08,
            "pc2_m": 0.03,
        })

        self.assertAlmostEqual(pc1_m, 0.08)
        self.assertAlmostEqual(pc2_m, 0.03)

    def test_crossing_field_expands_laterally(self):
        controller = LaptopController.__new__(LaptopController)
        controller.apf_cluster_range_enabled = True
        controller.apf_own_equivalent_radius_m = 0.0
        controller.obstacle_min_pc1_m = 0.3
        controller.obstacle_min_pc2_m = 0.16

        obstacle = {
            "pc1_m": 4.0,
            "pc2_m": 1.0,
        }
        offset = [0.0, 1.5]
        axis = [1.0, 0.0]

        static_level, _ = controller.apf_point_level_and_away(
            offset,
            obstacle,
            1.0,
            axis,
        )
        crossing_level, _ = controller.apf_point_level_and_away(
            offset,
            obstacle,
            1.0,
            axis,
            encounter="crossing_from_starboard",
        )

        self.assertGreater(static_level, 1.0)
        self.assertLess(crossing_level, 1.0)

    def test_crossing_strategy_forces_stern_when_obstacle_moves_right_to_left(self):
        controller = LaptopController.__new__(LaptopController)
        controller.apf_dynamic_speed_threshold_m_s = 0.05

        strategy, side = controller.apf_crossing_strategy_from_velocity([0.0, 0.5])

        self.assertEqual(strategy, "pass_astern")
        self.assertLess(side, 0.0)

    def test_crossing_strategy_forces_bow_when_obstacle_moves_left_to_right(self):
        controller = LaptopController.__new__(LaptopController)
        controller.apf_dynamic_speed_threshold_m_s = 0.05

        strategy, side = controller.apf_crossing_strategy_from_velocity([0.0, -0.5])

        self.assertEqual(strategy, "pass_ahead")
        self.assertLess(side, 0.0)

    def test_track_regularize_velocity_keeps_raw_ekf_velocity_before_stable(self):
        controller = LaptopController.__new__(LaptopController)
        controller.obstacle_ekf_initial_velocity_std_m_s = 1.0
        controller.obstacle_ekf_static_speed_reset_m_s = 0.05
        controller.obstacle_track_prediction_velocity_ne = lambda track: np.array([9.0, 9.0])

        track = {
            "state": np.array([0.0, 0.0, 0.4, -0.2], dtype=float),
            "covariance": np.eye(4, dtype=float),
            "motion_stable": False,
        }

        controller.obstacle_track_regularize_velocity(track)

        np.testing.assert_allclose(track["state"][2:4], [0.4, -0.2])

    def test_pass_astern_uses_obstacle_stern_for_right_to_left_crossing(self):
        controller = LaptopController.__new__(LaptopController)
        controller.apf_dynamic_speed_threshold_m_s = 0.05
        controller.apf_own_equivalent_radius_m = 0.25
        controller.obstacle_min_pc1_m = 0.3
        controller.obstacle_min_pc2_m = 0.16

        stern_dir = controller.apf_stern_direction_body(
            [5.0, -2.0],
            [0.0, 1.0],
            {"pc1_m": 4.0, "pc2_m": 1.0},
        )

        self.assertGreater(stern_dir[0], 0.0)
        self.assertLess(stern_dir[1], 0.0)

    def test_virtual_segment_risk_works_without_pc_dimensions(self):
        controller = LaptopController.__new__(LaptopController)
        controller.apf_cluster_range_enabled = True
        controller.apf_own_equivalent_radius_m = 0.25
        controller.obstacle_min_pc1_m = 0.3
        controller.obstacle_min_pc2_m = 0.16
        controller.p_robot = np.array([[0.0], [0.0], [0.0]])
        controller.v_robot = np.zeros((3, 1))

        risk_probe = {
            "virtual": True,
            "segment_start_ne": [0.1, 0.0],
            "segment_end_ne": [0.4, 0.0],
            "centre_ne": [0.4, 0.0],
            "centre_body": [0.4, 0.0],
            "equivalent_radius_m": 0.0,
            "pc1_m": 0.0,
            "pc2_m": 0.0,
        }

        level, away = controller.apf_obstacle_level_and_away(risk_probe, 10.0)

        self.assertLess(level, 1.0)
        np.testing.assert_allclose(away, [-1.0, 0.0])

    def test_prediction_runs_to_30_seconds_without_virtual_obstacle(self):
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

        self.assertEqual(virtual, [])
        self.assertTrue(np.isnan(track["collision_time_s"]))
        self.assertTrue(np.isnan(track["virtual_position_ne"]).all())
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

    def test_track_update_records_corrected_state_in_motion_window(self):
        controller = LaptopController.__new__(LaptopController)
        controller.obstacle_ekf_tracking_enabled = True
        controller.obstacle_ekf_prediction_enabled = False
        controller.obstacle_history_len = 10
        controller.obstacle_stats_window_s = 10.0
        controller.obstacle_min_pc1_m = 0.2
        controller.obstacle_min_pc2_m = 0.1
        controller.obstacle_heading_hold_speed_m_s = 0.03
        controller.obstacle_ekf_initial_position_std_m = 0.1
        controller.obstacle_ekf_initial_velocity_std_m_s = 1.0
        controller.obstacle_ekf_static_speed_reset_m_s = 0.0
        controller.obstacle_prediction_min_samples = 2
        controller.obstacle_prediction_min_hits = 2
        controller.obstacle_prediction_min_time_span_s = 0.05
        controller.obstacle_prediction_min_displacement_m = 0.05
        controller.obstacle_prediction_min_speed_m_s = 0.05
        controller.apf_dynamic_speed_threshold_m_s = 0.05
        controller.apf_dynamic_exit_speed_threshold_m_s = 0.03
        controller.obstacle_prediction_max_speed_std_m_s = 1.0
        controller.obstacle_prediction_max_heading_var_rad2 = np.pi ** 2
        controller.obstacle_max_accel_m_s2 = 10.0
        controller.apf_track_timeout_s = 1.0
        controller.apf_track_association_m = 1.0
        controller.lidar_dbscan_eps_m = 0.2
        controller.apf_next_track_id = 1
        controller.apf_obstacle_tracks = []
        controller.annotate_lidar_obstacle_with_track = lambda obstacle, track: None
        controller.sync_lidar_obstacle_arrays = lambda: None
        controller.obstacle_measurement_covariance = lambda _: np.eye(2, dtype=float)
        controller.obstacle_ekf_predict = lambda state, covariance, dt: (
            np.asarray(state, dtype=float).reshape(4).copy(),
            np.asarray(covariance, dtype=float).reshape(4, 4).copy(),
        )
        controller.obstacle_ekf_update = lambda state, covariance, detection, measurement_covariance: (
            np.array([detection[0], detection[1], 0.4, -0.1], dtype=float),
            np.asarray(covariance, dtype=float).reshape(4, 4).copy(),
        )

        controller.lidar_obstacles = [{
            "centre_ne": [1.0, 0.0],
            "pc1_m": 1.0,
            "pc2_m": 0.4,
            "length_axis_ne": [1.0, 0.0],
            "measurement_covariance": np.eye(2, dtype=float),
        }]
        controller.update_apf_obstacle_tracks(0.0)

        controller.lidar_obstacles = [{
            "centre_ne": [1.2, -0.05],
            "pc1_m": 1.0,
            "pc2_m": 0.4,
            "length_axis_ne": [1.0, 0.0],
            "measurement_covariance": np.eye(2, dtype=float),
        }]
        controller.update_apf_obstacle_tracks(0.1)

        track = controller.apf_obstacle_tracks[0]
        np.testing.assert_allclose(
            track["motion_window"][-1]["pos_ne"],
            [1.2, -0.05],
        )
        np.testing.assert_allclose(
            track["motion_window"][-1]["vel_ne"],
            [0.4, -0.1],
        )

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
        controller.robot_pose_history = [
            np.array([0.0, 0.0, 0.0, 0.0]),
            np.array([0.1, 0.0, 0.0, 0.1]),
            np.array([0.2, 0.0, 0.0, 0.2]),
        ]
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

    def test_colreg_scenarios_use_different_field_ranges(self):
        controller = LaptopController.__new__(LaptopController)
        controller.apf_cluster_range_enabled = True
        controller.apf_own_equivalent_radius_m = 0.0
        controller.obstacle_min_pc1_m = 0.3
        controller.obstacle_min_pc2_m = 0.16
        controller.apf_overtaking_longitudinal_scale = 0.75
        controller.apf_overtaking_lateral_scale = 0.65
        controller.apf_crossing_longitudinal_scale = 1.1
        controller.apf_crossing_lateral_scale = 3.2
        controller.apf_head_on_longitudinal_scale = 1.3
        controller.apf_head_on_lateral_scale = 1.2

        obstacle = {"pc1_m": 4.0, "pc2_m": 1.0}
        axis = [1.0, 0.0]

        overtaking_level, _ = controller.apf_point_level_and_away(
            [0.0, 1.5],
            obstacle,
            1.0,
            axis,
            encounter="overtaking",
        )
        crossing_level, _ = controller.apf_point_level_and_away(
            [0.0, 1.5],
            obstacle,
            1.0,
            axis,
            encounter="crossing_from_starboard",
        )
        head_on_level, _ = controller.apf_point_level_and_away(
            [2.1, 0.0],
            obstacle,
            1.0,
            axis,
            encounter="head_on",
        )
        static_level, _ = controller.apf_point_level_and_away(
            [2.1, 0.0],
            obstacle,
            1.0,
            axis,
        )

        self.assertGreater(overtaking_level, 1.0)
        self.assertLess(crossing_level, 1.0)
        self.assertLess(head_on_level, static_level)

    def test_colreg_scenarios_use_different_speed_logic(self):
        controller = LaptopController.__new__(LaptopController)
        controller.route_tracking_speed_m_s = 0.2
        controller.apf_constant_descent_speed_m_s = 0.2
        controller.apf_overtaking_surge_m_s = 0.26
        controller.apf_crossing_min_forward_speed = 0.18
        controller.apf_crossing_close_quarters_surge_m_s = 0.22
        controller.apf_head_on_surge_m_s = 0.16
        controller.apf_min_forward_speed = 0.1
        controller.apf_heading_step_limit_rad = np.deg2rad(60.0)
        controller.v_max = 0.3
        controller.apf_close_quarters_surge_m_s = 0.14

        overtaking_speed = controller.apf_encounter_speed_m_s(
            "overtaking",
            0.8,
            0.0,
        )
        crossing_speed = controller.apf_encounter_speed_m_s(
            "crossing_from_starboard",
            0.7,
            0.0,
        )
        head_on_speed = controller.apf_encounter_speed_m_s(
            "head_on",
            0.7,
            np.deg2rad(45.0),
        )

        self.assertGreater(overtaking_speed, controller.apf_constant_descent_speed_m_s)
        self.assertGreaterEqual(crossing_speed, controller.apf_crossing_close_quarters_surge_m_s)
        self.assertLess(head_on_speed, controller.apf_constant_descent_speed_m_s)


if __name__ == "__main__":
    unittest.main()
