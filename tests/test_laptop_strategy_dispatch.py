import sys
import unittest
from pathlib import Path
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import laptop  # noqa: E402


class LaptopStrategyDispatchTest(unittest.TestCase):
    def controller_for(self, rule):
        controller = laptop.LaptopController.__new__(laptop.LaptopController)
        controller.select_colreg_strategy = lambda: rule
        controller.apf_encounter_mode = "none"
        controller.apf_colreg_tcpa_s = float("nan")
        controller.apf_colreg_dcpa_m = float("nan")
        controller._last_colreg_decision = None
        return controller

    def test_compute_dispatches_to_each_strategy_source(self):
        cases = (
            ("head_on", "head_on", laptop._HeadOnController),
            ("overtaking", "overtaking", laptop._OvertakingController),
            ("crossing", "crossing", laptop._CrossingController),
        )

        for rule, selected, source in cases:
            with self.subTest(rule=rule):
                controller = self.controller_for(rule)
                expected = object()
                with patch.object(source, "compute_apf_control", return_value=expected) as compute:
                    result = controller.compute_apf_control(1.0, object())

                self.assertIs(result, expected)
                compute.assert_called_once()
                self.assertEqual(controller.apf_selected_controller, selected)

    def test_avoidance_check_dispatches_to_each_strategy_source(self):
        cases = (
            ("head_on", laptop._HeadOnController),
            ("overtaking", laptop._OvertakingController),
            ("crossing", laptop._CrossingController),
        )

        for rule, source in cases:
            with self.subTest(rule=rule):
                controller = self.controller_for(rule)
                with patch.object(source, "apf_avoidance_needed", return_value=True) as check:
                    self.assertTrue(controller.apf_avoidance_needed())

                check.assert_called_once()

    def test_head_on_helpers_are_bound_from_head_on_source(self):
        controller = self.controller_for("head_on")

        with controller._head_on_strategy_methods():
            self.assertIs(
                controller.apf_repulsion_for_obstacle.__func__,
                laptop._HeadOnController.apf_repulsion_for_obstacle,
            )

        self.assertIs(
            controller.apf_repulsion_for_obstacle.__func__,
            laptop._OvertakingController.apf_repulsion_for_obstacle,
        )

    def test_overtaking_uses_larger_avoidance_range_and_clearance(self):
        class Controller:
            OPERATING_MODE = 2
            route_tracking_speed_m_s = 0.85

            def apf_build_encounter_params(self, **overrides):
                params = {
                    "avoidance_pc_scale": self.apf_avoidance_pc_scale,
                    "direction_pc_scale": self.apf_direction_pc_scale,
                    "classic_influence_distance_m": self.apf_classic_influence_distance_m,
                }
                params.update(overrides)
                return params

        controller = Controller()
        controller.apf_classic_influence_distance_m = laptop.CLASSIC_APF_INFLUENCE_DISTANCE_M
        laptop._apply_unified_apf_params(controller)

        self.assertEqual(controller.apf_crossing_params["avoidance_pc_scale"], 20.0)
        self.assertEqual(controller.apf_head_on_params["avoidance_pc_scale"], 20.0)
        self.assertEqual(controller.apf_overtaking_params["avoidance_pc_scale"], 50.0)
        self.assertEqual(controller.apf_crossing_params["direction_pc_scale"], 30.0)
        self.assertEqual(controller.apf_head_on_params["direction_pc_scale"], 30.0)
        self.assertEqual(controller.apf_overtaking_params["direction_pc_scale"], 60.0)
        self.assertEqual(controller.apf_crossing_params["classic_influence_distance_m"], 5.0)
        self.assertEqual(controller.apf_head_on_params["classic_influence_distance_m"], 5.0)
        self.assertEqual(controller.apf_overtaking_params["classic_influence_distance_m"], 5.0)

    def test_overtaking_world_is_not_reclassified_by_noisy_velocity(self):
        controller = laptop.LaptopController.__new__(laptop.LaptopController)
        controller.webots_environment = "mr_webots_overtaking_large_ship.wbt"
        controller.apf_dynamic_speed_threshold_m_s = 0.06

        encounter, side, _ = controller.apf_classify_encounter(
            laptop.np.array([2.0, 0.2]),
            laptop.np.array([0.0, 0.5]),
            laptop.np.array([0.85, 0.0]),
        )

        self.assertEqual(encounter, "overtaking")
        self.assertEqual(side, -1.0)

    def test_close_overtaking_turns_first_then_resumes_speed(self):
        controller = laptop.LaptopController.__new__(laptop.LaptopController)
        controller.webots_environment = "mr_webots_overtaking_small_ship.wbt"
        controller.apf_overtaking_close_distance_m = 3.0
        controller.apf_overtaking_close_turn_angle_rad = laptop.np.deg2rad(55.0)
        controller.apf_overtaking_turn_release_angle_rad = laptop.np.deg2rad(40.0)
        controller.apf_overtaking_close_turn_speed_m_s = 0.15
        controller.route_heading_rad = 0.0
        controller.Yaw = 0.0

        angle, speed = controller.apf_overtaking_close_turn_command(
            "static_obstacle",
            2.0,
            laptop.np.deg2rad(20.0),
            0.85,
        )
        self.assertAlmostEqual(angle, laptop.np.deg2rad(55.0))
        self.assertEqual(speed, 0.15)

        controller.Yaw = laptop.np.deg2rad(45.0)
        _, speed = controller.apf_overtaking_close_turn_command(
            "overtaking",
            2.0,
            angle,
            0.85,
        )
        self.assertEqual(speed, 0.85)


if __name__ == "__main__":
    unittest.main()
