"""
Unified show_laptop.py-compatible entry point.

show_laptop.py imports this module as:
    import laptop as lt
    lt.LaptopController(...)

The source strategy files keep their historical '-' filenames, so they are
loaded with importlib instead of regular imports.
"""

from contextlib import contextmanager
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
from types import MethodType

import numpy as np


_HERE = Path(__file__).resolve().parent


def _load_source_module(name, filename):
    path = _HERE / filename
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_overtaking = _load_source_module("_laptop_overtaking_source", "laptop-overtaking.py")
_head_on = _load_source_module("_laptop_head_on_source", "laptop-headon.py")
_crossing = _load_source_module("_laptop_crossing_source", "laptop-crossing.py")

_OvertakingController = _overtaking.LaptopController
_HeadOnController = _head_on.LaptopController
_CrossingController = _crossing.LaptopController

cluster_principal_dimensions = _crossing.cluster_principal_dimensions
ellipse_level_and_away = _crossing.ellipse_level_and_away
linear_field_weight = _crossing.linear_field_weight
classic_apf_repulsive_gradient_weight = _crossing.classic_apf_repulsive_gradient_weight
papf_attractive_vector = _crossing.papf_attractive_vector
papf_repulsive_vector = _crossing.papf_repulsive_vector
generate_papf_trajectory = _crossing.generate_papf_trajectory

# Unified APF/EKF switch combinations. Change SWITCH_COMBINATION here, not in
# the strategy source files.
SWITCH_COMBINATION = os.environ.get("SWITCH_COMBINATION", "ekf_on_cluster_on")
SWITCH_COMBINATIONS = {
    "ekf_on_cluster_on": (True, True),
    "ekf_on_cluster_off": (True, False),
    "ekf_off_cluster_on": (False, True),
    "ekf_off_cluster_off": (False, False),
}
if SWITCH_COMBINATION not in SWITCH_COMBINATIONS:
    raise ValueError(f"Unknown SWITCH_COMBINATION: {SWITCH_COMBINATION}")

# show_laptop.py compatibility: keep the public module-level switches.
ENABLE_OBSTACLE_EKF_PREDICTION, ENABLE_CLUSTER_BASED_APF_RANGE = SWITCH_COMBINATIONS[SWITCH_COMBINATION]
CLASSIC_APF_INFLUENCE_DISTANCE_M = 5.0


def _mode_value(robot_value, simulation_value):
    return lambda controller: simulation_value if controller.OPERATING_MODE == 2 else robot_value


# APF parameters whose values are identical in all three strategy files.
# Parameters with different original values stay
# strategy-specific and are not listed here.
UNIFIED_APF_PARAMS = {
    "apf_avoidance_pc_scale": 20.0,
    "apf_direction_pc_scale": 30.0,
    "apf_activation_front_half_angle_rad": np.deg2rad(150.0),
    "apf_priority_front_half_angle_rad": np.deg2rad(90.0),
    "apf_goal_gain": 5.5,
    "apf_path_gain": 7.5,
    "apf_attraction_saturation_m": 3.5,
    "apf_path_threshold_m": 0.10,
    "apf_route_lookahead_m": _mode_value(1.8, 2.1),
    "apf_collision_horizon_s": _mode_value(6.0, 10.0),
    "apf_prediction_dt_s": 0.5,
    "apf_heading_gain": 0.9,
    "apf_heading_step_limit_rad": np.deg2rad(60.0),
    "apf_constant_descent_speed_m_s": lambda controller: controller.route_tracking_speed_m_s,
    "apf_dynamic_speed_threshold_m_s": _mode_value(0.05, 0.06),
    "apf_dynamic_exit_speed_threshold_m_s": _mode_value(0.03, 0.04),
    "apf_track_association_m": _mode_value(0.60, 0.80),
    "apf_track_timeout_s": _mode_value(1.0, 1.5),
    "obstacle_min_pc1_m": _mode_value(0.30, 0.36),
    "obstacle_min_pc2_m": _mode_value(0.16, 0.20),
    "apf_own_equivalent_radius_m": _mode_value(0.25, 0.30),
    "apf_side_lock_s": _mode_value(5.0, 8.0),
    "apf_side_lock_exit_level": 1.15,
    "apf_visual_hold_s": 2.0,
}


def _sync_strategy_switches():
    for module in (_overtaking, _head_on, _crossing):
        module.ENABLE_OBSTACLE_EKF_PREDICTION = bool(ENABLE_OBSTACLE_EKF_PREDICTION)
        module.ENABLE_CLUSTER_BASED_APF_RANGE = bool(ENABLE_CLUSTER_BASED_APF_RANGE)
        module.CLASSIC_APF_INFLUENCE_DISTANCE_M = float(CLASSIC_APF_INFLUENCE_DISTANCE_M)


_sync_strategy_switches()


_CSV_CONTEXT_COLUMNS = (
    "WebotsEnvironment",
    "SwitchCombination",
    "EKFPredictionEnabled",
    "ClusterSizeAPFEnabled",
    "ClassicAPFInfluenceDistance(m)",
)


def _apply_unified_apf_params(controller):
    for name, value in UNIFIED_APF_PARAMS.items():
        setattr(controller, name, value(controller) if callable(value) else value)

    controller.obstacle_min_equivalent_radius_m = 0.5 * controller.obstacle_min_pc1_m

    if hasattr(controller, "apf_build_encounter_params"):
        controller.apf_crossing_params = controller.apf_build_encounter_params()
        controller.apf_overtaking_params = controller.apf_build_encounter_params(
            avoidance_pc_scale=(
                controller.apf_avoidance_pc_scale * _overtaking.OVERTAKING_APF_RANGE_SCALE
            ),
            direction_pc_scale=(
                controller.apf_direction_pc_scale * _overtaking.OVERTAKING_APF_CLEARANCE_SCALE
            ),
        )
        controller.apf_head_on_params = controller.apf_build_encounter_params()


def _world_name_from_text(text):
    match = re.search(r'["\']([^"\']+\.wbt)["\']|(\S+\.wbt)', str(text), flags=re.IGNORECASE)
    if not match:
        return None
    return Path(match.group(1) or match.group(2)).name


def _detect_webots_environment(operating_mode):
    if operating_mode != 2:
        return "not_webots"

    for name in (
        "WEBOTS_WORLD",
        "WEBOTS_WORLD_FILE",
        "WEBOTS_CURRENT_WORLD",
        "WEBOTS_SCENARIO",
        "WORLD_FILE",
    ):
        value = os.environ.get(name)
        if not value:
            continue
        return _world_name_from_text(value) or Path(value).name or value

    try:
        if os.name == "nt":
            result = subprocess.run(
                ["wmic", "process", "where", "name like '%webots%'", "get", "CommandLine", "/value"],
                capture_output=True,
                text=True,
                timeout=0.8,
            )
        else:
            result = subprocess.run(
                ["ps", "-eo", "args"],
                capture_output=True,
                text=True,
                timeout=0.8,
            )
        detected = _world_name_from_text(result.stdout)
        if detected:
            return detected
    except Exception:
        pass

    return "WEBOTS_UNKNOWN"


def __getattr__(name):
    return getattr(_overtaking, name)


_MISSING = object()

_CROSSING_METHODS = (
    "apf_classic_qstar_m",
    "apf_point_level_and_away",
    "apf_repulsive_weight",
    "apf_boundary_repulsive_weight",
    "apf_direction_clearance_m",
    "apf_obstacle_level_and_away",
    "apf_obstacle_endpoint_direction_body",
    "apf_stern_direction_body",
    "apf_bow_direction_body",
    "apf_crossing_strategy_from_velocity",
    "apf_encounter_speed_m_s",
    "apf_obstacle_in_forward_half_plane",
    "apf_classify_encounter",
    "apf_repulsion_for_obstacle",
)

_HEAD_ON_METHODS = (
    "apf_track_for_obstacle",
    "obstacle_pc_dimensions",
    "obstacle_length_axis_ne",
    "apf_uses_cluster_range",
    "apf_build_encounter_params",
    "apf_profile_name_for_encounter",
    "apf_params_for_encounter",
    "apf_classic_qstar_m",
    "apf_point_level_and_away",
    "apf_repulsive_weight",
    "apf_direction_clearance_m",
    "apf_obstacle_level_and_away",
    "apf_cpa_metrics",
    "apf_pass_astern_side_from_velocity",
    "own_prediction_velocity_ne",
    "obstacle_track_state_at",
    "update_apf_virtual_obstacles",
    "virtual_collision_visuals",
    "apf_classify_encounter",
    "apf_default_side_from_obstacle",
    "apf_obstacle_in_priority_front_sector",
    "apf_lock_side",
    "refresh_apf_side_lock",
    "route_progress_and_point",
    "goal_distance_m",
    "final_approach_active",
    "apf_path_attraction_body",
    "apf_goal_attraction_body",
    "apf_repulsion_for_obstacle_crossing",
    "apf_avoidance_needed_crossing",
    "compute_apf_control_crossing",
    "apf_repulsion_for_obstacle",
)


class LaptopController(_OvertakingController):
    """One live controller instance with COLREG strategy dispatch."""

    _HEAD_ON_RULES = {"head_on"}
    _OVERTAKING_RULES = {"overtaking"}
    _CROSSING_RULES = {"crossing", "crossing_from_starboard", "crossing_from_port"}

    def __init__(self, OPERATING_MODE):
        _sync_strategy_switches()
        super().__init__(OPERATING_MODE)
        # Keep the live controller state tied to laptop.py's unified switches.
        self.obstacle_ekf_prediction_enabled = bool(ENABLE_OBSTACLE_EKF_PREDICTION)
        self.apf_cluster_range_enabled = bool(ENABLE_CLUSTER_BASED_APF_RANGE)
        self.apf_classic_influence_distance_m = float(CLASSIC_APF_INFLUENCE_DISTANCE_M)
        _apply_unified_apf_params(self)
        self.apf_selected_controller = "default_apf"
        self._last_colreg_decision = None
        self.webots_environment = _detect_webots_environment(self.OPERATING_MODE)
        self._csv_context_last_patched_line_end = None
        self._ensure_csv_context_header()
        # Defaults used directly by laptop-crossing.py helpers when they run on
        # this single overtaking-initialised controller instance.
        self.apf_boundary_repulsive_gain = getattr(self, "apf_boundary_repulsive_gain", 2.4)
        self.apf_boundary_activation_level = getattr(self, "apf_boundary_activation_level", 1.35)
        self.apf_boundary_inside_boost = getattr(self, "apf_boundary_inside_boost", 3.0)
        self.apf_min_detour_offset_m = getattr(
            self,
            "apf_min_detour_offset_m",
            1.0 if self.OPERATING_MODE == 2 else 0.8,
        )
        self.apf_pass_ahead_gain = getattr(self, "apf_pass_ahead_gain", 2.4)
        self.apf_crossing_min_forward_speed = getattr(
            self,
            "apf_crossing_min_forward_speed",
            0.14 if self.OPERATING_MODE == 2 else 0.16,
        )
        self.apf_crossing_close_quarters_surge_m_s = getattr(
            self,
            "apf_crossing_close_quarters_surge_m_s",
            0.18 if self.OPERATING_MODE == 2 else 0.16,
        )

    def update_apf_obstacle_tracks(self, stamp_s):
        _OvertakingController.update_apf_obstacle_tracks(self, stamp_s)

    def _run_context(self):
        return {
            "webots_environment": self.webots_environment,
            "switch_combination": SWITCH_COMBINATION,
            "ekf_prediction_enabled": bool(self.obstacle_ekf_prediction_enabled),
            "cluster_size_apf_enabled": bool(self.apf_cluster_range_enabled),
            "classic_apf_influence_distance_m": float(self.apf_classic_influence_distance_m),
        }

    def _csv_context_values(self):
        context = self._run_context()
        return [
            context["webots_environment"],
            context["switch_combination"],
            int(context["ekf_prediction_enabled"]),
            int(context["cluster_size_apf_enabled"]),
            context["classic_apf_influence_distance_m"],
        ]

    def _ensure_csv_context_header(self):
        try:
            lines = self.filename.read_text().splitlines()
            if not lines:
                return
            header = lines[0].split(",")
            if all(column in header for column in _CSV_CONTEXT_COLUMNS):
                return
            lines[0] = lines[0] + "," + ",".join(_CSV_CONTEXT_COLUMNS)
            self.filename.write_text("\n".join(lines) + "\n")
        except Exception:
            return

    def _patch_latest_csv_context_row(self):
        try:
            values = ",".join(str(value) for value in self._csv_context_values())
            with self.filename.open("rb+") as f:
                f.seek(0, os.SEEK_END)
                end = f.tell()
                if end <= 0:
                    return

                pos = end - 1
                while pos >= 0:
                    f.seek(pos)
                    if f.read(1) not in (b"\n", b"\r"):
                        break
                    pos -= 1
                if pos < 0:
                    return

                line_end = pos + 1
                if getattr(self, "_csv_context_last_patched_line_end", None) == line_end:
                    return

                f.seek(line_end)
                f.truncate()
                f.write(("," + values + "\n").encode("utf-8"))
                self._csv_context_last_patched_line_end = f.tell() - 1
        except Exception:
            return

    @contextmanager
    def _crossing_strategy_methods(self):
        """Temporarily use crossing.py APF helpers on this same controller."""
        previous = {}
        for name in _CROSSING_METHODS:
            method = getattr(_CrossingController, name, None)
            if method is None:
                continue
            previous[name] = self.__dict__.get(name, _MISSING)
            setattr(self, name, MethodType(method, self))

        try:
            yield
        finally:
            for name, value in previous.items():
                if value is _MISSING:
                    self.__dict__.pop(name, None)
                else:
                    setattr(self, name, value)

    @contextmanager
    def _head_on_strategy_methods(self):
        """Temporarily use headon.py APF helpers on this same controller."""
        previous = {}
        for name in _HEAD_ON_METHODS:
            method = getattr(_HeadOnController, name, None)
            if method is None:
                continue
            previous[name] = self.__dict__.get(name, _MISSING)
            setattr(self, name, MethodType(method, self))

        try:
            yield
        finally:
            for name, value in previous.items():
                if value is _MISSING:
                    self.__dict__.pop(name, None)
                else:
                    setattr(self, name, value)

    def _obstacle_body_position(self, obstacle):
        centre_body = np.asarray(
            obstacle.get("centre_body", [np.nan, np.nan]),
            dtype=float,
        ).reshape(2)
        if np.isfinite(centre_body).all():
            return centre_body

        centre_ne = np.asarray(obstacle.get("centre_ne", [np.nan, np.nan]), dtype=float).reshape(2)
        if np.isfinite(centre_ne).all():
            return self.earth_point_to_body(centre_ne)

        return centre_body

    def _obstacle_body_velocity(self, obstacle):
        velocity_ne = np.asarray(
            obstacle.get("velocity_ne", [np.nan, np.nan]),
            dtype=float,
        ).reshape(2)
        if np.isfinite(velocity_ne).all():
            return self.earth_vector_to_body(velocity_ne)

        track = None if bool(obstacle.get("virtual", False)) else self.apf_track_for_obstacle(obstacle)
        if track is None or not self.obstacle_track_motion_is_stable(track):
            return np.zeros(2, dtype=float)

        velocity_ne = np.asarray(
            track.get("velocity_mean_ne", track.get("vel_ne", [np.nan, np.nan])),
            dtype=float,
        ).reshape(2)
        if not np.isfinite(velocity_ne).all():
            velocity_ne = np.asarray(track.get("vel_ne", [np.nan, np.nan]), dtype=float).reshape(2)
        if np.isfinite(velocity_ne).all():
            return self.earth_vector_to_body(velocity_ne)

        return np.zeros(2, dtype=float)

    def _rule_priority(self, rule):
        if rule == "head_on":
            return 0
        if rule == "overtaking":
            return 1
        if rule in self._CROSSING_RULES:
            return 2
        if rule == "static_obstacle":
            return 3
        return 4

    # COLREG rule detection location.
    def select_colreg_strategy(self):
        """Pick the current primary COLREG rule without creating another controller."""
        own_vel_body = self.current_velocity_body()
        try:
            virtual_obstacles = self.update_apf_virtual_obstacles()
        except Exception:
            virtual_obstacles = []

        best = {
            "rule": "none",
            "controller": "default_apf",
            "tcpa_s": np.nan,
            "dcpa_m": np.nan,
            "score": (self._rule_priority("none"), np.inf, np.inf),
        }

        for obstacle in list(getattr(self, "lidar_obstacles", []) or []) + list(virtual_obstacles or []):
            try:
                obs_pos_body = self._obstacle_body_position(obstacle)
                if not np.isfinite(obs_pos_body).all():
                    continue

                angle_rad = abs(_overtaking.wrap_angle(float(np.arctan2(obs_pos_body[1], obs_pos_body[0]))))
                if (
                    not bool(obstacle.get("virtual", False))
                    and angle_rad > self.apf_activation_front_half_angle_rad
                ):
                    continue

                obs_vel_body = self._obstacle_body_velocity(obstacle)
                if bool(obstacle.get("virtual", False)):
                    rule = str(obstacle.get("encounter_mode", "crossing"))
                    if rule in {"dynamic_virtual_obstacle", "predicted_collision", "none", ""}:
                        rule, _, _ = _overtaking.LaptopController.apf_classify_encounter(
                            self,
                            obs_pos_body,
                            obs_vel_body,
                            own_vel_body,
                        )
                else:
                    rule, _, _ = _overtaking.LaptopController.apf_classify_encounter(
                        self,
                        obs_pos_body,
                        obs_vel_body,
                        own_vel_body,
                    )

                if rule not in self._HEAD_ON_RULES | self._OVERTAKING_RULES | self._CROSSING_RULES:
                    rule = "static_obstacle"

                tcpa_s, dcpa_m = self.apf_cpa_metrics(obs_pos_body, obs_vel_body, own_vel_body)
                distance_m = float(np.linalg.norm(obs_pos_body))
                tcpa_score = float(tcpa_s) if np.isfinite(tcpa_s) else np.inf
                distance_score = float(dcpa_m) if np.isfinite(dcpa_m) else distance_m
                score = (self._rule_priority(rule), tcpa_score, distance_score)

                if score < best["score"]:
                    best.update(
                        {
                            "rule": rule,
                            "tcpa_s": tcpa_s,
                            "dcpa_m": dcpa_m,
                            "score": score,
                        }
                    )
            except Exception:
                continue

        rule = best["rule"]
        if rule in self._HEAD_ON_RULES:
            best["controller"] = "head_on"
        elif rule in self._OVERTAKING_RULES:
            best["controller"] = "overtaking"
        elif rule in self._CROSSING_RULES:
            best["controller"] = "crossing"

        self._last_colreg_decision = best
        return rule

    def apf_primary_encounter_mode(self):
        return self.select_colreg_strategy()

    def _mark_selected_controller(self, rule):
        if rule in self._HEAD_ON_RULES:
            self.apf_selected_controller = "head_on"
            self.apf_active_profile_name = "head_on"
        elif rule in self._OVERTAKING_RULES:
            self.apf_selected_controller = "overtaking"
            self.apf_active_profile_name = "overtaking"
        elif rule in self._CROSSING_RULES:
            self.apf_selected_controller = "crossing"
            self.apf_active_profile_name = "crossing"
        else:
            self.apf_selected_controller = "default_apf"

    # crossing/overtaking/head_on dispatch location.
    def compute_apf_control(self, t, u_track):
        selected_rule = self.select_colreg_strategy()
        self._mark_selected_controller(selected_rule)

        if selected_rule in self._HEAD_ON_RULES:
            with self._head_on_strategy_methods():
                u_cmd = _HeadOnController.compute_apf_control(self, t, u_track)
        elif selected_rule in self._OVERTAKING_RULES:
            u_cmd = _OvertakingController.compute_apf_control(self, t, u_track)
        elif selected_rule in self._CROSSING_RULES:
            with self._crossing_strategy_methods():
                u_cmd = _CrossingController.compute_apf_control(self, t, u_track)
        else:
            u_cmd = _OvertakingController.compute_apf_control(self, t, u_track)

        self._mark_selected_controller(selected_rule)
        if self.apf_encounter_mode in ("none", "static_obstacle") and selected_rule != "none":
            self.apf_encounter_mode = selected_rule
        if self._last_colreg_decision is not None:
            self.apf_colreg_tcpa_s = self._last_colreg_decision.get("tcpa_s", self.apf_colreg_tcpa_s)
            self.apf_colreg_dcpa_m = self._last_colreg_decision.get("dcpa_m", self.apf_colreg_dcpa_m)
        return u_cmd

    def apf_avoidance_needed(self):
        selected_rule = self.select_colreg_strategy()
        self._mark_selected_controller(selected_rule)
        if selected_rule in self._HEAD_ON_RULES:
            with self._head_on_strategy_methods():
                return _HeadOnController.apf_avoidance_needed(self)
        if selected_rule in self._CROSSING_RULES:
            with self._crossing_strategy_methods():
                return _CrossingController.apf_avoidance_needed(self)
        return _OvertakingController.apf_avoidance_needed(self)

    def write_obstacle_snapshot(self):
        stamp_s = self.latest_lidar_received_s
        _OvertakingController.write_obstacle_snapshot(self)
        self._patch_latest_csv_context_row()
        if stamp_s is None:
            return

        path = self.obstacle_log_dir / f"obstacle_{int(round(float(stamp_s) * 1000.0))}.json"
        if not path.exists():
            return

        try:
            with path.open("r") as f:
                payload = json.load(f)
            payload["run_context"] = self._run_context()
            apf = payload.setdefault("apf", {})
            apf["selected_controller"] = self.apf_selected_controller
            apf["colreg_active"] = self.apf_colreg_active
            apf["active_profile"] = self.apf_active_profile_name
            with path.open("w") as f:
                json.dump(self.json_safe(payload), f, indent=2)
        except Exception:
            return


for _crossing_method_name in _CROSSING_METHODS:
    if not hasattr(LaptopController, _crossing_method_name):
        setattr(
            LaptopController,
            _crossing_method_name,
            getattr(_CrossingController, _crossing_method_name),
        )
