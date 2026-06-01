"""Parameter declarations and loading for the monitor node.

Lives outside ``node.py`` so the contract (names, types, defaults) is a single
flat module that can be exercised without spinning up rclpy.

Two parameter classes, two delivery paths:

* **Scalars and lists** (``_PARAM_SPECS``) follow the standard ROS 2 override
  chain (lowest to highest precedence):

      1. The defaults listed in ``_PARAM_SPECS`` below.
      2. ``config/monitor.yaml`` loaded by the launch via ``parameters=[...]``.
      3. A dict in the same ``parameters=[...]`` list (``LaunchConfiguration``
         substitutions in the launch file).
      4. CLI overrides on ``ros2 launch ... key:=value``.

  The ``payload_profile`` scalar selects which per-joint torque maps (idle and
  max) are injected for the B601. Valid values: ``light``, ``medium``,
  ``rated`` (see ``PAYLOAD_PROFILES`` and ``docs/per-joint-thresholds.md``).

* **Per-joint maps** (``per_joint_max_abs_torque_nm``,
  ``per_joint_idle_torque_warn_nm``, ``per_joint_max_abs_velocity_rad_s``)
  are *not* ROS parameters. ``rclpy``'s ``declare_parameter`` only accepts
  ``bool``, ``int``, ``float``, ``str`` and arrays of those; ``dict`` raises
  ``TypeError``. They live as Python constants in this module (see
  ``_B601_PROFILES`` and ``_B601_PER_JOINT_MAX_ABS_VELOCITY_RAD_S``) and are
  injected into the params dict by :func:`load_params`. Editing them
  requires a rebuild; YAML/launch/CLI overrides do **not** apply.

  These per-joint maps are **hardware-envelope** WARN thresholds derived from
  motor datasheets and URDF gravity computation, not mode-aware operational
  policy. The global fallbacks in ``monitor.yaml`` still apply to joints not
  listed here.

``declare_parameters`` registers the scalar defaults, ``load_params`` reads
whatever the parameter system resolved and tacks on the per-joint maps for
the selected profile.
"""

from __future__ import annotations

from typing import Any

from rclpy.node import Node

# Velocity map: hardware-envelope WARN thresholds sized to each motor class at
# 24 V supply. Datasheet (Damiao / Seeed Studio wiki) no-load speeds at 24 V:
#   - DM-J4340P-2EC: 52.5 rpm = 5.50 rad/s (rated 36 rpm = 3.77 rad/s)
#   - DM-J4310-2EC : 200  rpm = 20.94 rad/s (rated 120 rpm = 12.57 rad/s)
# These thresholds are not firmware safety stops; they are diagnostic WARN
# levels intended to flag the joint moving well outside its design envelope.
# Velocity is *not* payload-aware: motor envelope does not change with mass.
# TODO: add mode-aware velocity diagnostics (POS_VEL vlim vs MIT vs GC) once
#       control context is consumed by per-joint checks beyond stationary effort.
# TODO: collect an empirical velocity baseline (normal operation, GC, traj)
#       before treating these values as final tuning.
_B601_PER_JOINT_MAX_ABS_VELOCITY_RAD_S: dict[str, float] = {
    "joint1": 6.0,
    "joint2": 6.0,
    "joint3": 6.0,
    "joint4": 20.0,
    "joint5": 20.0,
    "joint6": 20.0,
}


# Payload profiles: per-joint idle and max torque thresholds, indexed by the
# assumed gripper payload mass. Selectable at launch via the
# ``payload_profile`` parameter.
#
# Derivation: static gravity torque |tau_i(q)| sampled across the reachable
# joint workspace from the reBot-DevArm_fixend URDF (link masses, COMs, joint
# geometry), adding a virtual point mass at the end-effector. Idle thresholds
# sit near the p95 of that distribution, capped below the max-torque limit so
# the sequence ``elevated -> WARN idle -> WARN high torque`` stays ordered.
# Max thresholds scale with the profile, from motor rated (light) to a
# generous fraction of motor peak (rated).
#
# Caveats:
#   * URDF masses are CAD-derived; real measured masses may differ.
#   * Thresholds are NOT empirically validated yet. Collect a baseline log
#     before treating them as final calibration. See open work in
#     docs/per-joint-thresholds.md.
#   * Velocity, idle_velocity, position_jump and torque_jump are NOT profile-
#     aware: they do not depend on payload.
#   * Globals in monitor.yaml are NOT profile-aware: they only apply to joints
#     not listed in the per-joint maps (i.e. non-B601 setups).
#
# Profile selection is static per session; changing it requires a node restart.
_B601_PROFILES: dict[str, dict[str, dict[str, float]]] = {
    "light": {
        # 0.5 kg payload. Default. Preserves currently shipped behaviour.
        # Tight thresholds matching motor rated torque per family.
        # NOTE: URDF home (q=0) is a stretched pose where joint3 sees ~9.3 Nm
        # with 0.5 kg payload, which equals the max threshold. Booting from
        # this pose may emit a transient WARN. The arm is rarely operated at
        # q=0; operationally rest poses fold the elbow.
        "per_joint_idle_torque_warn_nm": {
            "joint1": 1.0,
            "joint2": 8.0,
            "joint3": 8.0,
            "joint4": 2.5,
            "joint5": 1.5,
            "joint6": 1.0,
        },
        "per_joint_max_abs_torque_nm": {
            "joint1": 9.0,
            "joint2": 9.0,
            "joint3": 9.0,
            "joint4": 3.0,
            "joint5": 3.0,
            "joint6": 3.0,
        },
    },
    "medium": {
        # 1.0 kg payload. Pick-and-place with moderate tooling.
        # Max thresholds at ~1.33x motor rated to tolerate peak transients.
        "per_joint_idle_torque_warn_nm": {
            "joint1": 1.0,
            "joint2": 10.0,
            "joint3": 10.0,
            "joint4": 3.0,
            "joint5": 2.0,
            "joint6": 1.0,
        },
        "per_joint_max_abs_torque_nm": {
            "joint1": 12.0,
            "joint2": 12.0,
            "joint3": 12.0,
            "joint4": 4.0,
            "joint5": 4.0,
            "joint6": 4.0,
        },
    },
    "rated": {
        # 1.5 kg payload (manufacturer's max advertised).
        # Max thresholds at ~2x rated for 4340P (well below peak 27 Nm) and
        # ~80% of peak for 4310. Accepts sustained operation at spec maximum.
        # Extreme poses (folded back) still WARN: that is correct, those
        # configurations exceed even the peak motor envelope.
        "per_joint_idle_torque_warn_nm": {
            "joint1": 1.0,
            "joint2": 14.0,
            "joint3": 12.0,
            "joint4": 4.0,
            "joint5": 2.5,
            "joint6": 1.0,
        },
        "per_joint_max_abs_torque_nm": {
            "joint1": 18.0,
            "joint2": 18.0,
            "joint3": 18.0,
            "joint4": 5.5,
            "joint5": 5.5,
            "joint6": 5.5,
        },
    },
}

PAYLOAD_PROFILES: tuple[str, ...] = tuple(_B601_PROFILES.keys())
DEFAULT_PAYLOAD_PROFILE: str = "light"

PAYLOAD_PROFILE_ASSUMED_KG: dict[str, float] = {
    "light": 0.5,
    "medium": 1.0,
    "rated": 1.5,
}


_PARAM_SPECS: tuple[tuple[str, Any], ...] = (
    ("enable_joint_states_monitor", True),
    ("enable_per_joint_monitor", True),
    ("enable_arm_status_monitor", True),
    ("enable_gripper_monitor", True),
    ("log_only_on_change", True),
    ("joint_states_topic", "/rebotarm/joint_states"),
    ("expected_rate_hz", 100.0),
    ("stale_timeout_s", 0.5),
    ("min_rate_ratio", 0.5),
    ("max_position_jump_rad", 0.5),
    ("max_abs_velocity_rad_s", 10.0),
    ("max_abs_effort_nm", 8.0),
    ("joint_state_topic_prefix", "/rebotarm/joints"),
    ("joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]),
    ("per_joint_stale_timeout_s", 0.5),
    ("max_abs_joint_velocity_rad_s", 10.0),
    ("max_abs_joint_torque_nm", 8.0),
    ("idle_velocity_threshold_rad_s", 0.05),
    ("idle_torque_warn_nm", 3.0),
    ("max_joint_position_jump_rad", 0.5),
    ("max_joint_torque_jump_nm", 3.0),
    ("expected_enabled_status_code", 1),
    ("allow_disabled_status_code", True),
    ("disabled_status_code", 0),
    ("arm_status_topic", "/rebotarm/arm_status"),
    ("arm_status_stale_timeout_s", 1.0),
    ("arm_status_warn_on_snapshot_age", False),
    ("expect_arm_enabled", False),
    ("expect_control_loop_active", False),
    ("gripper_state_topic", "/rebotarm/gripper/state"),
    ("gripper_stale_timeout_s", 1.0),
    ("max_abs_gripper_velocity", 10.0),
    ("max_abs_gripper_torque", 5.0),
    ("enable_can_monitor", False),
    ("can_interfaces", "can0"),
    ("can_warn_on_iface_down", True),
    ("can_error_warn_per_period", 1),
    ("can_dropped_warn_per_period", 10),
    ("enable_serial_monitor", True),
    ("serial_device", "/dev/ttyACM0"),
    ("enable_process_monitor", True),
    ("driver_process_pattern", "reBotArmController"),
    ("driver_process_pid", 0),
    ("driver_cpu_warn_percent", 90.0),
    ("driver_rss_warn_mb", 1024.0),
    ("driver_threads_warn", 64),
    ("driver_zombie_is_error", True),
    ("status_log_period_s", 1.0),
    ("diagnostics_period_s", 1.0),
    ("payload_profile", DEFAULT_PAYLOAD_PROFILE),
)


_TYPES: dict[str, type] = {
    "enable_joint_states_monitor": bool,
    "enable_per_joint_monitor": bool,
    "enable_arm_status_monitor": bool,
    "enable_gripper_monitor": bool,
    "log_only_on_change": bool,
    "joint_states_topic": str,
    "expected_rate_hz": float,
    "stale_timeout_s": float,
    "min_rate_ratio": float,
    "max_position_jump_rad": float,
    "max_abs_velocity_rad_s": float,
    "max_abs_effort_nm": float,
    "joint_state_topic_prefix": str,
    "per_joint_stale_timeout_s": float,
    "max_abs_joint_velocity_rad_s": float,
    "max_abs_joint_torque_nm": float,
    "idle_velocity_threshold_rad_s": float,
    "idle_torque_warn_nm": float,
    "max_joint_position_jump_rad": float,
    "max_joint_torque_jump_nm": float,
    "expected_enabled_status_code": int,
    "allow_disabled_status_code": bool,
    "disabled_status_code": int,
    "arm_status_topic": str,
    "arm_status_stale_timeout_s": float,
    "arm_status_warn_on_snapshot_age": bool,
    "expect_arm_enabled": bool,
    "expect_control_loop_active": bool,
    "gripper_state_topic": str,
    "gripper_stale_timeout_s": float,
    "max_abs_gripper_velocity": float,
    "max_abs_gripper_torque": float,
    "enable_can_monitor": bool,
    "can_warn_on_iface_down": bool,
    "can_error_warn_per_period": int,
    "can_dropped_warn_per_period": int,
    "enable_serial_monitor": bool,
    "serial_device": str,
    "enable_process_monitor": bool,
    "driver_process_pattern": str,
    "driver_process_pid": int,
    "driver_cpu_warn_percent": float,
    "driver_rss_warn_mb": float,
    "driver_threads_warn": int,
    "driver_zombie_is_error": bool,
    "status_log_period_s": float,
    "diagnostics_period_s": float,
    "payload_profile": str,
}


def declare_parameters(node: Node) -> None:
    """Declare every parameter the monitor consumes with its default value."""
    for name, default in _PARAM_SPECS:
        node.declare_parameter(name, default)


def _split_csv(value: Any) -> list[str]:
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    return [str(item).strip() for item in value if str(item).strip()]


def resolve_joint_threshold(
    joint_name: str,
    global_value: float,
    overrides: dict[str, float],
) -> float:
    """Return per-joint override when present, otherwise the global fallback."""
    if joint_name in overrides:
        return overrides[joint_name]
    return global_value


def normalize_payload_profile(value: Any) -> str:
    """Lowercase + validate a payload profile name.

    Raises ``ValueError`` with the list of accepted values on unknown input.
    Centralised so launch-arg, YAML, CLI and programmatic callers all fail
    the same way.
    """
    profile = str(value).strip().lower()
    if profile not in _B601_PROFILES:
        raise ValueError(
            f"unknown payload_profile {value!r}; "
            f"expected one of {sorted(_B601_PROFILES)}"
        )
    return profile


def profile_assumed_payload_kg(profile: str) -> float:
    """Return the nominal gripper payload (kg) for a validated profile name."""
    profile = normalize_payload_profile(profile)
    return PAYLOAD_PROFILE_ASSUMED_KG[profile]


def per_joint_threshold_maps(
    profile: str = DEFAULT_PAYLOAD_PROFILE,
) -> dict[str, dict[str, float]]:
    """Per-joint override maps for the selected payload profile.

    Idle and max torque maps vary per profile. Velocity is profile-independent
    (motor envelope does not change with payload). Keyed by the params-dict
    name each map is injected into by :func:`load_params`.
    """
    profile = normalize_payload_profile(profile)
    profile_maps = _B601_PROFILES[profile]
    return {
        "per_joint_max_abs_torque_nm": dict(profile_maps["per_joint_max_abs_torque_nm"]),
        "per_joint_idle_torque_warn_nm": dict(
            profile_maps["per_joint_idle_torque_warn_nm"]
        ),
        "per_joint_max_abs_velocity_rad_s": dict(
            _B601_PER_JOINT_MAX_ABS_VELOCITY_RAD_S
        ),
    }


def load_params(node: Node) -> dict[str, Any]:
    """Read every declared parameter, coerce it, and return a plain dict."""
    params: dict[str, Any] = {}
    for name, _default in _PARAM_SPECS:
        raw = node.get_parameter(name).value
        if name in ("joint_names", "can_interfaces"):
            params[name] = _split_csv(raw)
            continue
        caster = _TYPES.get(name)
        params[name] = caster(raw) if caster is not None else raw
    # Validate and normalise the profile name before injecting the maps so a
    # typo fails the node at startup instead of silently falling back.
    params["payload_profile"] = normalize_payload_profile(params["payload_profile"])
    for key, mapping in per_joint_threshold_maps(params["payload_profile"]).items():
        params[key] = mapping
    return params
