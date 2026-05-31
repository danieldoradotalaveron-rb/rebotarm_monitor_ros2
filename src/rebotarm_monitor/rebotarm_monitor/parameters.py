"""Parameter declarations and loading for the monitor node.

Lives outside ``node.py`` so the contract (names, types, defaults) is a single
flat module that can be exercised without spinning up rclpy.

ROS 2 parameter override chain (lowest to highest precedence):

    1. The defaults listed in ``_PARAM_SPECS`` below
       (used when nothing else sets the value).
    2. A YAML passed to the node via ``parameters=[config_file]`` in the
       launch file. See ``config/monitor.yaml``.
    3. A dict passed in the same ``parameters=[...]`` list (typically wired
       to ``LaunchConfiguration`` substitutions in the launch).
    4. CLI overrides on ``ros2 launch ... key:=value``.

``declare_parameters`` registers the defaults, ``load_params`` reads whatever
value the parameter system ended up with after all the layers above resolved.
"""

from __future__ import annotations

from typing import Any

from rclpy.node import Node


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
}


def declare_parameters(node: Node) -> None:
    """Declare every parameter the monitor consumes with its default value."""
    for name, default in _PARAM_SPECS:
        node.declare_parameter(name, default)


def _split_csv(value: Any) -> list[str]:
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    return [str(item).strip() for item in value if str(item).strip()]


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
    return params
