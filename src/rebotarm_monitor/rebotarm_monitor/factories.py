"""Composition root: build trackers from a flat parameter dict.

Keeping creation here lets unit tests build trackers without ROS by passing a
plain dict (the same shape :func:`parameters.load_params` returns) and any
adapter fakes for I/O-heavy trackers.
"""

from __future__ import annotations

from typing import Any, Optional

from .adapters.device_path import DevicePathInspector
from .adapters.process_info import ProcessInspector
from .adapters.sysfs import SysFsReader
from .domain.tracker import HealthTracker
from .trackers.arm_status import ArmStatusTracker
from .trackers.gravity_compensation import GravityCompensationTracker
from .trackers.can_bus import CanBusTracker
from .trackers.gripper import GripperTracker
from .trackers.joint_states import JointStatesTracker
from .trackers.per_joint import PerJointTracker
from .trackers.process import ProcessHealthTracker
from .trackers.serial_link import SerialLinkTracker


def build_trackers(
    p: dict[str, Any],
    *,
    sysfs: Optional[SysFsReader] = None,
    process_inspector: Optional[ProcessInspector] = None,
    device_inspector: Optional[DevicePathInspector] = None,
) -> list[HealthTracker]:
    """Build the ordered tracker list the orchestrator will own."""
    trackers: list[HealthTracker] = []

    if p["enable_joint_states_monitor"]:
        trackers.append(
            JointStatesTracker(
                topic=p["joint_states_topic"],
                params={
                    "expected_rate_hz": p["expected_rate_hz"],
                    "min_rate_ratio": p["min_rate_ratio"],
                    "stale_timeout_s": p["stale_timeout_s"],
                    "max_position_jump_rad": p["max_position_jump_rad"],
                    "max_abs_velocity_rad_s": p["max_abs_velocity_rad_s"],
                    "max_abs_effort_nm": p["max_abs_effort_nm"],
                },
            )
        )

    if p["enable_per_joint_monitor"]:
        pj_params = {
            "per_joint_stale_timeout_s": p["per_joint_stale_timeout_s"],
            "max_abs_joint_velocity_rad_s": p["max_abs_joint_velocity_rad_s"],
            "max_abs_joint_torque_nm": p["max_abs_joint_torque_nm"],
            "idle_velocity_threshold_rad_s": p["idle_velocity_threshold_rad_s"],
            "idle_torque_warn_nm": p["idle_torque_warn_nm"],
            "max_joint_position_jump_rad": p["max_joint_position_jump_rad"],
            "max_joint_torque_jump_nm": p["max_joint_torque_jump_nm"],
            "expected_enabled_status_code": p["expected_enabled_status_code"],
            "allow_disabled_status_code": p["allow_disabled_status_code"],
            "disabled_status_code": p["disabled_status_code"],
        }
        prefix = p["joint_state_topic_prefix"].rstrip("/")
        for name in p["joint_names"]:
            trackers.append(
                PerJointTracker(
                    joint_name=name,
                    topic=f"{prefix}/{name}/state",
                    params=pj_params,
                )
            )

    if p["enable_arm_status_monitor"]:
        arm_status_topic = p["arm_status_topic"]
        trackers.append(
            ArmStatusTracker(
                topic=arm_status_topic,
                params={
                    "arm_status_stale_timeout_s": p["arm_status_stale_timeout_s"],
                    "arm_status_warn_on_snapshot_age": p[
                        "arm_status_warn_on_snapshot_age"
                    ],
                    "expect_arm_enabled": p["expect_arm_enabled"],
                    "expect_control_loop_active": p["expect_control_loop_active"],
                },
            )
        )
        trackers.append(GravityCompensationTracker(topic=arm_status_topic))

    if p["enable_gripper_monitor"]:
        trackers.append(
            GripperTracker(
                topic=p["gripper_state_topic"],
                params={
                    "gripper_stale_timeout_s": p["gripper_stale_timeout_s"],
                    "max_abs_gripper_velocity": p["max_abs_gripper_velocity"],
                    "max_abs_gripper_torque": p["max_abs_gripper_torque"],
                },
            )
        )

    if p["enable_can_monitor"]:
        can_params = {
            "warn_on_iface_down": p["can_warn_on_iface_down"],
            "error_warn_per_period": p["can_error_warn_per_period"],
            "dropped_warn_per_period": p["can_dropped_warn_per_period"],
        }
        for iface in p["can_interfaces"]:
            iface = str(iface).strip()
            if not iface:
                continue
            trackers.append(CanBusTracker(iface, can_params, sysfs=sysfs))

    if p["enable_serial_monitor"]:
        device = str(p["serial_device"]).strip()
        if device:
            trackers.append(
                SerialLinkTracker(
                    {"device_path": device},
                    inspector=device_inspector,
                )
            )

    if p["enable_process_monitor"]:
        trackers.append(
            ProcessHealthTracker(
                {
                    "name_pattern": p["driver_process_pattern"],
                    "pid": p["driver_process_pid"],
                    "cpu_warn_percent": p["driver_cpu_warn_percent"],
                    "rss_warn_mb": p["driver_rss_warn_mb"],
                    "threads_warn": p["driver_threads_warn"],
                    "zombie_is_error": p["driver_zombie_is_error"],
                },
                inspector=process_inspector,
            )
        )

    return trackers
