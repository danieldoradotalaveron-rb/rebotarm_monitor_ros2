"""Tests for :func:`build_trackers` (composition root)."""

from __future__ import annotations

from rebotarm_monitor.adapters import FakeDevicePathInspector, FakeProcessInspector, FakeSysFsReader
from rebotarm_monitor.factories import build_trackers
from rebotarm_monitor.trackers.arm_status import ArmStatusTracker
from rebotarm_monitor.trackers.can_bus import CanBusTracker
from rebotarm_monitor.trackers.gripper import GripperTracker
from rebotarm_monitor.trackers.joint_states import JointStatesTracker
from rebotarm_monitor.trackers.per_joint import PerJointTracker
from rebotarm_monitor.trackers.process import ProcessHealthTracker
from rebotarm_monitor.trackers.serial_link import SerialLinkTracker


def base_params(**overrides) -> dict:
    params = {
        "enable_joint_states_monitor": True,
        "enable_per_joint_monitor": True,
        "enable_arm_status_monitor": True,
        "enable_gripper_monitor": True,
        "enable_can_monitor": False,
        "enable_process_monitor": False,
        "log_only_on_change": True,
        "joint_states_topic": "/rebotarm/joint_states",
        "expected_rate_hz": 100.0,
        "stale_timeout_s": 0.5,
        "min_rate_ratio": 0.5,
        "max_position_jump_rad": 0.5,
        "max_abs_velocity_rad_s": 10.0,
        "max_abs_effort_nm": 8.0,
        "joint_state_topic_prefix": "/rebotarm/joints",
        "joint_names": ["j1", "j2"],
        "per_joint_stale_timeout_s": 0.5,
        "max_abs_joint_velocity_rad_s": 10.0,
        "max_abs_joint_torque_nm": 8.0,
        "idle_velocity_threshold_rad_s": 0.05,
        "idle_torque_warn_nm": 3.0,
        "per_joint_max_abs_torque_nm": {},
        "per_joint_idle_torque_warn_nm": {},
        "max_joint_position_jump_rad": 0.5,
        "max_joint_torque_jump_nm": 3.0,
        "expected_enabled_status_code": 1,
        "allow_disabled_status_code": True,
        "disabled_status_code": 0,
        "arm_status_topic": "/rebotarm/arm_status",
        "arm_status_stale_timeout_s": 1.0,
        "arm_status_warn_on_snapshot_age": False,
        "expect_arm_enabled": False,
        "expect_control_loop_active": False,
        "gripper_state_topic": "/rebotarm/gripper/state",
        "gripper_stale_timeout_s": 1.0,
        "max_abs_gripper_velocity": 10.0,
        "max_abs_gripper_torque": 5.0,
        "can_interfaces": ["can0", "can1"],
        "can_warn_on_iface_down": True,
        "can_error_warn_per_period": 1,
        "can_dropped_warn_per_period": 10,
        "enable_serial_monitor": True,
        "serial_device": "/dev/ttyACM0",
        "driver_process_pattern": "reBotArmController",
        "driver_process_pid": 0,
        "driver_cpu_warn_percent": 90.0,
        "driver_rss_warn_mb": 1024.0,
        "driver_threads_warn": 64,
        "driver_zombie_is_error": True,
        "status_log_period_s": 1.0,
        "diagnostics_period_s": 1.0,
    }
    params.update(overrides)
    return params


def test_default_set_builds_one_per_concern():
    trackers = build_trackers(
        base_params(enable_serial_monitor=False),
        device_inspector=FakeDevicePathInspector(),
    )
    kinds = [type(t) for t in trackers]
    assert kinds.count(JointStatesTracker) == 1
    assert kinds.count(PerJointTracker) == 2
    assert kinds.count(ArmStatusTracker) == 1
    assert kinds.count(GripperTracker) == 1


def test_serial_link_added_when_enabled():
    params = base_params(enable_serial_monitor=True, serial_device="/dev/ttyACM0")
    trackers = build_trackers(
        params,
        device_inspector=FakeDevicePathInspector(exists={"/dev/ttyACM0"}),
    )
    kinds = [type(t) for t in trackers]
    assert kinds.count(SerialLinkTracker) == 1


def test_can_and_process_added_when_enabled():
    params = base_params(enable_can_monitor=True, enable_process_monitor=True)
    trackers = build_trackers(
        params,
        sysfs=FakeSysFsReader(),
        process_inspector=FakeProcessInspector(),
    )
    kinds = [type(t) for t in trackers]
    assert kinds.count(CanBusTracker) == 2
    assert kinds.count(ProcessHealthTracker) == 1


def _per_joint_trackers(trackers: list) -> list[PerJointTracker]:
    return [t for t in trackers if isinstance(t, PerJointTracker)]


def test_empty_override_maps_use_global_torque_thresholds():
    trackers = _per_joint_trackers(
        build_trackers(
            base_params(
                joint_names=["joint1", "joint2"],
                enable_serial_monitor=False,
                enable_arm_status_monitor=False,
                enable_gripper_monitor=False,
                enable_joint_states_monitor=False,
            )
        )
    )
    assert len(trackers) == 2
    for tracker in trackers:
        assert tracker.params["max_abs_joint_torque_nm"] == 8.0
        assert tracker.params["idle_torque_warn_nm"] == 3.0


def test_per_joint_max_torque_override():
    trackers = _per_joint_trackers(
        build_trackers(
            base_params(
                joint_names=["joint1", "joint3"],
                per_joint_max_abs_torque_nm={"joint3": 10.0},
                enable_serial_monitor=False,
                enable_arm_status_monitor=False,
                enable_gripper_monitor=False,
                enable_joint_states_monitor=False,
            )
        )
    )
    by_name = {t.joint_name: t for t in trackers}
    assert by_name["joint1"].params["max_abs_joint_torque_nm"] == 8.0
    assert by_name["joint3"].params["max_abs_joint_torque_nm"] == 10.0


def test_per_joint_idle_torque_override():
    trackers = _per_joint_trackers(
        build_trackers(
            base_params(
                joint_names=["joint1", "joint3"],
                per_joint_idle_torque_warn_nm={"joint3": 5.0},
                enable_serial_monitor=False,
                enable_arm_status_monitor=False,
                enable_gripper_monitor=False,
                enable_joint_states_monitor=False,
            )
        )
    )
    by_name = {t.joint_name: t for t in trackers}
    assert by_name["joint1"].params["idle_torque_warn_nm"] == 3.0
    assert by_name["joint3"].params["idle_torque_warn_nm"] == 5.0


def test_each_per_joint_tracker_gets_distinct_params_dict():
    trackers = _per_joint_trackers(
        build_trackers(
            base_params(
                joint_names=["joint1", "joint3"],
                per_joint_max_abs_torque_nm={"joint3": 10.0},
                enable_serial_monitor=False,
                enable_arm_status_monitor=False,
                enable_gripper_monitor=False,
                enable_joint_states_monitor=False,
            )
        )
    )
    assert trackers[0].params is not trackers[1].params
    assert trackers[0].params["max_abs_joint_torque_nm"] == 8.0
    assert trackers[1].params["max_abs_joint_torque_nm"] == 10.0


def test_disabling_concerns_drops_trackers():
    params = base_params(
        enable_joint_states_monitor=False,
        enable_per_joint_monitor=False,
        enable_arm_status_monitor=False,
        enable_gripper_monitor=False,
        enable_serial_monitor=False,
    )
    trackers = build_trackers(params)
    assert trackers == []
