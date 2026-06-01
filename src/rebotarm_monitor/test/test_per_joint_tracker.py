"""Tests for :class:`PerJointTracker`."""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticStatus

from rebotarm_monitor.domain.tracker import TrackerContext
from rebotarm_monitor.trackers.per_joint import PerJointTracker
from conftest import make_motor_state


DEFAULT_PARAMS = {
    "per_joint_stale_timeout_s": 0.5,
    "max_abs_joint_velocity_rad_s": 10.0,
    "max_abs_joint_torque_nm": 8.0,
    "idle_velocity_threshold_rad_s": 0.05,
    "idle_torque_warn_nm": 3.0,
    "max_joint_position_jump_rad": 0.5,
    "max_joint_torque_jump_nm": 3.0,
    "expected_enabled_status_code": 1,
    "allow_disabled_status_code": True,
    "disabled_status_code": 0,
}


def make_tracker() -> PerJointTracker:
    return PerJointTracker("j1", "/rebotarm/joints/j1/state", dict(DEFAULT_PARAMS))


def kv_map(status) -> dict[str, str]:  # noqa: ANN001
    return {entry.key: entry.value for entry in status.values}


def hold_context(
    *,
    gravity: bool = False,
    position_hold: bool = False,
    control_context: str = "normal_or_unknown",
) -> TrackerContext:
    return TrackerContext(
        gravity_compensation_active=gravity,
        position_hold_active=position_hold,
        control_context=control_context,
    )


def test_no_messages_yields_error():
    tracker = make_tracker()
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR


def test_enabled_motor_is_ok():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(status_code=1))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.OK
    assert "|T|=0.0/8.0 Nm" in status.message
    assert "|v|=0.0/10.0 rad/s" in status.message


def test_disabled_motor_with_arm_enabled_yields_warn():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(status_code=0))
    status = tracker.build_status(time.monotonic(), TrackerContext(arm_enabled=True))
    assert status.level == DiagnosticStatus.WARN
    assert "disabled while arm enabled" in status.message


def test_disabled_motor_with_arm_disabled_is_ok():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(status_code=0))
    status = tracker.build_status(time.monotonic(), TrackerContext(arm_enabled=False))
    assert status.level == DiagnosticStatus.OK


def test_unexpected_status_code_yields_warn():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(status_code=99))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "unexpected status_code" in status.message


def test_position_jump_yields_warn():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(position=0.0))
    tracker.on_message(make_motor_state(position=2.0))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "position jump" in status.message


def test_high_torque_yields_warn():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(torque=20.0))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "high torque" in status.message
    assert "|T|=20.0/8.0 Nm" in status.message
    assert "|v|=0.0/10.0 rad/s" in status.message


def test_payload_profile_keyvalue_defaults_to_light():
    """No payload_profile in params falls back to the package default for visibility."""
    tracker = make_tracker()
    tracker.on_message(make_motor_state(status_code=1))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert kv_map(status)["payload_profile"] == "light"


def test_payload_profile_keyvalue_reflects_active_profile():
    """When the orchestrator propagates a profile, the per-joint status surfaces it."""
    params = dict(DEFAULT_PARAMS, payload_profile="rated")
    tracker = PerJointTracker("j1", "/rebotarm/joints/j1/state", params)
    tracker.on_message(make_motor_state(status_code=1))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert kv_map(status)["payload_profile"] == "rated"


def test_idle_torque_warning_triggers_in_normal_context():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(velocity=0.0, torque=5.0))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "high stationary effort" in status.message
    assert "|T|=5.0/8.0 Nm" in status.message
    assert "|v|=0.0/10.0 rad/s" in status.message
    assert "idle threshold 5.0/3.0 Nm" in status.message
    assert tracker.last_warning_reason == "idle_torque"
    values = kv_map(status)
    assert values["control_context"] == "normal_or_unknown"
    assert values["load_state"] == "elevated"
    assert values["stationary_effort_check_suppressed"] == "False"


def test_idle_torque_suppressed_during_gravity_compensation():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(velocity=0.0, torque=5.0))
    status = tracker.build_status(
        time.monotonic(),
        hold_context(
            gravity=True,
            control_context="gravity_compensation",
        ),
    )
    assert status.level == DiagnosticStatus.OK
    assert "high stationary effort" not in status.message
    assert "high torque while idle" not in status.message
    assert tracker.last_warning_reason != "idle_torque"
    values = kv_map(status)
    assert values["control_context"] == "gravity_compensation"
    assert values["load_state"] == "elevated"
    assert values["stationary_effort_check_suppressed"] == "True"


def test_idle_torque_suppressed_during_position_hold():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(velocity=0.0, torque=5.0))
    status = tracker.build_status(
        time.monotonic(),
        hold_context(
            position_hold=True,
            control_context="position_hold",
        ),
    )
    assert status.level == DiagnosticStatus.OK
    assert "high stationary effort" not in status.message
    assert "high torque while idle" not in status.message
    values = kv_map(status)
    assert values["control_context"] == "position_hold"
    assert values["load_state"] == "elevated"
    assert values["stationary_effort_check_suppressed"] == "True"
    assert "|T|=5.0/8.0 Nm" in status.message
    assert "/3.0 Nm" not in status.message
    assert "position hold" in status.message
    assert "idle-torque warning suppressed at 3.0 Nm" in status.message


def test_suppressed_message_uses_max_torque_in_gravity_compensation():
    """OK suppressed branch must compare |T| to max_abs_joint_torque_nm, not idle threshold."""
    tracker = make_tracker()
    tracker.on_message(make_motor_state(velocity=0.0, torque=5.0))
    status = tracker.build_status(
        time.monotonic(),
        hold_context(
            gravity=True,
            control_context="gravity_compensation",
        ),
    )
    assert status.level == DiagnosticStatus.OK
    assert "|T|=5.0/8.0 Nm" in status.message
    assert "/3.0 Nm" not in status.message
    assert "gravity compensation" in status.message
    assert "idle-torque warning suppressed at 3.0 Nm" in status.message


def test_absolute_high_torque_still_warns_during_gravity_compensation():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(velocity=0.0, torque=20.0))
    status = tracker.build_status(
        time.monotonic(),
        hold_context(
            gravity=True,
            control_context="gravity_compensation",
        ),
    )
    assert status.level == DiagnosticStatus.WARN
    assert "high torque" in status.message
    assert "|T|=20.0/8.0 Nm" in status.message
    assert "|v|=0.0/10.0 rad/s" in status.message


def test_absolute_high_torque_still_warns_during_position_hold():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(velocity=0.0, torque=20.0))
    status = tracker.build_status(
        time.monotonic(),
        hold_context(
            position_hold=True,
            control_context="position_hold",
        ),
    )
    assert status.level == DiagnosticStatus.WARN
    assert "high torque" in status.message
    assert "|T|=20.0/8.0 Nm" in status.message
    assert "|v|=0.0/10.0 rad/s" in status.message


def test_torque_jump_yields_warn():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(torque=0.0))
    tracker.on_message(make_motor_state(torque=5.0))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "torque jump" in status.message
    assert tracker.last_warning_reason == "torque_jump"


def test_idle_torque_uses_resolved_threshold_from_params():
    params = dict(DEFAULT_PARAMS)
    params["idle_torque_warn_nm"] = 5.0
    tracker = PerJointTracker("j1", "/rebotarm/joints/j1/state", params)
    tracker.on_message(make_motor_state(velocity=0.0, torque=4.0))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.OK

    tracker.on_message(make_motor_state(velocity=0.0, torque=5.5))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "high stationary effort" in status.message


def test_non_finite_values_yield_error():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(position=float("nan")))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR
    assert "non-finite" in status.message


def test_stale_yields_error():
    tracker = make_tracker()
    tracker.on_message(make_motor_state())
    tracker.last_recv_mono = time.monotonic() - 10.0
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR
    assert "stale" in status.message


def test_reset_period_clears_flags():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(velocity=50.0, torque=20.0))
    assert tracker.period_peak_abs_velocity_rad_s == 50.0
    assert tracker.period_peak_abs_torque_nm == 20.0
    tracker.reset_period()
    assert tracker.period_peak_abs_velocity_rad_s == 0.0
    assert tracker.period_peak_abs_torque_nm == 0.0


def test_period_torque_spike_does_not_warn_when_current_sample_recovered():
    """Level follows last_msg, not a latched flag from an earlier sample."""
    params = dict(DEFAULT_PARAMS)
    params["max_abs_joint_torque_nm"] = 9.0
    params["idle_torque_warn_nm"] = 8.0
    tracker = PerJointTracker("joint2", "/rebotarm/joints/joint2/state", params)
    tracker.on_message(make_motor_state(velocity=0.0, torque=10.0))
    tracker.on_message(make_motor_state(velocity=0.0, torque=8.6))
    status = tracker.build_status(
        time.monotonic(),
        hold_context(
            gravity=True,
            control_context="gravity_compensation",
        ),
    )
    assert status.level == DiagnosticStatus.OK
    assert "idle-torque warning suppressed" in status.message
    assert "high torque" not in status.message
    values = kv_map(status)
    assert values["peak_abs_torque_nm_this_period"] == "10.000"


def test_high_torque_during_gravity_comp_includes_context():
    params = dict(DEFAULT_PARAMS)
    params["max_abs_joint_torque_nm"] = 9.0
    tracker = PerJointTracker("joint2", "/rebotarm/joints/joint2/state", params)
    tracker.on_message(make_motor_state(velocity=0.0, torque=9.5))
    status = tracker.build_status(
        time.monotonic(),
        hold_context(
            gravity=True,
            control_context="gravity_compensation",
        ),
    )
    assert status.level == DiagnosticStatus.WARN
    assert "high torque" in status.message
    assert "gravity compensation" in status.message
    assert "|v|=0.0/10.0 rad/s" in status.message
    values = kv_map(status)
    assert values["stationary_effort_check_suppressed"] == "True"
    assert values["load_state"] == "elevated"


def test_elevated_stationary_warns_when_gravity_comp_stops():
    params = dict(DEFAULT_PARAMS)
    params["max_abs_joint_torque_nm"] = 9.0
    params["idle_torque_warn_nm"] = 8.0
    tracker = PerJointTracker("joint2", "/rebotarm/joints/joint2/state", params)
    tracker.on_message(make_motor_state(velocity=0.0, torque=8.6))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "high stationary effort" in status.message
    values = kv_map(status)
    assert values["stationary_effort_check_suppressed"] == "False"
    assert values["control_context"] == "normal_or_unknown"


def test_high_velocity_during_position_hold_includes_context():
    params = dict(DEFAULT_PARAMS)
    params["max_abs_joint_velocity_rad_s"] = 6.0
    tracker = PerJointTracker("joint2", "/rebotarm/joints/joint2/state", params)
    tracker.on_message(make_motor_state(velocity=7.0, torque=1.0))
    status = tracker.build_status(
        time.monotonic(),
        hold_context(
            position_hold=True,
            control_context="position_hold",
        ),
    )
    assert status.level == DiagnosticStatus.WARN
    assert "high velocity" in status.message
    assert "position hold" in status.message
    assert "|T|=1.0/8.0 Nm" in status.message
