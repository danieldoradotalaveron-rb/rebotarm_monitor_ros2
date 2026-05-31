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


def test_no_messages_yields_error():
    tracker = make_tracker()
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR


def test_enabled_motor_is_ok():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(status_code=1))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.OK


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


def test_idle_torque_warning_triggers_when_arm_still():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(velocity=0.0, torque=5.0))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "high torque while idle" in status.message


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
    assert tracker.period_high_vel
    assert tracker.period_high_torque
    tracker.reset_period()
    assert not tracker.period_high_vel
    assert not tracker.period_high_torque
