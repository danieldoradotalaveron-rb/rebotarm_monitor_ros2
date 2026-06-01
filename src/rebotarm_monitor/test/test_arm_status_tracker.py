"""Tests for :class:`ArmStatusTracker`."""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticStatus

from rebotarm_monitor.domain.tracker import TrackerContext
from rebotarm_monitor.trackers.arm_status import ArmStatusTracker
from conftest import make_arm_status


DEFAULT_PARAMS = {
    "arm_status_stale_timeout_s": 1.0,
    "arm_status_warn_on_snapshot_age": False,
    "expect_arm_enabled": False,
    "expect_control_loop_active": False,
}


def make_tracker(**overrides) -> ArmStatusTracker:
    params = dict(DEFAULT_PARAMS)
    params.update(overrides)
    return ArmStatusTracker("/rebotarm/arm_status", params)


def test_no_messages_yields_error():
    tracker = make_tracker()
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR


def test_healthy_message_yields_ok():
    tracker = make_tracker()
    tracker.on_message(make_arm_status(enabled=True))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.name == "rebotarm/control/arm_status"
    assert status.level == DiagnosticStatus.OK
    assert status.message == "active mode: 0, state: 0"


def test_arm_enabled_property_reflects_message():
    tracker = make_tracker()
    assert tracker.arm_enabled is None
    tracker.on_message(make_arm_status(enabled=False))
    assert tracker.arm_enabled is False
    tracker.on_message(make_arm_status(enabled=True))
    assert tracker.arm_enabled is True


def test_error_codes_yields_error():
    tracker = make_tracker()
    tracker.on_message(make_arm_status(enabled=True, error_codes=[42]))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR
    assert "error_codes" in status.message
    assert tracker.last_warning_reason == "error_codes"


def test_expect_arm_enabled_warns_when_disabled():
    tracker = make_tracker(expect_arm_enabled=True)
    tracker.on_message(make_arm_status(enabled=False))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "arm disabled" in status.message


def test_expect_control_loop_warns_when_inactive():
    tracker = make_tracker(expect_control_loop_active=True)
    tracker.on_message(make_arm_status(enabled=True, control_loop_active=False))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "control loop inactive" in status.message


def test_snapshot_age_warning_only_when_enabled():
    tracker = make_tracker(arm_status_warn_on_snapshot_age=True)
    tracker.on_message(make_arm_status(enabled=True))
    tracker.last_recv_mono = time.monotonic() - 10.0
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "snapshot age" in status.message
