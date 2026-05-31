"""Tests for :class:`GripperTracker`."""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticStatus

from rebotarm_monitor.domain.tracker import TrackerContext
from rebotarm_monitor.trackers.gripper import GripperTracker
from conftest import make_motor_state


DEFAULT_PARAMS = {
    "gripper_stale_timeout_s": 1.0,
    "max_abs_gripper_velocity": 10.0,
    "max_abs_gripper_torque": 5.0,
}


def make_tracker() -> GripperTracker:
    return GripperTracker("/rebotarm/gripper/state", dict(DEFAULT_PARAMS))


def test_no_messages_yields_warn():
    tracker = make_tracker()
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN


def test_healthy_message_yields_ok():
    tracker = make_tracker()
    tracker.on_message(make_motor_state())
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.OK


def test_high_velocity_yields_warn():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(velocity=50.0))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "high gripper velocity" in status.message


def test_non_finite_yields_error():
    tracker = make_tracker()
    tracker.on_message(make_motor_state(position=float("inf")))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR
