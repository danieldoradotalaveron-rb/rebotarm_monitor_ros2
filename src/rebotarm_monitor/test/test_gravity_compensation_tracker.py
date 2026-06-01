"""Tests for :class:`GravityCompensationTracker`."""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticStatus

from rebotarm_monitor.domain.tracker import TrackerContext
from rebotarm_monitor.trackers.gravity_compensation import GravityCompensationTracker
from conftest import make_arm_status


def make_tracker() -> GravityCompensationTracker:
    return GravityCompensationTracker("/rebotarm/arm_status")


def test_diag_name():
    tracker = make_tracker()
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.name == "rebotarm/control/gravity_compensation"


def test_no_messages_yields_error():
    tracker = make_tracker()
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR
    assert status.message == "no arm_status received"


def test_gravity_comp_state_active():
    tracker = make_tracker()
    tracker.on_message(make_arm_status(state_machine="GRAVITY_COMP"))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.OK
    assert status.message == "gravity compensation: active"


def test_idle_state_inactive():
    tracker = make_tracker()
    tracker.on_message(make_arm_status(state_machine="IDLE"))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.OK
    assert status.message == "gravity compensation: inactive"


def test_mit_mode_without_gravity_comp_state_is_inactive():
    tracker = make_tracker()
    tracker.on_message(make_arm_status(mode="mit", state_machine="IDLE"))
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.OK
    assert status.message == "gravity compensation: inactive"
