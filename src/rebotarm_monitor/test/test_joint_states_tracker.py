"""Tests for :class:`JointStatesTracker` (whole-arm joint_states health)."""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticStatus

from rebotarm_monitor.domain.tracker import TrackerContext
from rebotarm_monitor.trackers.joint_states import JointStatesTracker
from conftest import make_joint_state


DEFAULT_PARAMS = {
    "expected_rate_hz": 100.0,
    "min_rate_ratio": 0.5,
    "stale_timeout_s": 0.5,
    "max_position_jump_rad": 0.5,
    "max_abs_velocity_rad_s": 10.0,
    "max_abs_effort_nm": 8.0,
}


def make_tracker() -> JointStatesTracker:
    return JointStatesTracker("/rebotarm/joint_states", dict(DEFAULT_PARAMS))


def test_no_messages_yields_error():
    tracker = make_tracker()
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR
    assert "no joint_states" in status.message


def test_healthy_path_returns_ok():
    tracker = make_tracker()
    names = ["j1", "j2"]
    for _ in range(60):
        tracker.on_message(
            make_joint_state(names, positions=[0.0, 0.0], velocities=[0.0, 0.0], efforts=[0.0, 0.0])
        )
        tracker.rate.msg_count = 60
    tracker.rate.window_start = time.monotonic() - 1.0
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.OK
    assert status.message == "joint_states healthy"


def test_stale_yields_error():
    tracker = make_tracker()
    tracker.on_message(make_joint_state(["j1"], [0.0], [0.0], [0.0]))
    tracker.last_recv_mono = time.monotonic() - 10.0
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR
    assert "stale data" in status.message
    assert tracker.last_warning_reason == "stale"


def test_non_finite_position_yields_error():
    tracker = make_tracker()
    tracker.on_message(
        make_joint_state(["j1"], [float("nan")], [0.0], [0.0])
    )
    tracker.rate.msg_count = 60
    tracker.rate.window_start = time.monotonic() - 1.0
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR
    assert tracker.last_warning_reason == "non_finite"


def test_low_rate_yields_warn():
    tracker = make_tracker()
    tracker.on_message(make_joint_state(["j1"], [0.0], [0.0], [0.0]))
    tracker.rate.msg_count = 1
    tracker.rate.window_start = time.monotonic() - 1.0
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "low rate" in status.message
    assert tracker.last_warning_reason == "low_rate"


def test_position_jump_yields_warn():
    tracker = make_tracker()
    tracker.on_message(make_joint_state(["j1"], [0.0], [0.0], [0.0]))
    tracker.on_message(make_joint_state(["j1"], [5.0], [0.0], [0.0]))
    tracker.rate.msg_count = 60
    tracker.rate.window_start = time.monotonic() - 1.0
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "position jump" in status.message


def test_high_velocity_yields_warn():
    tracker = make_tracker()
    tracker.on_message(make_joint_state(["j1"], [0.0], [50.0], [0.0]))
    tracker.rate.msg_count = 60
    tracker.rate.window_start = time.monotonic() - 1.0
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "high velocity" in status.message


def test_missing_velocity_array_yields_warn():
    tracker = make_tracker()
    tracker.on_message(make_joint_state(["j1"], [0.0], [], [0.0]))
    tracker.rate.msg_count = 60
    tracker.rate.window_start = time.monotonic() - 1.0
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "missing velocity" in status.message


def test_reset_period_clears_state():
    tracker = make_tracker()
    tracker.on_message(make_joint_state(["j1"], [0.0], [50.0], [0.0]))
    assert tracker.period_high_vel
    tracker.reset_period()
    assert not tracker.period_high_vel
    assert tracker.max_abs_vel_observed == 0.0
