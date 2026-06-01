"""Tests for :class:`MonitorConfigTracker`."""

from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticStatus

from rebotarm_monitor.domain.tracker import TrackerContext
from rebotarm_monitor.factories import build_trackers
from rebotarm_monitor.trackers.monitor_config import MonitorConfigTracker
from test_factories import base_params


def test_build_status_reports_payload_profile_in_message():
    tracker = MonitorConfigTracker(
        {"payload_profile": "medium", "assumed_payload_kg": 1.0}
    )
    status = tracker.build_status(0.0, TrackerContext())
    assert status.level == DiagnosticStatus.OK
    assert status.message == "payload profile: medium (1 kg assumed payload)"


def test_keyvalues_expose_profile_fields():
    tracker = MonitorConfigTracker(
        {"payload_profile": "rated", "assumed_payload_kg": 1.5}
    )
    status = tracker.build_status(0.0, TrackerContext())
    values = {entry.key: entry.value for entry in status.values}
    assert values["payload_profile"] == "rated"
    assert values["assumed_payload_kg"] == "1.5"


def test_factory_always_adds_monitor_config_tracker():
    trackers = build_trackers(
        base_params(
            enable_joint_states_monitor=False,
            enable_per_joint_monitor=False,
            enable_arm_status_monitor=False,
            enable_gripper_monitor=False,
            enable_serial_monitor=False,
            enable_process_monitor=False,
            payload_profile="light",
        )
    )
    config_trackers = [t for t in trackers if isinstance(t, MonitorConfigTracker)]
    assert len(config_trackers) == 1
    status = config_trackers[0].build_status(0.0, TrackerContext())
    assert "payload profile: light" in status.message
