"""Unit tests for SerialLinkTracker."""

import time

from diagnostic_msgs.msg import DiagnosticStatus

from rebotarm_monitor.adapters.device_path import FakeDevicePathInspector
from rebotarm_monitor.domain.tracker import TrackerContext
from rebotarm_monitor.trackers.serial_link import SerialLinkTracker


def test_serial_link_ok_when_device_present():
    inspector = FakeDevicePathInspector(exists={"/dev/ttyACM0"})
    tracker = SerialLinkTracker({"device_path": "/dev/ttyACM0"}, inspector=inspector)
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.name == "rebotarm/link/serial"
    assert status.level == DiagnosticStatus.OK
    assert "/dev/ttyACM0" in status.message


def test_serial_link_error_when_device_missing():
    inspector = FakeDevicePathInspector(exists=set())
    tracker = SerialLinkTracker({"device_path": "/dev/ttyACM0"}, inspector=inspector)
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR
    assert "not present" in status.message.lower()


def test_serial_link_warn_when_path_empty():
    inspector = FakeDevicePathInspector(exists={"/dev/ttyACM0"})
    tracker = SerialLinkTracker({"device_path": "  "}, inspector=inspector)
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "not configured" in status.message.lower()
