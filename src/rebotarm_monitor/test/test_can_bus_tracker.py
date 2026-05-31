"""Tests for :class:`CanBusTracker` using the in-memory sysfs fake."""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticStatus

from rebotarm_monitor.adapters import FakeSysFsReader
from rebotarm_monitor.domain.tracker import TrackerContext
from rebotarm_monitor.trackers.can_bus import CanBusTracker


PARAMS = {
    "warn_on_iface_down": True,
    "error_warn_per_period": 1,
    "dropped_warn_per_period": 10,
}


def _bring_up(fake: FakeSysFsReader, iface: str = "can0", **stats: int) -> None:
    base = f"/sys/class/net/{iface}"
    fake.set(f"{base}/operstate", "up\n")
    fake.set(f"{base}/carrier", "1\n")
    for field in (
        "rx_packets",
        "tx_packets",
        "rx_bytes",
        "tx_bytes",
        "rx_errors",
        "tx_errors",
        "rx_dropped",
        "tx_dropped",
    ):
        fake.set(f"{base}/statistics/{field}", str(stats.get(field, 0)))


def test_missing_interface_yields_error():
    fake = FakeSysFsReader()
    tracker = CanBusTracker("can0", PARAMS, sysfs=fake)
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR
    assert "not present" in status.message


def test_healthy_bus_is_ok_on_second_sample():
    fake = FakeSysFsReader()
    _bring_up(fake)
    tracker = CanBusTracker("can0", PARAMS, sysfs=fake)

    first = tracker.build_status(time.monotonic(), TrackerContext())
    assert first.level == DiagnosticStatus.OK

    second = tracker.build_status(time.monotonic() + 1.0, TrackerContext())
    assert second.level == DiagnosticStatus.OK


def test_bus_errors_yield_error():
    fake = FakeSysFsReader()
    _bring_up(fake)
    tracker = CanBusTracker("can0", PARAMS, sysfs=fake)
    tracker.build_status(time.monotonic(), TrackerContext())

    _bring_up(fake, rx_errors=2)
    status = tracker.build_status(time.monotonic() + 1.0, TrackerContext())
    assert status.level == DiagnosticStatus.ERROR
    assert "bus error" in status.message
    assert tracker.last_warning_reason == "bus_errors"


def test_dropped_frames_yield_warn():
    fake = FakeSysFsReader()
    _bring_up(fake)
    tracker = CanBusTracker("can0", PARAMS, sysfs=fake)
    tracker.build_status(time.monotonic(), TrackerContext())

    _bring_up(fake, rx_dropped=15)
    status = tracker.build_status(time.monotonic() + 1.0, TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "dropped frame" in status.message
    assert tracker.last_warning_reason == "dropped_frames"


def test_iface_down_yields_warn():
    fake = FakeSysFsReader()
    _bring_up(fake)
    fake.set("/sys/class/net/can0/operstate", "down\n")
    tracker = CanBusTracker("can0", PARAMS, sysfs=fake)
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "operstate=down" in status.message
