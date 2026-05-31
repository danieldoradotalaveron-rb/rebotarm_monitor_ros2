"""Tests for :class:`ProcessHealthTracker` using the fake inspector."""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticStatus

from rebotarm_monitor.adapters import (
    FakeProcessInspector,
    ProcessSnapshot,
)
from rebotarm_monitor.adapters.process_info import STATUS_ZOMBIE
from rebotarm_monitor.domain.tracker import TrackerContext
from rebotarm_monitor.trackers.process import ProcessHealthTracker


PARAMS = {
    "name_pattern": "reBotArmController",
    "pid": 0,
    "cpu_warn_percent": 90.0,
    "rss_warn_mb": 1024.0,
    "threads_warn": 64,
    "zombie_is_error": True,
}


def snap(**overrides) -> ProcessSnapshot:
    base = dict(
        pid=4242,
        status="running",
        cpu_percent=10.0,
        rss_mb=128.0,
        num_threads=8,
        num_fds=42,
        create_time=time.time() - 60.0,
    )
    base.update(overrides)
    return ProcessSnapshot(**base)


def test_missing_psutil_yields_warn():
    inspector = FakeProcessInspector(snapshot=None)
    inspector.set_available(False)
    tracker = ProcessHealthTracker(PARAMS, inspector=inspector)
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "psutil" in status.message


def test_process_missing_yields_error():
    inspector = FakeProcessInspector(snapshot=None)
    tracker = ProcessHealthTracker(PARAMS, inspector=inspector)
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR
    assert "not found" in status.message


def test_healthy_process_is_ok():
    inspector = FakeProcessInspector(snapshot=snap())
    tracker = ProcessHealthTracker(PARAMS, inspector=inspector)
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.OK


def test_high_cpu_yields_warn():
    inspector = FakeProcessInspector(snapshot=snap(cpu_percent=99.5))
    tracker = ProcessHealthTracker(PARAMS, inspector=inspector)
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "high CPU" in status.message


def test_high_memory_yields_warn():
    inspector = FakeProcessInspector(snapshot=snap(rss_mb=2048.0))
    tracker = ProcessHealthTracker(PARAMS, inspector=inspector)
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.WARN
    assert "high memory" in status.message


def test_zombie_yields_error():
    inspector = FakeProcessInspector(snapshot=snap(status=STATUS_ZOMBIE))
    tracker = ProcessHealthTracker(PARAMS, inspector=inspector)
    status = tracker.build_status(time.monotonic(), TrackerContext())
    assert status.level == DiagnosticStatus.ERROR
    assert "status=zombie" in status.message
