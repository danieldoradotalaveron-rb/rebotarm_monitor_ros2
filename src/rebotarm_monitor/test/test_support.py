"""Unit tests for the support layer (diagnostics helpers, rate window)."""

from __future__ import annotations

import math
import time

from diagnostic_msgs.msg import DiagnosticStatus

from rebotarm_monitor.support import RateWindow, is_finite, kv, max_level


def test_is_finite_handles_nan_and_inf():
    assert is_finite(0.0) is True
    assert is_finite(1e9) is True
    assert is_finite(float("nan")) is False
    assert is_finite(float("inf")) is False
    assert is_finite(-math.inf) is False


def test_kv_coerces_value_to_string():
    entry = kv("foo", 42)
    assert entry.key == "foo"
    assert entry.value == "42"


def test_max_level_prefers_more_severe():
    assert max_level(DiagnosticStatus.OK, DiagnosticStatus.WARN) == DiagnosticStatus.WARN
    assert (
        max_level(DiagnosticStatus.ERROR, DiagnosticStatus.WARN)
        == DiagnosticStatus.ERROR
    )
    assert max_level(DiagnosticStatus.OK, DiagnosticStatus.OK) == DiagnosticStatus.OK


def test_rate_window_measures_hz_and_resets():
    window = RateWindow()
    window.msg_count = 10
    window.window_start = time.monotonic() - 1.0
    hz = window.measured_hz()
    assert 8.0 < hz < 12.0
    window.reset()
    assert window.msg_count == 0
    assert window.measured_hz() == 0.0 or window.measured_hz() < 1e6
