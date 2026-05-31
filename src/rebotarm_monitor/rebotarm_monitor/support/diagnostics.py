"""Tiny helpers used across trackers to build ``diagnostic_msgs`` payloads."""

from __future__ import annotations

import math

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def is_finite(value: float) -> bool:
    """Return ``True`` only if ``value`` is a finite real number."""
    return math.isfinite(value)


def kv(key: str, value) -> KeyValue:  # noqa: ANN001
    """Build a ``KeyValue`` entry, coercing ``value`` to ``str``."""
    entry = KeyValue()
    entry.key = key
    entry.value = str(value)
    return entry


_LEVEL_ORDER = {
    DiagnosticStatus.OK: 0,
    DiagnosticStatus.WARN: 1,
    DiagnosticStatus.ERROR: 2,
    DiagnosticStatus.STALE: 3,
}


def max_level(a: bytes, b: bytes) -> bytes:
    """Return the more severe of two ``DiagnosticStatus`` levels."""
    return a if _LEVEL_ORDER.get(a, 0) >= _LEVEL_ORDER.get(b, 0) else b
