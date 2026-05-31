from __future__ import annotations

import math

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def is_finite(value: float) -> bool:
    return math.isfinite(value)


def kv(key: str, value) -> KeyValue:
    entry = KeyValue()
    entry.key = key
    entry.value = str(value)
    return entry


def max_level(a: bytes, b: bytes) -> bytes:
    order = {
        DiagnosticStatus.OK: 0,
        DiagnosticStatus.WARN: 1,
        DiagnosticStatus.ERROR: 2,
        DiagnosticStatus.STALE: 3,
    }
    return a if order.get(a, 0) >= order.get(b, 0) else b
