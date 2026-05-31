"""Driver-process health tracker (CPU / memory / threads / FDs)."""

from __future__ import annotations

import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from ..adapters.process_info import (
    STATUS_STOPPED,
    STATUS_ZOMBIE,
    ProcessInspector,
    PsutilProcessInspector,
)
from ..domain.tracker import HealthTracker, TrackerContext
from ..support.diagnostics import kv, max_level


class ProcessHealthTracker(HealthTracker):
    """Tracks the driver process via an injected :class:`ProcessInspector`."""

    DIAG_NAME = "rebotarm/system/driver"

    def __init__(
        self,
        params: dict,
        inspector: Optional[ProcessInspector] = None,
    ) -> None:
        self.params = params
        self._inspector: ProcessInspector = inspector or PsutilProcessInspector()
        self._last_snapshot = None
        self.last_warning_reason = ""

    @property
    def diag_name(self) -> str:
        return self.DIAG_NAME

    def build_status(self, now: float, context: TrackerContext) -> DiagnosticStatus:
        level = DiagnosticStatus.OK
        message = "driver process healthy"
        reason = ""
        values: list[KeyValue] = [
            kv("name_pattern", self.params.get("name_pattern", "")),
            kv("pid_override", int(self.params.get("pid", 0))),
        ]

        if not self._inspector.available():
            level = DiagnosticStatus.WARN
            message = "psutil not installed; process check disabled"
            reason = "psutil_missing"
            self.last_warning_reason = reason
            values.append(kv("last_warning_reason", reason))
            return self._build(level, message, values)

        snap = None
        if self._last_snapshot is not None:
            snap = self._inspector.refresh(self._last_snapshot.pid)
        if snap is None:
            snap = self._inspector.find(
                str(self.params.get("name_pattern", "")),
                int(self.params.get("pid", 0)),
            )
        self._last_snapshot = snap

        if snap is None:
            level = DiagnosticStatus.ERROR
            message = "driver process not found"
            reason = "process_missing"
            self.last_warning_reason = reason
            values.append(kv("last_warning_reason", reason))
            return self._build(level, message, values)

        uptime_s = max(time.time() - snap.create_time, 0.0)
        values.extend(
            [
                kv("pid", snap.pid),
                kv("status", snap.status),
                kv("cpu_percent", f"{snap.cpu_percent:.1f}"),
                kv("rss_mb", f"{snap.rss_mb:.1f}"),
                kv("num_threads", snap.num_threads),
                kv("num_fds", snap.num_fds),
                kv("uptime_s", f"{uptime_s:.0f}"),
            ]
        )

        cpu_warn = float(self.params.get("cpu_warn_percent", 90.0))
        rss_warn = float(self.params.get("rss_warn_mb", 1024.0))
        threads_warn = int(self.params.get("threads_warn", 64))
        zombie_is_error = bool(self.params.get("zombie_is_error", True))

        if zombie_is_error and snap.status in (STATUS_ZOMBIE, STATUS_STOPPED):
            level = DiagnosticStatus.ERROR
            message = f"driver process status={snap.status}"
            reason = f"process_{snap.status}"
        else:
            if snap.cpu_percent > cpu_warn:
                level = max_level(level, DiagnosticStatus.WARN)
                message = f"high CPU {snap.cpu_percent:.0f}% (>{cpu_warn:.0f}%)"
                reason = "high_cpu"
            if snap.rss_mb > rss_warn:
                level = max_level(level, DiagnosticStatus.WARN)
                message = f"high memory {snap.rss_mb:.0f} MB (>{rss_warn:.0f} MB)"
                reason = "high_memory"
            if snap.num_threads > threads_warn:
                level = max_level(level, DiagnosticStatus.WARN)
                message = (
                    f"high thread count {snap.num_threads} (>{threads_warn})"
                )
                reason = "high_threads"

        if reason:
            self.last_warning_reason = reason
        values.append(kv("last_warning_reason", self.last_warning_reason or "none"))

        return self._build(level, message, values)

    def _build(
        self,
        level: bytes,
        message: str,
        values: list[KeyValue],
    ) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = self.DIAG_NAME
        status.hardware_id = "rebotarm_monitor:driver"
        status.level = level
        status.message = message
        status.values = values
        return status
