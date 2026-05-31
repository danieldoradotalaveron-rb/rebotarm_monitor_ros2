"""Serial/USB character device presence (host-side link check)."""

from __future__ import annotations

from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from ..adapters.device_path import DevicePathInspector, RealDevicePathInspector
from ..domain.tracker import HealthTracker, TrackerContext
from ..support.diagnostics import kv, max_level


class SerialLinkTracker(HealthTracker):
    """Checks that the driver serial device node exists and is usable.

    This complements topic-based stale checks: it fails fast when the TTY
    disappears (unplugged cable, missing udev symlink) before joint feedback
    stops. It does **not** read the driver's ``channel`` parameter; set
    ``serial_device`` to the same path you pass to the driver launch
    (default matches Seeed ``arm.yaml``: ``/dev/ttyACM0``).
    """

    DIAG_NAME = "rebotarm/link/serial"

    def __init__(
        self,
        params: dict,
        inspector: Optional[DevicePathInspector] = None,
    ) -> None:
        self.params = params
        self._inspector: DevicePathInspector = inspector or RealDevicePathInspector()
        self.last_warning_reason = ""

    @property
    def diag_name(self) -> str:
        return self.DIAG_NAME

    def build_status(self, now: float, context: TrackerContext) -> DiagnosticStatus:
        device = str(self.params.get("device_path", "")).strip()
        level = DiagnosticStatus.OK
        message = f"{device} present"
        reason = ""
        values: list[KeyValue] = [kv("device_path", device or "unset")]

        if not device:
            level = DiagnosticStatus.WARN
            message = "serial_device not configured"
            reason = "unset_device"
            self.last_warning_reason = reason
            values.append(kv("last_warning_reason", reason))
            return self._build(level, message, values)

        resolved = self._inspector.resolve(device)
        values.append(kv("resolved_path", resolved or "n/a"))

        if not self._inspector.exists(device):
            level = DiagnosticStatus.ERROR
            message = f"{device} not present"
            reason = "missing_device"
            self.last_warning_reason = reason
            values.append(kv("last_warning_reason", reason))
            return self._build(level, message, values)

        if not self._inspector.is_char_device(device):
            level = DiagnosticStatus.WARN
            message = f"{device} is not a character device"
            reason = "not_char_device"

        if not self._inspector.is_readable_writable(device):
            level = max_level(level, DiagnosticStatus.WARN)
            message = f"{device} not readable/writable by this user"
            reason = "permission_denied"

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
        status.hardware_id = "rebotarm_monitor:serial"
        status.level = level
        status.message = message
        status.values = values
        return status
