"""SocketCAN interface health (kernel-counter polling, no ROS subscription)."""

from __future__ import annotations

from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from ..adapters.sysfs import RealSysFsReader, SysFsReader
from ..domain.tracker import HealthTracker, TrackerContext
from ..support.diagnostics import kv, max_level


class CanBusTracker(HealthTracker):
    """Reports SocketCAN interface counters and operstate."""

    _STAT_FIELDS = (
        "rx_packets",
        "tx_packets",
        "rx_bytes",
        "tx_bytes",
        "rx_errors",
        "tx_errors",
        "rx_dropped",
        "tx_dropped",
    )

    def __init__(
        self,
        iface: str,
        params: dict,
        sysfs: Optional[SysFsReader] = None,
    ) -> None:
        self.iface = iface
        self.params = params
        self._sysfs: SysFsReader = sysfs or RealSysFsReader()
        self._base = f"/sys/class/net/{iface}"
        self._stats = f"{self._base}/statistics"
        self._prev_counters: Optional[dict[str, int]] = None
        self._prev_mono: Optional[float] = None
        self.last_warning_reason = ""

    @property
    def diag_name(self) -> str:
        return f"rebotarm/bus/{self.iface}"

    def _read_counters(self) -> Optional[dict[str, int]]:
        if not self._sysfs.is_dir(self._stats):
            return None
        out: dict[str, int] = {}
        for name in self._STAT_FIELDS:
            value = self._sysfs.read_int(f"{self._stats}/{name}")
            out[name] = value if value is not None else 0
        return out

    def build_status(self, now: float, context: TrackerContext) -> DiagnosticStatus:
        level = DiagnosticStatus.OK
        message = f"{self.iface} healthy"
        reason = ""
        values: list[KeyValue] = [kv("interface", self.iface)]

        if not self._sysfs.is_dir(self._base):
            level = DiagnosticStatus.ERROR
            message = f"{self.iface} not present"
            reason = "missing_interface"
            self.last_warning_reason = reason
            values.append(kv("last_warning_reason", reason))
            return self._build(level, message, values)

        operstate = self._sysfs.read_text(f"{self._base}/operstate") or "unknown"
        carrier = self._sysfs.read_int(f"{self._base}/carrier")
        values.append(kv("operstate", operstate))
        values.append(kv("carrier", "n/a" if carrier is None else carrier))

        counters = self._read_counters()
        if counters is None:
            level = DiagnosticStatus.ERROR
            message = f"{self.iface} statistics unavailable"
            reason = "no_statistics"
            self.last_warning_reason = reason
            values.append(kv("last_warning_reason", reason))
            return self._build(level, message, values)

        delta = {k: 0 for k in self._STAT_FIELDS}
        period = 0.0
        if self._prev_counters is not None and self._prev_mono is not None:
            for k in self._STAT_FIELDS:
                d = counters[k] - self._prev_counters.get(k, counters[k])
                delta[k] = d if d >= 0 else 0
            period = max(now - self._prev_mono, 0.0)

        for k in self._STAT_FIELDS:
            values.append(kv(k, counters[k]))
            values.append(kv(f"{k}_delta", delta[k]))
        values.append(kv("period_s", f"{period:.2f}"))

        warn_iface_down = bool(self.params.get("warn_on_iface_down", True))
        err_per_period = int(self.params.get("error_warn_per_period", 1))
        drop_per_period = int(self.params.get("dropped_warn_per_period", 10))

        if warn_iface_down and operstate.lower() not in ("up", "unknown"):
            level = max_level(level, DiagnosticStatus.WARN)
            message = f"{self.iface} operstate={operstate}"
            reason = "iface_down"

        bus_errors = delta["rx_errors"] + delta["tx_errors"]
        bus_drops = delta["rx_dropped"] + delta["tx_dropped"]

        if bus_errors >= err_per_period:
            level = DiagnosticStatus.ERROR
            message = f"{self.iface} {bus_errors} bus error(s) in last {period:.1f}s"
            reason = "bus_errors"
        elif bus_drops >= drop_per_period:
            level = max_level(level, DiagnosticStatus.WARN)
            message = (
                f"{self.iface} {bus_drops} dropped frame(s) in last {period:.1f}s"
            )
            reason = "dropped_frames"

        if reason:
            self.last_warning_reason = reason
        values.append(kv("last_warning_reason", self.last_warning_reason or "none"))

        self._prev_counters = counters
        self._prev_mono = now
        return self._build(level, message, values)

    def _build(
        self,
        level: bytes,
        message: str,
        values: list[KeyValue],
    ) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = self.diag_name
        status.hardware_id = f"rebotarm_monitor:{self.iface}"
        status.level = level
        status.message = message
        status.values = values
        return status
