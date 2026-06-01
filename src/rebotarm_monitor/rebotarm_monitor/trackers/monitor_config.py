"""Static monitor configuration surfaced in ``rqt_robot_monitor``."""

from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from ..domain.tracker import HealthTracker, TrackerContext
from ..parameters import profile_assumed_payload_kg
from ..support.diagnostics import kv


class MonitorConfigTracker(HealthTracker):
    """Reports the active payload profile (monitor torque threshold set).

    Lives under the ``System`` aggregator group alongside the driver process
    check. No subscriptions: values are fixed for the node lifetime.
    """

    DIAG_NAME = "rebotarm/system/monitor_config"

    def __init__(self, params: dict) -> None:
        self._payload_profile = str(params["payload_profile"])
        self._assumed_payload_kg = float(params["assumed_payload_kg"])

    @property
    def diag_name(self) -> str:
        return self.DIAG_NAME

    def build_status(self, now: float, context: TrackerContext) -> DiagnosticStatus:
        del now, context
        profile = self._payload_profile
        kg = self._assumed_payload_kg
        message = f"payload profile: {profile} ({kg:g} kg assumed payload)"
        values: list[KeyValue] = [
            kv("payload_profile", profile),
            kv("assumed_payload_kg", f"{kg:g}"),
        ]

        status = DiagnosticStatus()
        status.name = self.DIAG_NAME
        status.hardware_id = "rebotarm_monitor"
        status.level = DiagnosticStatus.OK
        status.message = message
        status.values = values
        return status
