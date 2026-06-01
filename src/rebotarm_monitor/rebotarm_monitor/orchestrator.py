"""Application service: registers trackers and turns them into diagnostics."""

from __future__ import annotations

import time
from typing import Iterable, Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from .domain.tracker import HealthTracker, SubscriptionRegistrar, TrackerContext
from .trackers.arm_status import ArmStatusTracker


class MonitorOrchestrator:
    """Owns the ordered list of trackers and the per-cycle build/reset flow."""

    def __init__(self, trackers: Optional[Iterable[HealthTracker]] = None) -> None:
        self._trackers: list[HealthTracker] = list(trackers or [])

    @property
    def trackers(self) -> list[HealthTracker]:
        return list(self._trackers)

    def add(self, tracker: HealthTracker) -> None:
        self._trackers.append(tracker)

    def register_subscriptions(self, registrar: SubscriptionRegistrar) -> None:
        for tracker in self._trackers:
            tracker.register_subscriptions(registrar)

    def _build_context(self) -> TrackerContext:
        for tracker in self._trackers:
            if isinstance(tracker, ArmStatusTracker):
                msg = tracker.last_msg
                arm_enabled = tracker.arm_enabled
                if msg is None:
                    return TrackerContext(arm_enabled=arm_enabled)
                if msg.state_machine == "GRAVITY_COMP":
                    return TrackerContext(
                        arm_enabled=arm_enabled,
                        gravity_compensation_active=True,
                        position_hold_active=False,
                        control_context="gravity_compensation",
                    )
                if (
                    msg.mode == "pos_vel"
                    and bool(msg.enabled)
                    and bool(msg.control_loop_active)
                ):
                    return TrackerContext(
                        arm_enabled=arm_enabled,
                        gravity_compensation_active=False,
                        position_hold_active=True,
                        control_context="position_hold",
                    )
                return TrackerContext(
                    arm_enabled=arm_enabled,
                    gravity_compensation_active=False,
                    position_hold_active=False,
                    control_context="normal_or_unknown",
                )
        return TrackerContext()

    def build_statuses(self, now: Optional[float] = None) -> list[DiagnosticStatus]:
        if now is None:
            now = time.monotonic()
        ctx = self._build_context()
        return [tracker.build_status(now, ctx) for tracker in self._trackers]

    def build_diagnostic_array(
        self,
        header_stamp,
        now: Optional[float] = None,
    ) -> DiagnosticArray:
        array = DiagnosticArray()
        array.header.stamp = header_stamp
        array.status = self.build_statuses(now)
        return array

    def reset_periods(self) -> None:
        for tracker in self._trackers:
            tracker.reset_period()

    def find_by_diag_name(self, name: str) -> Optional[HealthTracker]:
        for tracker in self._trackers:
            if tracker.diag_name == name:
                return tracker
        return None
