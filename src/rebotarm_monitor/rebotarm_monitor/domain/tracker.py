"""Domain contract every health tracker must satisfy.

A tracker is a strategy that observes one health concern (a topic, a bus, a
process...) and produces one ``diagnostic_msgs/DiagnosticStatus`` per
publication cycle. Trackers are owned by ``MonitorOrchestrator`` and are kept
intentionally free of ROS plumbing: anything ROS-specific is injected via the
``register_subscriptions`` hook.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Protocol

from diagnostic_msgs.msg import DiagnosticStatus


@dataclass
class TrackerContext:
    """Cross-tracker read-only context shared per publication cycle.

    Trackers may read from this struct but must not mutate it. The orchestrator
    populates it before calling ``build_status`` on each tracker.
    """

    arm_enabled: Optional[bool] = None


class SubscriptionRegistrar(Protocol):
    """Minimal port a tracker uses to register ROS subscriptions.

    Implemented by the ROS node; mocked in tests when needed.
    """

    def create_subscription(self, msg_type, topic, callback, qos):  # noqa: ANN001
        ...


class HealthTracker(ABC):
    """Base class for every passive health check.

    Sub-classes must implement :py:meth:`build_status`. The other hooks have
    default no-op implementations so simple trackers stay short.
    """

    @property
    @abstractmethod
    def diag_name(self) -> str:
        """The ``DiagnosticStatus.name`` this tracker publishes."""

    def register_subscriptions(self, registrar: SubscriptionRegistrar) -> None:
        """Optional: register ROS topic subscriptions on the host node."""

    @abstractmethod
    def build_status(
        self,
        now: float,
        context: TrackerContext,
    ) -> DiagnosticStatus:
        """Produce one DiagnosticStatus snapshot at monotonic time ``now``."""

    def reset_period(self) -> None:
        """Optional: clear per-cycle accumulators after diagnostics publish."""

    def summary_line(self, now: float) -> Optional[str]:
        """Optional: human-readable console line; ``None`` to stay silent."""
        return None
