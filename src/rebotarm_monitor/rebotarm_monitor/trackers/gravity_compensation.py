"""Gravity compensation state from latched ``rebotarm_msgs/ArmStatus``."""

from __future__ import annotations

import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus

from ..domain.tracker import HealthTracker, SubscriptionRegistrar, TrackerContext
from ..support.diagnostics import kv


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def _gravity_compensation_active(msg: ArmStatus) -> bool:
    return msg.state_machine == "GRAVITY_COMP"


class GravityCompensationTracker(HealthTracker):
    """Reports whether the driver state machine is in ``GRAVITY_COMP``."""

    DIAG_NAME = "rebotarm/control/gravity_compensation"

    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.last_recv_mono: Optional[float] = None
        self.last_msg: Optional[ArmStatus] = None
        self.last_warning_reason = ""

    @property
    def diag_name(self) -> str:
        return self.DIAG_NAME

    def register_subscriptions(self, registrar: SubscriptionRegistrar) -> None:
        registrar.create_subscription(
            ArmStatus,
            self.topic,
            self.on_message,
            _latched_qos(),
        )

    def on_message(self, msg: ArmStatus) -> None:
        self.last_recv_mono = time.monotonic()
        self.last_msg = msg

    def build_status(self, now: float, context: TrackerContext) -> DiagnosticStatus:
        del context  # unused; arm status context not required for GC reporting
        age = None if self.last_recv_mono is None else now - self.last_recv_mono
        msg = self.last_msg

        if self.last_recv_mono is None:
            level = DiagnosticStatus.ERROR
            message = "no arm_status received"
            self.last_warning_reason = "no_messages"
            values: list[KeyValue] = [
                kv("topic", self.topic),
                kv("last_warning_reason", self.last_warning_reason),
            ]
        else:
            level = DiagnosticStatus.OK
            self.last_warning_reason = "none"
            active = msg is not None and _gravity_compensation_active(msg)
            if active:
                message = "gravity compensation: active"
            else:
                message = "gravity compensation: inactive"
            values = [
                kv("topic", self.topic),
                kv("last_message_age_s", f"{age:.3f}"),
                kv("gravity_compensation_active", active),
                kv("state_machine", msg.state_machine if msg is not None else "n/a"),
                kv("mode", msg.mode if msg is not None else "n/a"),
                kv("enabled", msg.enabled if msg is not None else "n/a"),
                kv(
                    "control_loop_active",
                    msg.control_loop_active if msg is not None else "n/a",
                ),
                kv("last_warning_reason", self.last_warning_reason),
            ]

        status = DiagnosticStatus()
        status.name = self.DIAG_NAME
        status.hardware_id = "rebotarm_monitor"
        status.level = level
        status.message = message
        status.values = values
        return status
