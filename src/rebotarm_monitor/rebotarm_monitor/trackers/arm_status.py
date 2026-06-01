"""Latched ``rebotarm_msgs/ArmStatus`` tracker (mode-agnostic reporter)."""

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


class ArmStatusTracker(HealthTracker):
    """Subscribes to the latched ``ArmStatus`` topic and republishes its state.

    Branchless reporter: it does not interpret ``state_machine`` and does not
    suppress sibling warnings based on the arm's control mode.
    """

    DIAG_NAME = "rebotarm/control/arm_status"

    def __init__(self, topic: str, params: dict) -> None:
        self.topic = topic
        self.params = params
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

    @property
    def arm_enabled(self) -> Optional[bool]:
        if self.last_msg is None:
            return None
        return bool(self.last_msg.enabled)

    def build_status(self, now: float, context: TrackerContext) -> DiagnosticStatus:
        age = None if self.last_recv_mono is None else now - self.last_recv_mono
        level = DiagnosticStatus.OK
        message = "arm_status healthy"
        reason = ""
        msg = self.last_msg

        if self.last_recv_mono is None:
            level = DiagnosticStatus.ERROR
            message = "no arm_status received"
            reason = "no_messages"
        elif (
            self.params["arm_status_warn_on_snapshot_age"]
            and age is not None
            and age > self.params["arm_status_stale_timeout_s"]
        ):
            level = DiagnosticStatus.WARN
            message = f"arm_status snapshot age={age:.2f}s"
            reason = "stale_snapshot"
        elif msg is not None:
            message = (
                f"active mode: {msg.mode}, state: {msg.state_machine}"
            )

        if msg is not None and msg.error_codes:
            level = DiagnosticStatus.ERROR
            message = f"error_codes present: {list(msg.error_codes)}"
            reason = "error_codes"

        if level in (DiagnosticStatus.OK, DiagnosticStatus.WARN) and msg is not None:
            if self.params["expect_arm_enabled"] and not msg.enabled:
                level = DiagnosticStatus.WARN
                message = "arm disabled"
                reason = "disabled"
            elif (
                self.params["expect_control_loop_active"]
                and not msg.control_loop_active
            ):
                level = DiagnosticStatus.WARN
                message = "control loop inactive"
                reason = "control_loop_inactive"

        if reason:
            self.last_warning_reason = reason

        values: list[KeyValue] = [
            kv("topic", self.topic),
            kv("last_message_age_s", "n/a" if age is None else f"{age:.3f}"),
            kv("snapshot_event_driven", True),
            kv("last_warning_reason", self.last_warning_reason or "none"),
        ]
        if msg is not None:
            values.extend(
                [
                    kv("enabled", msg.enabled),
                    kv("mode", msg.mode),
                    kv("control_loop_active", msg.control_loop_active),
                    kv("state_machine", msg.state_machine),
                    kv("error_codes", list(msg.error_codes) or "none"),
                    kv(
                        "joint_status_codes",
                        list(msg.per_joint_status_code)
                        if msg.per_joint_status_code
                        else "none",
                    ),
                ]
            )

        status = DiagnosticStatus()
        status.name = self.DIAG_NAME
        status.hardware_id = "rebotarm_monitor"
        status.level = level
        status.message = message
        status.values = values
        return status
