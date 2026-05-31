"""Gripper health tracker."""

from __future__ import annotations

import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.qos import qos_profile_sensor_data
from rebotarm_msgs.msg import JointMotorState

from ..domain.tracker import HealthTracker, SubscriptionRegistrar, TrackerContext
from ..support.diagnostics import is_finite, kv


class GripperTracker(HealthTracker):
    """Monitors the gripper state topic."""

    DIAG_NAME = "rebotarm/gripper/state"

    def __init__(self, topic: str, params: dict) -> None:
        self.topic = topic
        self.params = params
        self.last_recv_mono: Optional[float] = None
        self.last_msg: Optional[JointMotorState] = None
        self.period_non_finite = False
        self.period_high_vel = False
        self.period_high_torque = False
        self.last_warning_reason = ""

    @property
    def diag_name(self) -> str:
        return self.DIAG_NAME

    def register_subscriptions(self, registrar: SubscriptionRegistrar) -> None:
        registrar.create_subscription(
            JointMotorState,
            self.topic,
            self.on_message,
            qos_profile_sensor_data,
        )

    def on_message(self, msg: JointMotorState) -> None:
        self.last_recv_mono = time.monotonic()
        self.last_msg = msg
        if not (
            is_finite(msg.position)
            and is_finite(msg.velocity)
            and is_finite(msg.torque)
        ):
            self.period_non_finite = True
        if abs(msg.velocity) > self.params["max_abs_gripper_velocity"]:
            self.period_high_vel = True
        if abs(msg.torque) > self.params["max_abs_gripper_torque"]:
            self.period_high_torque = True

    def reset_period(self) -> None:
        self.period_non_finite = False
        self.period_high_vel = False
        self.period_high_torque = False

    def build_status(self, now: float, context: TrackerContext) -> DiagnosticStatus:
        age = None if self.last_recv_mono is None else now - self.last_recv_mono
        level = DiagnosticStatus.OK
        message = "gripper healthy"
        reason = ""

        if self.last_recv_mono is None:
            level = DiagnosticStatus.WARN
            message = "no gripper state received"
            reason = "no_messages"
        elif age is not None and age > self.params["gripper_stale_timeout_s"]:
            level = DiagnosticStatus.WARN
            message = f"stale age={age:.2f}s"
            reason = "stale"
        elif self.period_non_finite:
            level = DiagnosticStatus.ERROR
            message = "non-finite gripper values"
            reason = "non_finite"
        elif self.period_high_vel:
            level = DiagnosticStatus.WARN
            message = "high gripper velocity"
            reason = "high_velocity"
        elif self.period_high_torque:
            level = DiagnosticStatus.WARN
            message = "high gripper torque"
            reason = "high_torque"

        if reason:
            self.last_warning_reason = reason

        msg = self.last_msg
        values: list[KeyValue] = [
            kv("topic", self.topic),
            kv("last_message_age_s", "n/a" if age is None else f"{age:.3f}"),
            kv("last_warning_reason", self.last_warning_reason or "none"),
        ]
        if msg is not None:
            values.extend(
                [
                    kv("position", f"{msg.position:.4f}"),
                    kv("velocity", f"{msg.velocity:.4f}"),
                    kv("torque", f"{msg.torque:.4f}"),
                    kv("status_code", int(msg.status_code)),
                ]
            )

        status = DiagnosticStatus()
        status.name = self.DIAG_NAME
        status.hardware_id = "rebotarm_monitor"
        status.level = level
        status.message = message
        status.values = values
        return status
