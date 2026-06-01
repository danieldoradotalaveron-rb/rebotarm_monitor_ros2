"""Per-joint motor health (one instance per joint)."""

from __future__ import annotations

import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.qos import qos_profile_sensor_data
from rebotarm_msgs.msg import JointMotorState

from ..domain.tracker import HealthTracker, SubscriptionRegistrar, TrackerContext
from ..support.diagnostics import is_finite, kv, max_level


class PerJointTracker(HealthTracker):
    """One tracker instance per joint, listening to its per-joint state topic."""

    def __init__(self, joint_name: str, topic: str, params: dict) -> None:
        self.joint_name = joint_name
        self.topic = topic
        self.params = params
        self.last_recv_mono: Optional[float] = None
        self.last_msg: Optional[JointMotorState] = None
        self.prev_position: Optional[float] = None
        self.prev_torque: Optional[float] = None
        self.period_position_jump: Optional[float] = None
        self.period_torque_jump: Optional[float] = None
        self.period_high_vel = False
        self.period_high_torque = False
        self.period_idle_torque = False
        self.period_non_finite = False
        self.last_warning_reason = ""

    @property
    def diag_name(self) -> str:
        return f"rebotarm/joints/{self.joint_name}"

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
        pos = float(msg.position)
        vel = float(msg.velocity)
        torque = float(msg.torque)

        if not (is_finite(pos) and is_finite(vel) and is_finite(torque)):
            self.period_non_finite = True

        if self.prev_position is not None and is_finite(pos):
            jump = abs(pos - self.prev_position)
            if jump > self.params["max_joint_position_jump_rad"]:
                self.period_position_jump = jump
        if is_finite(pos):
            self.prev_position = pos

        if self.prev_torque is not None and is_finite(torque):
            t_jump = abs(torque - self.prev_torque)
            if t_jump > self.params["max_joint_torque_jump_nm"]:
                self.period_torque_jump = t_jump
        if is_finite(torque):
            self.prev_torque = torque

        if is_finite(vel) and abs(vel) > self.params["max_abs_joint_velocity_rad_s"]:
            self.period_high_vel = True

        if is_finite(torque) and abs(torque) > self.params["max_abs_joint_torque_nm"]:
            self.period_high_torque = True

        if (
            is_finite(vel)
            and is_finite(torque)
            and abs(vel) < self.params["idle_velocity_threshold_rad_s"]
            and abs(torque) > self.params["idle_torque_warn_nm"]
        ):
            self.period_idle_torque = True

    def reset_period(self) -> None:
        self.period_position_jump = None
        self.period_torque_jump = None
        self.period_high_vel = False
        self.period_high_torque = False
        self.period_idle_torque = False
        self.period_non_finite = False

    def _status_code_level(
        self,
        code: int,
        arm_enabled: Optional[bool],
    ) -> tuple[bytes, str]:
        expected = self.params["expected_enabled_status_code"]
        disabled = self.params["disabled_status_code"]
        if code == expected:
            return DiagnosticStatus.OK, "motor enabled"
        if code == disabled and self.params["allow_disabled_status_code"]:
            if arm_enabled is True:
                return DiagnosticStatus.WARN, "motor disabled while arm enabled"
            return DiagnosticStatus.OK, "motor disabled"
        return DiagnosticStatus.WARN, f"unexpected status_code={code}"

    def build_status(self, now: float, context: TrackerContext) -> DiagnosticStatus:
        arm_enabled = context.arm_enabled
        age = None if self.last_recv_mono is None else now - self.last_recv_mono
        level = DiagnosticStatus.OK
        message = f"{self.joint_name} healthy"
        reason = ""

        if self.last_recv_mono is None:
            level = DiagnosticStatus.ERROR
            message = "no joint state received"
            reason = "no_messages"
        elif age is not None and age > self.params["per_joint_stale_timeout_s"]:
            level = DiagnosticStatus.ERROR
            message = f"stale age={age:.2f}s"
            reason = "stale"
        elif self.period_non_finite:
            level = DiagnosticStatus.ERROR
            message = "non-finite values"
            reason = "non_finite"
        elif self.last_msg is not None:
            code_level, code_msg = self._status_code_level(
                int(self.last_msg.status_code), arm_enabled
            )
            level = max_level(level, code_level)
            if code_level != DiagnosticStatus.OK:
                message = code_msg
                reason = "status_code"

        if level == DiagnosticStatus.OK:
            if self.period_position_jump is not None:
                level = DiagnosticStatus.WARN
                message = f"position jump {self.period_position_jump:.2f} rad"
                reason = "position_jump"
            elif self.period_torque_jump is not None:
                level = DiagnosticStatus.WARN
                message = f"torque jump {self.period_torque_jump:.2f} Nm"
                reason = "torque_jump"
            elif self.period_high_vel:
                level = DiagnosticStatus.WARN
                message = "high velocity"
                reason = "high_velocity"
            elif self.period_high_torque:
                level = DiagnosticStatus.WARN
                message = "high torque"
                reason = "high_torque"
            elif self.period_idle_torque and not context.gravity_compensation_active:
                level = DiagnosticStatus.WARN
                message = "high torque while idle"
                reason = "idle_torque"

        if reason:
            self.last_warning_reason = reason

        msg = self.last_msg
        values: list[KeyValue] = [
            kv("topic", self.topic),
            kv("last_message_age_s", "n/a" if age is None else f"{age:.3f}"),
            kv("max_abs_joint_velocity_rad_s", self.params["max_abs_joint_velocity_rad_s"]),
            kv("max_abs_joint_torque_nm", self.params["max_abs_joint_torque_nm"]),
            kv("idle_velocity_threshold_rad_s", self.params["idle_velocity_threshold_rad_s"]),
            kv("idle_torque_warn_nm", self.params["idle_torque_warn_nm"]),
            kv(
                "control_gravity_compensation_active",
                context.gravity_compensation_active,
            ),
            kv(
                "idle_torque_check_suppressed",
                self.period_idle_torque and context.gravity_compensation_active,
            ),
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
        status.name = self.diag_name
        status.hardware_id = "rebotarm_monitor"
        status.level = level
        status.message = message
        status.values = values
        return status
