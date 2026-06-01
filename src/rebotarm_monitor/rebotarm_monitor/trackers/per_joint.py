"""Per-joint motor health (one instance per joint)."""

from __future__ import annotations

import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.qos import qos_profile_sensor_data
from rebotarm_msgs.msg import JointMotorState

from ..domain.per_joint_limits import PerJointLimitsView
from ..domain.tracker import HealthTracker, SubscriptionRegistrar, TrackerContext
from ..support.diagnostics import is_finite, kv, max_level
from ..support.measurement_format import format_abs_vs_limit


class PerJointTracker(HealthTracker):
    """One tracker instance per joint, listening to its per-joint state topic."""

    def __init__(self, joint_name: str, topic: str, params: dict) -> None:
        self.joint_name = joint_name
        self.topic = topic
        self.params = params
        self._limits = PerJointLimitsView.from_tracker_params(params)
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
            if jump > self._limits.max_position_jump_rad:
                self.period_position_jump = jump
        if is_finite(pos):
            self.prev_position = pos

        if self.prev_torque is not None and is_finite(torque):
            t_jump = abs(torque - self.prev_torque)
            if t_jump > self._limits.max_torque_jump_nm:
                self.period_torque_jump = t_jump
        if is_finite(torque):
            self.prev_torque = torque

        if is_finite(vel) and abs(vel) > self._limits.max_abs_velocity_rad_s:
            self.period_high_vel = True

        if is_finite(torque) and abs(torque) > self._limits.max_abs_torque_nm:
            self.period_high_torque = True

        if (
            is_finite(vel)
            and is_finite(torque)
            and abs(vel) < self._limits.idle_velocity_threshold_rad_s
            and abs(torque) > self._limits.idle_torque_warn_nm
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

    @staticmethod
    def _stationary_effort_suppressed(context: TrackerContext) -> bool:
        return context.gravity_compensation_active or context.position_hold_active

    def _torque_vs_max(self, torque: float) -> str:
        return format_abs_vs_limit(
            torque, self._limits.max_abs_torque_nm, "Nm"
        )

    def _velocity_vs_max(self, velocity: float) -> str:
        return format_abs_vs_limit(
            velocity, self._limits.max_abs_velocity_rad_s, "rad/s"
        )

    def _torque_vs_idle(self, torque: float) -> str:
        return format_abs_vs_limit(
            torque, self._limits.idle_torque_warn_nm, "Nm"
        )

    def _measurement_suffix(self, msg: JointMotorState) -> str:
        """Live |T| and |v| vs resolved limits (shown in rqt message when healthy)."""
        torque = float(msg.torque)
        velocity = float(msg.velocity)
        return (
            f"|T|={self._torque_vs_max(torque)} "
            f"|v|={self._velocity_vs_max(velocity)}"
        )

    def _healthy_message(self, msg: JointMotorState, *, elevated: bool = False) -> str:
        torque = float(msg.torque)
        velocity = float(msg.velocity)
        if elevated:
            return (
                f"{self.joint_name} "
                f"|T|={self._torque_vs_idle(torque)} "
                f"|v|={self._velocity_vs_max(velocity)} "
                f"(elevated stationary)"
            )
        return f"{self.joint_name} {self._measurement_suffix(msg)}"

    def build_status(self, now: float, context: TrackerContext) -> DiagnosticStatus:
        arm_enabled = context.arm_enabled
        age = None if self.last_recv_mono is None else now - self.last_recv_mono
        level = DiagnosticStatus.OK
        message = f"{self.joint_name} healthy"
        reason = ""
        msg = self.last_msg

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
                jump = self.period_position_jump
                message = (
                    f"position jump {jump:.2f}/"
                    f"{self._limits.max_position_jump_rad:.2f} rad"
                )
                reason = "position_jump"
            elif self.period_torque_jump is not None:
                level = DiagnosticStatus.WARN
                jump = self.period_torque_jump
                message = (
                    f"torque jump {jump:.2f}/"
                    f"{self._limits.max_torque_jump_nm:.2f} Nm"
                )
                reason = "torque_jump"
            elif self.period_high_vel and msg is not None:
                level = DiagnosticStatus.WARN
                message = (
                    f"high velocity |v|={self._velocity_vs_max(float(msg.velocity))}"
                )
                reason = "high_velocity"
            elif self.period_high_torque and msg is not None:
                level = DiagnosticStatus.WARN
                message = (
                    f"high torque |T|={self._torque_vs_max(float(msg.torque))}"
                )
                reason = "high_torque"
            elif self.period_idle_torque:
                if not self._stationary_effort_suppressed(context):
                    level = DiagnosticStatus.WARN
                    if msg is not None:
                        message = (
                            "high torque while idle "
                            f"|T|={self._torque_vs_idle(float(msg.torque))}"
                        )
                    else:
                        message = "high torque while idle"
                    reason = "idle_torque"
                elif msg is not None:
                    message = self._healthy_message(msg, elevated=True)

        stationary_suppressed = (
            self.period_idle_torque and self._stationary_effort_suppressed(context)
        )
        if level == DiagnosticStatus.OK and msg is not None and not stationary_suppressed:
            message = self._healthy_message(msg)
        if reason:
            self.last_warning_reason = reason
        elif stationary_suppressed and self.last_warning_reason == "idle_torque":
            self.last_warning_reason = ""

        load_state = "elevated" if self.period_idle_torque else "nominal"
        values: list[KeyValue] = [
            kv("topic", self.topic),
            kv("last_message_age_s", "n/a" if age is None else f"{age:.3f}"),
            kv("max_abs_joint_velocity_rad_s", self._limits.max_abs_velocity_rad_s),
            kv("max_abs_joint_torque_nm", self._limits.max_abs_torque_nm),
            kv("idle_velocity_threshold_rad_s", self._limits.idle_velocity_threshold_rad_s),
            kv("idle_torque_warn_nm", self._limits.idle_torque_warn_nm),
            kv("control_context", context.control_context),
            kv("load_state", load_state),
            kv(
                "control_gravity_compensation_active",
                context.gravity_compensation_active,
            ),
            kv("position_hold_active", context.position_hold_active),
            kv("stationary_effort_check_suppressed", stationary_suppressed),
            kv("last_warning_reason", self.last_warning_reason or "none"),
        ]
        if msg is not None:
            torque = float(msg.torque)
            velocity = float(msg.velocity)
            values.extend(
                [
                    kv("position", f"{msg.position:.4f}"),
                    kv("velocity", f"{msg.velocity:.4f}"),
                    kv("torque", f"{torque:.4f}"),
                    kv("torque_vs_max_nm", self._torque_vs_max(torque)),
                    kv("torque_vs_idle_nm", self._torque_vs_idle(torque)),
                    kv("velocity_vs_max_rad_s", self._velocity_vs_max(velocity)),
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
