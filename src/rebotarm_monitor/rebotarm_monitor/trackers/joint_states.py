"""Aggregate health check for ``sensor_msgs/JointState`` (whole-arm view)."""

from __future__ import annotations

import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from ..domain.tracker import HealthTracker, SubscriptionRegistrar, TrackerContext
from ..parameters import resolve_joint_threshold
from ..support.diagnostics import is_finite, kv
from ..support.rate_window import RateWindow


class JointStatesTracker(HealthTracker):
    """Monitors the aggregated ``/rebotarm/joint_states`` topic."""

    DIAG_NAME = "rebotarm/hardware/joint_states"

    def __init__(self, topic: str, params: dict) -> None:
        self.topic = topic
        self.params = params
        self.rate = RateWindow()
        self.last_recv_mono: Optional[float] = None
        self.joint_count = 0
        self.has_pos = False
        self.has_vel = False
        self.has_effort = False
        self.prev_positions: dict[str, float] = {}
        self.period_non_finite_pos = False
        self.period_non_finite_vel = False
        self.period_non_finite_effort = False
        self.period_missing_vel = False
        self.period_missing_effort = False
        self.period_position_jumps: dict[str, float] = {}
        self.period_high_vel: dict[str, float] = {}
        self.period_high_effort: dict[str, float] = {}
        self.max_abs_vel_observed = 0.0
        self.max_abs_effort_observed = 0.0
        self.last_warning_reason = ""

    @property
    def diag_name(self) -> str:
        return self.DIAG_NAME

    def register_subscriptions(self, registrar: SubscriptionRegistrar) -> None:
        registrar.create_subscription(
            JointState,
            self.topic,
            self.on_message,
            qos_profile_sensor_data,
        )

    def on_message(self, msg: JointState) -> None:
        self.last_recv_mono = time.monotonic()
        self.rate.msg_count += 1
        names = list(msg.name)
        self.joint_count = len(names)
        self.has_pos = len(msg.position) == len(names) and len(names) > 0
        self.has_vel = len(msg.velocity) == len(names) and len(names) > 0
        self.has_effort = len(msg.effort) == len(names) and len(names) > 0

        if not self.has_vel:
            self.period_missing_vel = True
        if not self.has_effort:
            self.period_missing_effort = True

        max_vel_overrides = self.params.get("per_joint_max_abs_velocity_rad_s", {})
        max_effort_overrides = self.params.get("per_joint_max_abs_torque_nm", {})

        for i, name in enumerate(names):
            max_vel = resolve_joint_threshold(
                name,
                self.params["max_abs_velocity_rad_s"],
                max_vel_overrides,
            )
            max_effort = resolve_joint_threshold(
                name,
                self.params["max_abs_effort_nm"],
                max_effort_overrides,
            )

            if self.has_pos:
                pos = float(msg.position[i])
                if not is_finite(pos):
                    self.period_non_finite_pos = True
                else:
                    prev = self.prev_positions.get(name)
                    if prev is not None:
                        jump = abs(pos - prev)
                        if jump > self.params["max_position_jump_rad"]:
                            self.period_position_jumps[name] = jump
                    self.prev_positions[name] = pos

            if self.has_vel:
                vel = float(msg.velocity[i])
                self.max_abs_vel_observed = max(self.max_abs_vel_observed, abs(vel))
                if not is_finite(vel):
                    self.period_non_finite_vel = True
                elif abs(vel) > max_vel:
                    self.period_high_vel[name] = vel

            if self.has_effort:
                effort = float(msg.effort[i])
                self.max_abs_effort_observed = max(
                    self.max_abs_effort_observed, abs(effort)
                )
                if not is_finite(effort):
                    self.period_non_finite_effort = True
                elif abs(effort) > max_effort:
                    self.period_high_effort[name] = effort

    def reset_period(self) -> None:
        self.period_non_finite_pos = False
        self.period_non_finite_vel = False
        self.period_non_finite_effort = False
        self.period_missing_vel = False
        self.period_missing_effort = False
        self.period_position_jumps.clear()
        self.period_high_vel.clear()
        self.period_high_effort.clear()
        self.max_abs_vel_observed = 0.0
        self.max_abs_effort_observed = 0.0
        self.rate.reset()

    @staticmethod
    def _abnormal_joints_message(joints: dict[str, float]) -> tuple[str, str]:
        """Generic summary when one or more joints tripped a per-joint check."""
        count = len(joints)
        message = f"some joints in abnormal state ({count} joints)"
        return message, "abnormal_state"

    def build_status(self, now: float, context: TrackerContext) -> DiagnosticStatus:
        rate = self.rate.measured_hz()
        min_rate = self.params["expected_rate_hz"] * self.params["min_rate_ratio"]
        age = None if self.last_recv_mono is None else now - self.last_recv_mono

        level = DiagnosticStatus.OK
        message = "joint_states healthy"
        reason = ""

        if self.last_recv_mono is None:
            level = DiagnosticStatus.ERROR
            message = "no joint_states received"
            reason = "no_messages"
        elif age is not None and age > self.params["stale_timeout_s"]:
            level = DiagnosticStatus.ERROR
            message = f"stale data age={age:.2f}s"
            reason = "stale"
        elif self.joint_count == 0:
            level = DiagnosticStatus.ERROR
            message = "joint count is zero"
            reason = "zero_joints"
        elif not self.has_pos:
            level = DiagnosticStatus.ERROR
            message = "position array missing"
            reason = "missing_position_array"
        elif (
            self.period_non_finite_pos
            or self.period_non_finite_vel
            or self.period_non_finite_effort
        ):
            level = DiagnosticStatus.ERROR
            message = "non-finite joint values detected"
            reason = "non_finite"
        elif rate < min_rate:
            level = DiagnosticStatus.WARN
            message = f"low rate {rate:.1f}Hz"
            reason = "low_rate"
        elif self.period_missing_vel or self.period_missing_effort:
            level = DiagnosticStatus.WARN
            message = "missing velocity or effort arrays"
            reason = "missing_arrays"
        elif self.period_position_jumps:
            level = DiagnosticStatus.WARN
            message, reason = self._abnormal_joints_message(self.period_position_jumps)
        elif self.period_high_vel:
            level = DiagnosticStatus.WARN
            message, reason = self._abnormal_joints_message(self.period_high_vel)
        elif self.period_high_effort:
            level = DiagnosticStatus.WARN
            message, reason = self._abnormal_joints_message(self.period_high_effort)

        if reason:
            self.last_warning_reason = reason

        abnormal_joints: dict[str, float] = {}
        if self.period_position_jumps:
            abnormal_joints = self.period_position_jumps
        elif self.period_high_vel:
            abnormal_joints = self.period_high_vel
        elif self.period_high_effort:
            abnormal_joints = self.period_high_effort

        values: list[KeyValue] = [
            kv("topic", self.topic),
            kv("expected_rate_hz", f"{self.params['expected_rate_hz']:.1f}"),
            kv("measured_rate_hz", f"{rate:.1f}"),
            kv("last_message_age_s", "n/a" if age is None else f"{age:.3f}"),
            kv("joint_count", self.joint_count),
            kv("has_position", self.has_pos),
            kv("has_velocity", self.has_vel),
            kv("has_effort", self.has_effort),
            kv("max_abs_velocity_rad_s_observed", f"{self.max_abs_vel_observed:.3f}"),
            kv("max_abs_effort_nm_observed", f"{self.max_abs_effort_observed:.3f}"),
            kv("last_warning_reason", self.last_warning_reason or "none"),
        ]
        if abnormal_joints:
            values.append(kv("abnormal_joint_count", len(abnormal_joints)))
            values.append(
                kv("abnormal_joint_names", ", ".join(sorted(abnormal_joints)))
            )
        for joint, jump in sorted(self.period_position_jumps.items()):
            values.append(kv(f"{joint}_position_jump_rad", f"{jump:.3f}"))
        for joint, vel in sorted(self.period_high_vel.items()):
            values.append(kv(f"{joint}_velocity_rad_s", f"{vel:.3f}"))
        for joint, effort in sorted(self.period_high_effort.items()):
            values.append(kv(f"{joint}_effort_nm", f"{effort:.3f}"))

        status = DiagnosticStatus()
        status.name = self.DIAG_NAME
        status.hardware_id = "rebotarm_monitor"
        status.level = level
        status.message = message
        status.values = values
        return status

    def summary_line(self, now: float) -> Optional[str]:
        rate = self.rate.measured_hz()
        age = None if self.last_recv_mono is None else now - self.last_recv_mono

        if self.last_recv_mono is None:
            return "WARN joint_states no messages received yet"
        if age is not None and age > self.params["stale_timeout_s"]:
            return f"WARN joint_states stale age={age:.2f}s"
        min_rate = self.params["expected_rate_hz"] * self.params["min_rate_ratio"]
        if rate < min_rate:
            return (
                f"WARN joint_states low_rate rate={rate:.1f}Hz "
                f"expected>={min_rate:.1f}Hz"
            )
        st = self.build_status(now, TrackerContext())
        if st.level == DiagnosticStatus.OK:
            return (
                "OK joint_states alive "
                f"rate={rate:.1f}Hz joints={self.joint_count} "
                f"pos={'yes' if self.has_pos else 'no'} "
                f"vel={'yes' if self.has_vel else 'no'} "
                f"effort={'yes' if self.has_effort else 'no'}"
            )
        return f"WARN joint_states {st.message}"
