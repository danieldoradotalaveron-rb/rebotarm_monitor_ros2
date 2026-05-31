from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from sensor_msgs.msg import JointState

from .diagnostics_util import is_finite, kv, max_level


@dataclass
class RateWindow:
    msg_count: int = 0
    window_start: float = field(default_factory=time.monotonic)

    def measured_hz(self) -> float:
        elapsed = time.monotonic() - self.window_start
        if elapsed <= 0.0:
            return 0.0
        return self.msg_count / elapsed

    def reset(self) -> None:
        self.msg_count = 0
        self.window_start = time.monotonic()


class JointStatesTracker:
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

        for i, name in enumerate(names):
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
                elif abs(vel) > self.params["max_abs_velocity_rad_s"]:
                    self.period_high_vel[name] = vel

            if self.has_effort:
                effort = float(msg.effort[i])
                self.max_abs_effort_observed = max(
                    self.max_abs_effort_observed, abs(effort)
                )
                if not is_finite(effort):
                    self.period_non_finite_effort = True
                elif abs(effort) > self.params["max_abs_effort_nm"]:
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

    def build_status(self, now: float) -> DiagnosticStatus:
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
            joint = next(iter(self.period_position_jumps))
            level = DiagnosticStatus.WARN
            message = f"position jump on {joint}"
            reason = f"position_jump:{joint}"
        elif self.period_high_vel:
            joint = next(iter(self.period_high_vel))
            level = DiagnosticStatus.WARN
            message = f"high velocity on {joint}"
            reason = f"high_velocity:{joint}"
        elif self.period_high_effort:
            joint = next(iter(self.period_high_effort))
            level = DiagnosticStatus.WARN
            message = f"high effort on {joint}"
            reason = f"high_effort:{joint}"

        if reason:
            self.last_warning_reason = reason

        values: list[KeyValue] = [
            kv("topic", self.topic),
            kv("expected_rate_hz", f"{self.params['expected_rate_hz']:.1f}"),
            kv("measured_rate_hz", f"{rate:.1f}"),
            kv("last_message_age_s", "n/a" if age is None else f"{age:.3f}"),
            kv("joint_count", self.joint_count),
            kv("has_position", self.has_pos),
            kv("has_velocity", self.has_vel),
            kv("has_effort", self.has_effort),
            kv(
                "max_abs_velocity_rad_s_observed",
                f"{self.max_abs_vel_observed:.3f}",
            ),
            kv(
                "max_abs_effort_nm_observed",
                f"{self.max_abs_effort_observed:.3f}",
            ),
            kv("last_warning_reason", self.last_warning_reason or "none"),
        ]
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
        st = self.build_status(now)
        rate = self.rate.measured_hz()
        age = None if self.last_recv_mono is None else now - self.last_recv_mono

        if st.level == DiagnosticStatus.OK:
            return (
                "OK joint_states alive "
                f"rate={rate:.1f}Hz joints={self.joint_count} "
                f"pos={'yes' if self.has_pos else 'no'} "
                f"vel={'yes' if self.has_vel else 'no'} "
                f"effort={'yes' if self.has_effort else 'no'}"
            )
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
        return f"WARN joint_states {st.message}"


class PerJointTracker:
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
        self.period_status_warn = False
        self.last_warning_reason = ""

    @property
    def diag_name(self) -> str:
        return f"rebotarm/joints/{self.joint_name}"

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
        self.period_status_warn = False

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

    def build_status(
        self,
        now: float,
        arm_enabled: Optional[bool],
    ) -> DiagnosticStatus:
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
            elif self.period_idle_torque:
                level = DiagnosticStatus.WARN
                message = "high torque while idle"
                reason = "idle_torque"

        if reason:
            self.last_warning_reason = reason

        msg = self.last_msg
        values: list[KeyValue] = [
            kv("topic", self.topic),
            kv("last_message_age_s", "n/a" if age is None else f"{age:.3f}"),
            kv(
                "max_abs_joint_velocity_rad_s",
                self.params["max_abs_joint_velocity_rad_s"],
            ),
            kv("max_abs_joint_torque_nm", self.params["max_abs_joint_torque_nm"]),
            kv(
                "idle_velocity_threshold_rad_s",
                self.params["idle_velocity_threshold_rad_s"],
            ),
            kv("idle_torque_warn_nm", self.params["idle_torque_warn_nm"]),
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


class ArmStatusTracker:
    DIAG_NAME = "rebotarm/hardware/arm_status"

    def __init__(self, topic: str, params: dict) -> None:
        self.topic = topic
        self.params = params
        self.last_recv_mono: Optional[float] = None
        self.last_msg: Optional[ArmStatus] = None
        self.last_warning_reason = ""

    def on_message(self, msg: ArmStatus) -> None:
        self.last_recv_mono = time.monotonic()
        self.last_msg = msg

    def build_status(self, now: float) -> DiagnosticStatus:
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
                f"mode={msg.mode} enabled={msg.enabled} "
                f"loop={msg.control_loop_active} state={msg.state_machine}"
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
            elif self.params["expect_control_loop_active"] and not msg.control_loop_active:
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


class GripperTracker:
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

    def build_status(self, now: float) -> DiagnosticStatus:
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
