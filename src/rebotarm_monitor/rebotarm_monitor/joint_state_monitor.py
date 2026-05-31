from __future__ import annotations

import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from .health_trackers import (
    ArmStatusTracker,
    GripperTracker,
    JointStatesTracker,
    PerJointTracker,
)


class JointStateMonitor(Node):
    """Passive hardware health monitor — subscribes to existing driver topics only."""

    def __init__(self) -> None:
        super().__init__("rebotarm_joint_state_monitor")
        self._declare_parameters()
        p = self._load_params()

        self._log_only_on_change = p["log_only_on_change"]
        self._last_log_line: Optional[str] = None
        self._last_warn_mono: dict[str, float] = {}
        self._warn_cooldown_s = p["status_log_period_s"]

        diag_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", diag_qos)

        arm_latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._joint_states: Optional[JointStatesTracker] = None
        self._per_joints: list[PerJointTracker] = []
        self._arm_status: Optional[ArmStatusTracker] = None
        self._gripper: Optional[GripperTracker] = None

        if p["enable_joint_states_monitor"]:
            js_params = {
                "expected_rate_hz": p["expected_rate_hz"],
                "min_rate_ratio": p["min_rate_ratio"],
                "stale_timeout_s": p["stale_timeout_s"],
                "max_position_jump_rad": p["max_position_jump_rad"],
                "max_abs_velocity_rad_s": p["max_abs_velocity_rad_s"],
                "max_abs_effort_nm": p["max_abs_effort_nm"],
            }
            self._joint_states = JointStatesTracker(p["joint_states_topic"], js_params)
            self.create_subscription(
                JointState,
                p["joint_states_topic"],
                self._joint_states.on_message,
                qos_profile_sensor_data,
            )

        if p["enable_per_joint_monitor"]:
            pj_params = {
                "per_joint_stale_timeout_s": p["per_joint_stale_timeout_s"],
                "max_abs_joint_velocity_rad_s": p["max_abs_joint_velocity_rad_s"],
                "max_abs_joint_torque_nm": p["max_abs_joint_torque_nm"],
                "idle_velocity_threshold_rad_s": p["idle_velocity_threshold_rad_s"],
                "idle_torque_warn_nm": p["idle_torque_warn_nm"],
                "max_joint_position_jump_rad": p["max_joint_position_jump_rad"],
                "max_joint_torque_jump_nm": p["max_joint_torque_jump_nm"],
                "expected_enabled_status_code": p["expected_enabled_status_code"],
                "allow_disabled_status_code": p["allow_disabled_status_code"],
                "disabled_status_code": p["disabled_status_code"],
            }
            prefix = p["joint_state_topic_prefix"].rstrip("/")
            for name in p["joint_names"]:
                topic = f"{prefix}/{name}/state"
                tracker = PerJointTracker(name, topic, pj_params)
                self._per_joints.append(tracker)
                self.create_subscription(
                    JointMotorState,
                    topic,
                    tracker.on_message,
                    qos_profile_sensor_data,
                )

        if p["enable_arm_status_monitor"]:
            arm_params = {
                "arm_status_stale_timeout_s": p["arm_status_stale_timeout_s"],
                "arm_status_warn_on_snapshot_age": p["arm_status_warn_on_snapshot_age"],
                "expect_arm_enabled": p["expect_arm_enabled"],
                "expect_control_loop_active": p["expect_control_loop_active"],
            }
            self._arm_status = ArmStatusTracker(p["arm_status_topic"], arm_params)
            self.create_subscription(
                ArmStatus,
                p["arm_status_topic"],
                self._arm_status.on_message,
                arm_latched_qos,
            )

        if p["enable_gripper_monitor"]:
            g_params = {
                "gripper_stale_timeout_s": p["gripper_stale_timeout_s"],
                "max_abs_gripper_velocity": p["max_abs_gripper_velocity"],
                "max_abs_gripper_torque": p["max_abs_gripper_torque"],
            }
            self._gripper = GripperTracker(p["gripper_state_topic"], g_params)
            self.create_subscription(
                JointMotorState,
                p["gripper_state_topic"],
                self._gripper.on_message,
                qos_profile_sensor_data,
            )

        status_period = max(p["status_log_period_s"], 0.1)
        diag_period = (
            status_period
            if p["diagnostics_period_s"] <= 0.0
            else max(p["diagnostics_period_s"], 0.1)
        )
        self._diagnostics_on_status_timer = abs(diag_period - status_period) <= 1e-6

        self.create_timer(status_period, self._status_timer_callback)
        if not self._diagnostics_on_status_timer:
            self.create_timer(diag_period, self._diagnostics_timer_callback)

        self.get_logger().info(
            "passive health monitor active "
            f"(joint_states={p['enable_joint_states_monitor']}, "
            f"per_joint={p['enable_per_joint_monitor']}, "
            f"arm_status={p['enable_arm_status_monitor']}, "
            f"gripper={p['enable_gripper_monitor']})"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("enable_joint_states_monitor", True)
        self.declare_parameter("enable_per_joint_monitor", True)
        self.declare_parameter("enable_arm_status_monitor", True)
        self.declare_parameter("enable_gripper_monitor", True)
        self.declare_parameter("log_only_on_change", True)

        self.declare_parameter("joint_states_topic", "/rebotarm/joint_states")
        self.declare_parameter("expected_rate_hz", 100.0)
        self.declare_parameter("stale_timeout_s", 0.5)
        self.declare_parameter("min_rate_ratio", 0.5)
        self.declare_parameter("max_position_jump_rad", 0.5)
        self.declare_parameter("max_abs_velocity_rad_s", 10.0)
        self.declare_parameter("max_abs_effort_nm", 8.0)

        self.declare_parameter("joint_state_topic_prefix", "/rebotarm/joints")
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter("per_joint_stale_timeout_s", 0.5)
        self.declare_parameter("max_abs_joint_velocity_rad_s", 10.0)
        self.declare_parameter("max_abs_joint_torque_nm", 8.0)
        self.declare_parameter("idle_velocity_threshold_rad_s", 0.05)
        self.declare_parameter("idle_torque_warn_nm", 3.0)
        self.declare_parameter("max_joint_position_jump_rad", 0.5)
        self.declare_parameter("max_joint_torque_jump_nm", 3.0)
        self.declare_parameter("expected_enabled_status_code", 1)
        self.declare_parameter("allow_disabled_status_code", True)
        self.declare_parameter("disabled_status_code", 0)

        self.declare_parameter("arm_status_topic", "/rebotarm/arm_status")
        self.declare_parameter("arm_status_stale_timeout_s", 1.0)
        self.declare_parameter("arm_status_warn_on_snapshot_age", False)
        self.declare_parameter("expect_arm_enabled", False)
        self.declare_parameter("expect_control_loop_active", False)

        self.declare_parameter("gripper_state_topic", "/rebotarm/gripper/state")
        self.declare_parameter("gripper_stale_timeout_s", 1.0)
        self.declare_parameter("max_abs_gripper_velocity", 10.0)
        self.declare_parameter("max_abs_gripper_torque", 5.0)

        self.declare_parameter("status_log_period_s", 1.0)
        self.declare_parameter("diagnostics_period_s", 1.0)

    def _load_params(self) -> dict:
        joint_names = self.get_parameter("joint_names").value
        if isinstance(joint_names, str):
            joint_names = [n.strip() for n in joint_names.split(",") if n.strip()]
        return {
            "enable_joint_states_monitor": bool(
                self.get_parameter("enable_joint_states_monitor").value
            ),
            "enable_per_joint_monitor": bool(
                self.get_parameter("enable_per_joint_monitor").value
            ),
            "enable_arm_status_monitor": bool(
                self.get_parameter("enable_arm_status_monitor").value
            ),
            "enable_gripper_monitor": bool(
                self.get_parameter("enable_gripper_monitor").value
            ),
            "log_only_on_change": bool(self.get_parameter("log_only_on_change").value),
            "joint_states_topic": str(self.get_parameter("joint_states_topic").value),
            "expected_rate_hz": float(self.get_parameter("expected_rate_hz").value),
            "stale_timeout_s": float(self.get_parameter("stale_timeout_s").value),
            "min_rate_ratio": float(self.get_parameter("min_rate_ratio").value),
            "max_position_jump_rad": float(
                self.get_parameter("max_position_jump_rad").value
            ),
            "max_abs_velocity_rad_s": float(
                self.get_parameter("max_abs_velocity_rad_s").value
            ),
            "max_abs_effort_nm": float(self.get_parameter("max_abs_effort_nm").value),
            "joint_state_topic_prefix": str(
                self.get_parameter("joint_state_topic_prefix").value
            ),
            "joint_names": list(joint_names),
            "per_joint_stale_timeout_s": float(
                self.get_parameter("per_joint_stale_timeout_s").value
            ),
            "max_abs_joint_velocity_rad_s": float(
                self.get_parameter("max_abs_joint_velocity_rad_s").value
            ),
            "max_abs_joint_torque_nm": float(
                self.get_parameter("max_abs_joint_torque_nm").value
            ),
            "idle_velocity_threshold_rad_s": float(
                self.get_parameter("idle_velocity_threshold_rad_s").value
            ),
            "idle_torque_warn_nm": float(
                self.get_parameter("idle_torque_warn_nm").value
            ),
            "max_joint_position_jump_rad": float(
                self.get_parameter("max_joint_position_jump_rad").value
            ),
            "max_joint_torque_jump_nm": float(
                self.get_parameter("max_joint_torque_jump_nm").value
            ),
            "expected_enabled_status_code": int(
                self.get_parameter("expected_enabled_status_code").value
            ),
            "allow_disabled_status_code": bool(
                self.get_parameter("allow_disabled_status_code").value
            ),
            "disabled_status_code": int(
                self.get_parameter("disabled_status_code").value
            ),
            "arm_status_topic": str(self.get_parameter("arm_status_topic").value),
            "arm_status_stale_timeout_s": float(
                self.get_parameter("arm_status_stale_timeout_s").value
            ),
            "arm_status_warn_on_snapshot_age": bool(
                self.get_parameter("arm_status_warn_on_snapshot_age").value
            ),
            "expect_arm_enabled": bool(
                self.get_parameter("expect_arm_enabled").value
            ),
            "expect_control_loop_active": bool(
                self.get_parameter("expect_control_loop_active").value
            ),
            "gripper_state_topic": str(
                self.get_parameter("gripper_state_topic").value
            ),
            "gripper_stale_timeout_s": float(
                self.get_parameter("gripper_stale_timeout_s").value
            ),
            "max_abs_gripper_velocity": float(
                self.get_parameter("max_abs_gripper_velocity").value
            ),
            "max_abs_gripper_torque": float(
                self.get_parameter("max_abs_gripper_torque").value
            ),
            "status_log_period_s": float(
                self.get_parameter("status_log_period_s").value
            ),
            "diagnostics_period_s": float(
                self.get_parameter("diagnostics_period_s").value
            ),
        }

    def _arm_enabled(self) -> Optional[bool]:
        if self._arm_status is None or self._arm_status.last_msg is None:
            return None
        return bool(self._arm_status.last_msg.enabled)

    def _warn_throttled(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warn_mono.get(key, 0.0) < self._warn_cooldown_s:
            return
        self._last_warn_mono[key] = now
        self.get_logger().warn(message)

    def _log_line(self, line: str, *, is_warn: bool = False) -> None:
        if self._log_only_on_change and line == self._last_log_line:
            return
        self._last_log_line = line
        if is_warn:
            self.get_logger().warn(line)
        else:
            self.get_logger().info(line)

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        statuses: list[DiagnosticStatus] = []

        if self._joint_states is not None:
            statuses.append(self._joint_states.build_status(now))
            self._joint_states.reset_period()
            self._joint_states.rate.reset()

        arm_enabled = self._arm_enabled()
        for tracker in self._per_joints:
            statuses.append(
                tracker.build_status(
                    now,
                    arm_enabled,
                )
            )
            tracker.reset_period()

        if self._arm_status is not None:
            statuses.append(self._arm_status.build_status(now))

        if self._gripper is not None:
            statuses.append(self._gripper.build_status(now))
            self._gripper.reset_period()

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = statuses
        self._diag_pub.publish(array)

    def _emit_console_logs(self, now: float) -> None:
        if self._joint_states is None:
            return

        line = self._joint_states.summary_line(now)
        if line is None:
            return
        is_warn = line.startswith("WARN")
        if is_warn:
            key = line.split()[1] if len(line.split()) > 1 else "warn"
            self._warn_throttled(key, line)
        else:
            self._log_line(line)

        for tracker in self._per_joints:
            st = tracker.build_status(
                now,
                self._arm_enabled(),
            )
            if st.level == DiagnosticStatus.WARN:
                self._warn_throttled(
                    tracker.diag_name,
                    f"WARN {tracker.joint_name}: {st.message}",
                )
            elif st.level == DiagnosticStatus.ERROR:
                self._warn_throttled(
                    tracker.diag_name,
                    f"WARN {tracker.joint_name}: {st.message}",
                )

    def _status_timer_callback(self) -> None:
        now = time.monotonic()
        self._emit_console_logs(now)
        if self._diagnostics_on_status_timer:
            self._publish_diagnostics()
        elif self._joint_states is not None:
            self._joint_states.rate.reset()

    def _diagnostics_timer_callback(self) -> None:
        self._publish_diagnostics()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
