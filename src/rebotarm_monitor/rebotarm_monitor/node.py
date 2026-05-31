"""Thin ROS 2 adapter that wires the orchestrator to rclpy.

All monitoring logic lives in ``trackers/`` and ``orchestrator.py``. This file
only handles ROS plumbing: parameters, publisher, timers, console logging.
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray

from .factories import build_trackers
from .orchestrator import MonitorOrchestrator
from .parameters import declare_parameters, load_params
from .trackers.joint_states import JointStatesTracker
from .trackers.per_joint import PerJointTracker


class MonitorNode(Node):
    """Composition root: orchestrator + ROS subscriptions, publisher, timers."""

    def __init__(self) -> None:
        super().__init__("rebotarm_monitor")
        declare_parameters(self)
        p = load_params(self)
        self._params = p

        self._log_only_on_change = p["log_only_on_change"]
        self._last_log_line: Optional[str] = None
        self._last_warn_mono: dict[str, float] = {}
        self._warn_cooldown_s = p["status_log_period_s"]

        diag_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._diag_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", diag_qos
        )

        self._orchestrator = MonitorOrchestrator(build_trackers(p))
        self._orchestrator.register_subscriptions(self)

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
            f"gripper={p['enable_gripper_monitor']}, "
            f"can={p['enable_can_monitor']}:{p['can_interfaces']}, "
            f"process={p['enable_process_monitor']}:'{p['driver_process_pattern']}')"
        )

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
        array = self._orchestrator.build_diagnostic_array(
            header_stamp=self.get_clock().now().to_msg(),
            now=now,
        )
        self._diag_pub.publish(array)
        self._orchestrator.reset_periods()

    def _emit_console_logs(self, now: float) -> None:
        for tracker in self._orchestrator.trackers:
            if isinstance(tracker, JointStatesTracker):
                line = tracker.summary_line(now)
                if line is None:
                    continue
                is_warn = line.startswith("WARN")
                if is_warn:
                    key = line.split()[1] if len(line.split()) > 1 else "warn"
                    self._warn_throttled(key, line)
                else:
                    self._log_line(line)
            elif isinstance(tracker, PerJointTracker):
                st = tracker.build_status(now, self._orchestrator._build_context())
                if st.level in (DiagnosticStatus.WARN, DiagnosticStatus.ERROR):
                    self._warn_throttled(
                        tracker.diag_name,
                        f"WARN {tracker.joint_name}: {st.message}",
                    )

    def _status_timer_callback(self) -> None:
        now = time.monotonic()
        self._emit_console_logs(now)
        if self._diagnostics_on_status_timer:
            self._publish_diagnostics()

    def _diagnostics_timer_callback(self) -> None:
        self._publish_diagnostics()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MonitorNode()
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
