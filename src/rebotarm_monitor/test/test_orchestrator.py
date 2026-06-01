"""Tests for :class:`MonitorOrchestrator`."""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticStatus

from rebotarm_monitor.domain.tracker import HealthTracker, TrackerContext
from rebotarm_monitor.orchestrator import MonitorOrchestrator
from rebotarm_monitor.trackers.arm_status import ArmStatusTracker
from conftest import make_arm_status


class StubTracker(HealthTracker):
    def __init__(
        self,
        name: str = "stub/x",
        level: bytes = DiagnosticStatus.OK,
    ) -> None:
        self._name = name
        self._level = level
        self.last_context = None
        self.subscriptions_registered = 0
        self.reset_calls = 0

    @property
    def diag_name(self) -> str:
        return self._name

    def register_subscriptions(self, registrar) -> None:  # noqa: ANN001
        self.subscriptions_registered += 1

    def build_status(self, now: float, context: TrackerContext) -> DiagnosticStatus:
        self.last_context = context
        status = DiagnosticStatus()
        status.name = self._name
        status.level = self._level
        status.message = "stub"
        return status

    def reset_period(self) -> None:
        self.reset_calls += 1


def test_register_subscriptions_calls_every_tracker():
    a, b = StubTracker("a"), StubTracker("b")
    orchestrator = MonitorOrchestrator([a, b])
    orchestrator.register_subscriptions(object())
    assert a.subscriptions_registered == 1
    assert b.subscriptions_registered == 1


def test_build_statuses_returns_one_per_tracker():
    orchestrator = MonitorOrchestrator(
        [StubTracker("a"), StubTracker("b", DiagnosticStatus.WARN)]
    )
    statuses = orchestrator.build_statuses(now=0.0)
    assert [s.name for s in statuses] == ["a", "b"]
    assert statuses[0].level == DiagnosticStatus.OK
    assert statuses[1].level == DiagnosticStatus.WARN


def test_context_carries_arm_enabled_from_arm_status_tracker():
    arm = ArmStatusTracker(
        "/rebotarm/arm_status",
        {
            "arm_status_stale_timeout_s": 1.0,
            "arm_status_warn_on_snapshot_age": False,
            "expect_arm_enabled": False,
            "expect_control_loop_active": False,
        },
    )
    arm.on_message(make_arm_status(enabled=False))
    stub = StubTracker()
    orchestrator = MonitorOrchestrator([arm, stub])
    orchestrator.build_statuses(now=time.monotonic())
    assert stub.last_context.arm_enabled is False


def test_context_gravity_compensation_active_when_state_is_gravity_comp():
    arm = ArmStatusTracker(
        "/rebotarm/arm_status",
        {
            "arm_status_stale_timeout_s": 1.0,
            "arm_status_warn_on_snapshot_age": False,
            "expect_arm_enabled": False,
            "expect_control_loop_active": False,
        },
    )
    arm.on_message(make_arm_status(state_machine="GRAVITY_COMP"))
    stub = StubTracker()
    orchestrator = MonitorOrchestrator([arm, stub])
    orchestrator.build_statuses(now=time.monotonic())
    assert stub.last_context.gravity_compensation_active is True
    assert stub.last_context.position_hold_active is False
    assert stub.last_context.control_context == "gravity_compensation"


def test_context_position_hold_when_pos_vel_enabled_and_loop_active():
    arm = ArmStatusTracker(
        "/rebotarm/arm_status",
        {
            "arm_status_stale_timeout_s": 1.0,
            "arm_status_warn_on_snapshot_age": False,
            "expect_arm_enabled": False,
            "expect_control_loop_active": False,
        },
    )
    arm.on_message(
        make_arm_status(
            mode="pos_vel",
            enabled=True,
            control_loop_active=True,
            state_machine="IDLE",
        )
    )
    stub = StubTracker()
    orchestrator = MonitorOrchestrator([arm, stub])
    orchestrator.build_statuses(now=time.monotonic())
    assert stub.last_context.position_hold_active is True
    assert stub.last_context.gravity_compensation_active is False
    assert stub.last_context.control_context == "position_hold"


def test_context_gravity_compensation_inactive_when_state_is_idle():
    arm = ArmStatusTracker(
        "/rebotarm/arm_status",
        {
            "arm_status_stale_timeout_s": 1.0,
            "arm_status_warn_on_snapshot_age": False,
            "expect_arm_enabled": False,
            "expect_control_loop_active": False,
        },
    )
    arm.on_message(make_arm_status(state_machine="IDLE"))
    stub = StubTracker()
    orchestrator = MonitorOrchestrator([arm, stub])
    orchestrator.build_statuses(now=time.monotonic())
    assert stub.last_context.gravity_compensation_active is False
    assert stub.last_context.control_context == "normal_or_unknown"


def test_mit_mode_without_gravity_comp_state_does_not_activate_context_flag():
    arm = ArmStatusTracker(
        "/rebotarm/arm_status",
        {
            "arm_status_stale_timeout_s": 1.0,
            "arm_status_warn_on_snapshot_age": False,
            "expect_arm_enabled": False,
            "expect_control_loop_active": False,
        },
    )
    arm.on_message(make_arm_status(mode="mit", state_machine="IDLE"))
    stub = StubTracker()
    orchestrator = MonitorOrchestrator([arm, stub])
    orchestrator.build_statuses(now=time.monotonic())
    assert stub.last_context.gravity_compensation_active is False
    assert stub.last_context.position_hold_active is False
    assert stub.last_context.control_context == "normal_or_unknown"


def test_reset_periods_calls_every_tracker():
    a, b = StubTracker(), StubTracker()
    orchestrator = MonitorOrchestrator([a, b])
    orchestrator.reset_periods()
    assert a.reset_calls == 1
    assert b.reset_calls == 1


def test_find_by_diag_name_returns_match_or_none():
    a = StubTracker("rebotarm/x")
    orchestrator = MonitorOrchestrator([a])
    assert orchestrator.find_by_diag_name("rebotarm/x") is a
    assert orchestrator.find_by_diag_name("nope") is None
