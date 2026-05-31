"""Shared pytest fixtures and lightweight message factories.

The trackers consume attribute-bag messages (``position``, ``velocity``,
``status_code`` ...). We use ``SimpleNamespace`` instead of the real generated
message classes so tests stay focused on the tracker logic and remain easy to
parameterise (e.g. injecting NaN, oversized arrays, missing fields).
"""

from __future__ import annotations

from types import SimpleNamespace
from typing import Optional


def make_motor_state(
    position: float = 0.0,
    velocity: float = 0.0,
    torque: float = 0.0,
    status_code: int = 1,
) -> SimpleNamespace:
    """Mimic ``rebotarm_msgs/JointMotorState``."""
    return SimpleNamespace(
        position=position,
        velocity=velocity,
        torque=torque,
        status_code=status_code,
    )


def make_joint_state(
    names: list[str],
    positions: Optional[list[float]] = None,
    velocities: Optional[list[float]] = None,
    efforts: Optional[list[float]] = None,
) -> SimpleNamespace:
    """Mimic ``sensor_msgs/JointState``."""
    return SimpleNamespace(
        name=list(names),
        position=list(positions if positions is not None else []),
        velocity=list(velocities if velocities is not None else []),
        effort=list(efforts if efforts is not None else []),
    )


def make_arm_status(
    enabled: bool = True,
    mode: int = 0,
    control_loop_active: bool = True,
    state_machine: int = 0,
    error_codes: Optional[list[int]] = None,
    per_joint_status_code: Optional[list[int]] = None,
) -> SimpleNamespace:
    """Mimic ``rebotarm_msgs/ArmStatus``."""
    return SimpleNamespace(
        enabled=enabled,
        mode=mode,
        control_loop_active=control_loop_active,
        state_machine=state_machine,
        error_codes=list(error_codes or []),
        per_joint_status_code=list(per_joint_status_code or []),
    )
