"""Resolved per-joint check limits (single source for trackers and messages).

Built only from the flat ``params`` dict that :func:`factories.build_trackers`
already passes into :class:`PerJointTracker` (globals + per-joint overrides
applied via :func:`parameters.resolve_joint_threshold`). Future robot profiles
(e.g. Juggler) change limits upstream in ``parameters`` / factory wiring, not here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class PerJointLimitsView:
    """Read-only limits for one joint tracker instance."""

    max_abs_torque_nm: float
    max_abs_velocity_rad_s: float
    idle_torque_warn_nm: float
    idle_velocity_threshold_rad_s: float
    max_position_jump_rad: float
    max_torque_jump_nm: float

    @classmethod
    def from_tracker_params(cls, params: dict[str, Any]) -> PerJointLimitsView:
        return cls(
            max_abs_torque_nm=float(params["max_abs_joint_torque_nm"]),
            max_abs_velocity_rad_s=float(params["max_abs_joint_velocity_rad_s"]),
            idle_torque_warn_nm=float(params["idle_torque_warn_nm"]),
            idle_velocity_threshold_rad_s=float(
                params["idle_velocity_threshold_rad_s"]
            ),
            max_position_jump_rad=float(params["max_joint_position_jump_rad"]),
            max_torque_jump_nm=float(params["max_joint_torque_jump_nm"]),
        )
