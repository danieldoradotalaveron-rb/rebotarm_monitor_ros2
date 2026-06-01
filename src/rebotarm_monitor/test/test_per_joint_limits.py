"""Tests for :class:`PerJointLimitsView`."""

from __future__ import annotations

import pytest

from rebotarm_monitor.domain.per_joint_limits import PerJointLimitsView
from rebotarm_monitor.parameters import _PARAM_SPECS, per_joint_threshold_maps


_REQUIRED_KEYS = (
    "max_abs_joint_torque_nm",
    "max_abs_joint_velocity_rad_s",
    "idle_torque_warn_nm",
    "idle_velocity_threshold_rad_s",
    "max_joint_position_jump_rad",
    "max_joint_torque_jump_nm",
)


def test_from_tracker_params_uses_resolved_per_joint_values():
    limits = PerJointLimitsView.from_tracker_params(
        {
            "max_abs_joint_torque_nm": 9.0,
            "max_abs_joint_velocity_rad_s": 10.0,
            "idle_torque_warn_nm": 3.0,
            "idle_velocity_threshold_rad_s": 0.05,
            "max_joint_position_jump_rad": 0.5,
            "max_joint_torque_jump_nm": 3.0,
        }
    )
    assert limits.max_abs_torque_nm == 9.0
    assert limits.idle_torque_warn_nm == 3.0


def test_param_specs_declare_every_key_per_joint_limits_view_needs():
    """Renaming a key in ``_PARAM_SPECS`` must keep the limits view contract."""
    declared = {name for name, _default in _PARAM_SPECS}
    missing = [key for key in _REQUIRED_KEYS if key not in declared]
    assert not missing, (
        f"keys consumed by PerJointLimitsView but not declared in _PARAM_SPECS: {missing}"
    )


def test_b601_threshold_maps_only_contain_known_joint_names():
    """The B601 map must not reference joints outside the declared ``joint_names``."""
    declared = dict(_PARAM_SPECS)
    joint_names = set(declared["joint_names"])
    max_map, idle_map = per_joint_threshold_maps()
    unknown_max = set(max_map) - joint_names
    unknown_idle = set(idle_map) - joint_names
    assert not unknown_max, f"unknown joints in max-torque map: {unknown_max}"
    assert not unknown_idle, f"unknown joints in idle-torque map: {unknown_idle}"


def test_missing_key_raises_keyerror():
    with pytest.raises(KeyError):
        PerJointLimitsView.from_tracker_params(
            {
                "max_abs_joint_velocity_rad_s": 10.0,
                "idle_torque_warn_nm": 3.0,
                "idle_velocity_threshold_rad_s": 0.05,
                "max_joint_position_jump_rad": 0.5,
                "max_joint_torque_jump_nm": 3.0,
            }
        )
