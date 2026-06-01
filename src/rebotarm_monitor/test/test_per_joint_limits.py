"""Tests for :class:`PerJointLimitsView` and B601 per-joint threshold maps."""

from __future__ import annotations

import pytest

from rebotarm_monitor.domain.per_joint_limits import PerJointLimitsView
from rebotarm_monitor.parameters import (
    DEFAULT_PAYLOAD_PROFILE,
    PAYLOAD_PROFILES,
    _PARAM_SPECS,
    normalize_payload_profile,
    per_joint_threshold_maps,
)


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


def test_param_specs_declare_payload_profile():
    """``payload_profile`` is the only ROS-level switch for B601 maps."""
    declared = {name: default for name, default in _PARAM_SPECS}
    assert "payload_profile" in declared
    assert declared["payload_profile"] == DEFAULT_PAYLOAD_PROFILE


def test_default_payload_profile_is_light():
    """Default preserves the previously shipped behaviour."""
    assert DEFAULT_PAYLOAD_PROFILE == "light"


def test_payload_profiles_expose_known_set():
    """Three profiles ship: light / medium / rated."""
    assert set(PAYLOAD_PROFILES) == {"light", "medium", "rated"}


@pytest.mark.parametrize("profile", PAYLOAD_PROFILES)
def test_b601_threshold_maps_only_contain_known_joint_names(profile):
    """Every B601 map must not reference joints outside ``joint_names``."""
    declared = dict(_PARAM_SPECS)
    joint_names = set(declared["joint_names"])
    for key, mapping in per_joint_threshold_maps(profile).items():
        unknown = set(mapping) - joint_names
        assert not unknown, f"unknown joints in {profile}/{key}: {unknown}"


@pytest.mark.parametrize("profile", PAYLOAD_PROFILES)
def test_b601_velocity_map_is_profile_independent(profile):
    """Velocity envelope tracks the motor, not the payload."""
    velocity = per_joint_threshold_maps(profile)["per_joint_max_abs_velocity_rad_s"]
    for shoulder in ("joint1", "joint2", "joint3"):
        assert velocity[shoulder] == 6.0
    for wrist in ("joint4", "joint5", "joint6"):
        assert velocity[wrist] == 20.0


# Each profile's per-joint torque thresholds are locked here as a contract.
# Updating any value below requires updating docs/per-joint-thresholds.md too.
_PROFILE_EXPECTED_IDLE: dict[str, dict[str, float]] = {
    "light": {
        "joint1": 1.0, "joint2": 8.0, "joint3": 8.0,
        "joint4": 2.5, "joint5": 1.5, "joint6": 1.0,
    },
    "medium": {
        "joint1": 1.0, "joint2": 10.0, "joint3": 10.0,
        "joint4": 3.0, "joint5": 2.0, "joint6": 1.0,
    },
    "rated": {
        "joint1": 1.0, "joint2": 14.0, "joint3": 12.0,
        "joint4": 4.0, "joint5": 2.5, "joint6": 1.0,
    },
}

_PROFILE_EXPECTED_MAX: dict[str, dict[str, float]] = {
    "light": {
        "joint1": 9.0, "joint2": 9.0, "joint3": 9.0,
        "joint4": 3.0, "joint5": 3.0, "joint6": 3.0,
    },
    "medium": {
        "joint1": 12.0, "joint2": 12.0, "joint3": 12.0,
        "joint4": 4.0, "joint5": 4.0, "joint6": 4.0,
    },
    "rated": {
        "joint1": 18.0, "joint2": 18.0, "joint3": 18.0,
        "joint4": 5.5, "joint5": 5.5, "joint6": 5.5,
    },
}


@pytest.mark.parametrize("profile", PAYLOAD_PROFILES)
def test_b601_idle_torque_map_matches_locked_profile(profile):
    maps = per_joint_threshold_maps(profile)
    assert maps["per_joint_idle_torque_warn_nm"] == _PROFILE_EXPECTED_IDLE[profile]


@pytest.mark.parametrize("profile", PAYLOAD_PROFILES)
def test_b601_max_torque_map_matches_locked_profile(profile):
    maps = per_joint_threshold_maps(profile)
    assert maps["per_joint_max_abs_torque_nm"] == _PROFILE_EXPECTED_MAX[profile]


@pytest.mark.parametrize("profile", PAYLOAD_PROFILES)
def test_idle_threshold_stays_below_max_in_every_profile(profile):
    """Sequence ``elevated -> WARN idle -> WARN high torque`` must stay ordered."""
    maps = per_joint_threshold_maps(profile)
    idle = maps["per_joint_idle_torque_warn_nm"]
    mx = maps["per_joint_max_abs_torque_nm"]
    for joint in idle:
        assert idle[joint] < mx[joint], (
            f"{profile}/{joint}: idle ({idle[joint]}) must be < max ({mx[joint]})"
        )


def test_default_profile_preserves_previously_shipped_idle_values():
    """Default (light) keeps the values monitor users are calibrated against."""
    maps = per_joint_threshold_maps()
    assert maps["per_joint_idle_torque_warn_nm"] == _PROFILE_EXPECTED_IDLE["light"]
    assert maps["per_joint_max_abs_torque_nm"] == _PROFILE_EXPECTED_MAX["light"]


def test_normalize_payload_profile_accepts_lowercase():
    assert normalize_payload_profile("rated") == "rated"


def test_normalize_payload_profile_normalises_case_and_whitespace():
    assert normalize_payload_profile(" LIGHT ") == "light"
    assert normalize_payload_profile("Medium") == "medium"


def test_normalize_payload_profile_raises_on_unknown():
    with pytest.raises(ValueError, match="unknown payload_profile"):
        normalize_payload_profile("heavy")


def test_per_joint_threshold_maps_raises_on_unknown_profile():
    with pytest.raises(ValueError):
        per_joint_threshold_maps("typo")


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
