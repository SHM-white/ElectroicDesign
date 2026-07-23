"""Tests for the mission executor preflight validation."""

from __future__ import annotations

import pytest

from ed_uav_mission.executor import PreflightCode, validate_preflight


def test_executor_requires_valid_start() -> None:
    """Stale AUX switch causes preflight rejection."""
    result = validate_preflight(
        fcu_communication_ok=True,
        fcu_motors_armed=True,
        aux_start_active=False,
        localization_active=True,
        map_to_odom_valid=True,
        profile_loaded=True,
        calibration_valid=True,
    )
    assert result.code == PreflightCode.STALE_AUX
    assert "aux" in result.reason.lower()


def test_preflight_rejects_no_fcu_link() -> None:
    result = validate_preflight(
        fcu_communication_ok=False,
        fcu_motors_armed=True,
        aux_start_active=True,
        localization_active=True,
        map_to_odom_valid=True,
        profile_loaded=True,
        calibration_valid=True,
    )
    assert result.code == PreflightCode.NO_FCU_LINK


def test_preflight_rejects_lost_localization() -> None:
    result = validate_preflight(
        fcu_communication_ok=True,
        fcu_motors_armed=True,
        aux_start_active=True,
        localization_active=False,
        map_to_odom_valid=True,
        profile_loaded=True,
        calibration_valid=True,
    )
    assert result.code == PreflightCode.LOCALIZATION_LOST


def test_preflight_rejects_invalid_map_to_odom() -> None:
    result = validate_preflight(
        fcu_communication_ok=True,
        fcu_motors_armed=True,
        aux_start_active=True,
        localization_active=True,
        map_to_odom_valid=False,
        profile_loaded=True,
        calibration_valid=True,
    )
    assert result.code == PreflightCode.LOCALIZATION_LOST


def test_preflight_rejects_missing_profile() -> None:
    result = validate_preflight(
        fcu_communication_ok=True,
        fcu_motors_armed=True,
        aux_start_active=True,
        localization_active=True,
        map_to_odom_valid=True,
        profile_loaded=False,
        calibration_valid=True,
    )
    assert result.code == PreflightCode.PROFILE_INVALID


def test_preflight_all_clear() -> None:
    result = validate_preflight(
        fcu_communication_ok=True,
        fcu_motors_armed=True,
        aux_start_active=True,
        localization_active=True,
        map_to_odom_valid=True,
        profile_loaded=True,
        calibration_valid=True,
    )
    assert result.code == PreflightCode.OK
