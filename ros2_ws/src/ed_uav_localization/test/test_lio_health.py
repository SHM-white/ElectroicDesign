"""Tests for LIO health monitoring logic."""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_localization.lio_health import (
    LIOHealth,
    evaluate_health,
)


def test_health_healthy_state() -> None:
    """Fresh inputs produce HEALTHY."""
    health = evaluate_health(
        odom_age_sec=0.05,
        imu_age_sec=0.02,
        time_regression=False,
        covariance_finite=True,
        covariance_exceeds=False,
        no_odom_duration_sec=0.05,
    )
    assert health == LIOHealth.HEALTHY


def test_health_degraded_on_stale_odom() -> None:
    """An odometry message older than the error threshold yields DEGRADED."""
    health = evaluate_health(
        odom_age_sec=0.60,
        imu_age_sec=0.02,
        time_regression=False,
        covariance_finite=True,
        covariance_exceeds=False,
        no_odom_duration_sec=0.60,
    )
    assert health == LIOHealth.DEGRADED


def test_health_lost_on_no_odom() -> None:
    """Absence of valid odometry for longer than the lost timeout yields LOST."""
    health = evaluate_health(
        odom_age_sec=1.10,
        imu_age_sec=0.02,
        time_regression=False,
        covariance_finite=True,
        covariance_exceeds=False,
        no_odom_duration_sec=1.10,
    )
    assert health == LIOHealth.LOST


def test_health_detects_time_regression() -> None:
    """Non-monotonic timestamps produce DEGRADED even when inputs are fresh."""
    health = evaluate_health(
        odom_age_sec=0.05,
        imu_age_sec=0.02,
        time_regression=True,
        covariance_finite=True,
        covariance_exceeds=False,
        no_odom_duration_sec=0.05,
    )
    assert health == LIOHealth.DEGRADED


def test_health_detects_covariance_blowup() -> None:
    """A covariance diagonal element exceeding the blowup threshold yields LOST."""
    health = evaluate_health(
        odom_age_sec=0.05,
        imu_age_sec=0.02,
        time_regression=False,
        covariance_finite=True,
        covariance_exceeds=True,
        no_odom_duration_sec=0.05,
    )
    assert health == LIOHealth.LOST
