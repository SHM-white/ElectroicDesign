"""Deterministic lidar health contract tests."""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_lidar.health import HealthState, evaluate_health


def test_reports_stale_imu_from_injected_steady_time() -> None:
    # Given: live point traffic and an IMU older than the configured deadline.
    state = HealthState(
        driver_alive=True,
        last_driver_steady_ns=1_400,
        last_point_steady_ns=1_400,
        last_imu_steady_ns=100,
    )

    # When: health is evaluated with a deterministic local steady clock.
    report = evaluate_health(state, now_steady_ns=1_500, deadline_ns=200)

    # Then: stale IMU is the actionable failure, independent of ROS clock time.
    assert report.code == "LIDAR_IMU_STALE"
    assert report.active is False


def test_reports_driver_death_without_reusing_stale_input() -> None:
    # Given: a transport state whose supervised driver has exited.
    state = HealthState(
        driver_alive=False,
        last_driver_steady_ns=1_000,
        last_point_steady_ns=1_000,
        last_imu_steady_ns=1_000,
    )

    # When: health evaluates the driver process state.
    report = evaluate_health(state, now_steady_ns=1_001, deadline_ns=200)

    # Then: it reports driver death and never treats retained points as fresh.
    assert report.code == "LIDAR_DRIVER_DEAD"
    assert report.active is False


def test_reports_hung_driver_timeout_from_injected_steady_time() -> None:
    # Given: an alive process with no heartbeat inside the watchdog deadline.
    state = HealthState(
        driver_alive=True,
        last_driver_steady_ns=100,
        last_point_steady_ns=1_000,
        last_imu_steady_ns=1_000,
    )

    # When: the injected steady clock crosses the timeout.
    report = evaluate_health(state, now_steady_ns=500, deadline_ns=200)

    # Then: the diagnostic distinguishes a hang from an observed exit.
    assert report.code == "LIDAR_DRIVER_TIMEOUT"
    assert report.active is False
