"""Steady-clock lidar transport health evaluation."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class HealthState:
    """Latest locally observed events; all times use a local steady clock."""

    driver_alive: bool
    last_driver_steady_ns: int
    last_point_steady_ns: int
    last_imu_steady_ns: int


@dataclass(frozen=True, slots=True)
class HealthReport:
    code: str
    active: bool


def evaluate_health(
    state: HealthState, now_steady_ns: int, deadline_ns: int
) -> HealthReport:
    """Evaluate transport liveness without subtracting ROS acquisition timestamps."""
    if not state.driver_alive:
        return HealthReport(code="LIDAR_DRIVER_DEAD", active=False)
    if now_steady_ns - state.last_driver_steady_ns > deadline_ns:
        return HealthReport(code="LIDAR_DRIVER_TIMEOUT", active=False)
    if now_steady_ns - state.last_point_steady_ns > deadline_ns:
        return HealthReport(code="LIDAR_POINT_STALE", active=False)
    if now_steady_ns - state.last_imu_steady_ns > deadline_ns:
        return HealthReport(code="LIDAR_IMU_STALE", active=False)
    return HealthReport(code="LIDAR_ACTIVE", active=True)
