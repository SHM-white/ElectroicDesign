"""FAST-LIO odometry health monitoring."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

from typing_extensions import assert_never

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class LIOHealth(IntEnum):
    """Health states for the LIO odometry pipeline."""

    HEALTHY = 0
    DEGRADED = 1
    LOST = 2


@dataclass
class HealthThresholds:
    """Configurable thresholds for LIO health evaluation.

    All durations are in seconds.
    """

    odom_age_warn: float = 0.15
    odom_age_error: float = 0.50
    imu_age_warn: float = 0.10
    lost_timeout: float = 1.0
    covariance_blowup: float = 1e6


# Indices of diagonal elements in the 36-element 6×6 flattened covariance matrix
# used by nav_msgs/Odometry.pose.covariance.
_COV_DIAG_IDX = (0, 7, 14, 21, 28, 35)


def _finite_covariance(cov: tuple[float, ...]) -> bool:
    """Return whether every covariance element is finite."""
    return all(math.isfinite(v) for v in cov)


def _covariance_blows_up(cov: tuple[float, ...], blowup: float) -> bool:
    """Return whether any diagonal covariance element exceeds *blowup*."""
    return any(i < len(cov) and abs(cov[i]) > blowup for i in _COV_DIAG_IDX)


def evaluate_health(
    *,
    odom_age_sec: Optional[float],
    imu_age_sec: Optional[float],
    time_regression: bool,
    covariance_finite: bool,
    covariance_exceeds: bool,
    no_odom_duration_sec: Optional[float],
    thresholds: Optional[HealthThresholds] = None,
) -> LIOHealth:
    """Evaluate LIO health from accumulated input state.

    All timing arguments are in seconds so callers can supply them without
    importing ROS types — helpful for deterministic unit tests.
    """
    if thresholds is None:
        thresholds = HealthThresholds()

    if not covariance_finite:
        return LIOHealth.LOST
    if covariance_exceeds:
        return LIOHealth.LOST

    if no_odom_duration_sec is not None and no_odom_duration_sec > thresholds.lost_timeout:
        return LIOHealth.LOST

    if odom_age_sec is None:
        return LIOHealth.LOST

    degraded = (
        time_regression
        or odom_age_sec > thresholds.odom_age_error
        or (imu_age_sec is not None and imu_age_sec > thresholds.imu_age_warn)
    )
    return LIOHealth.DEGRADED if degraded else LIOHealth.HEALTHY


class LIOHealthMonitor(Node):
    """Monitor FAST-LIO odometry pipeline health.

    Subscribes to the LIO odometry topic and an IMU topic, evaluates input
    freshness, timestamp monotonicity, and covariance sanity, then publishes a
    ``diagnostic_msgs/DiagnosticArray`` at 10 Hz on ``/localization/lio/health``.

    Parameters
    ----------
    odom_age_warn : float
        Odometry age (s) that triggers a warning.  Default ``0.15``.
    odom_age_error : float
        Odometry age (s) that triggers an error / DEGRADED.  Default ``0.50``.
    imu_age_warn : float
        IMU age (s) that triggers a warning / DEGRADED.  Default ``0.10``.
    lost_timeout : float
        Seconds without any odometry message before declaring LOST.
        Default ``1.0``.
    covariance_blowup : float
        Any diagonal pose-covariance element exceeding this value declares
        LOST.  Default ``1e6``.
    imu_topic : str
        Name of the IMU topic to subscribe to.  Default ``"/imu/data"``.
    """

    def __init__(self) -> None:
        super().__init__("lio_health_monitor")

        self.declare_parameter("odom_age_warn", 0.15)
        self.declare_parameter("odom_age_error", 0.50)
        self.declare_parameter("imu_age_warn", 0.10)
        self.declare_parameter("lost_timeout", 1.0)
        self.declare_parameter("covariance_blowup", 1e6)
        self.declare_parameter("imu_topic", "/imu/data")

        self._thresholds = HealthThresholds(
            odom_age_warn=self.get_parameter("odom_age_warn").value,
            odom_age_error=self.get_parameter("odom_age_error").value,
            imu_age_warn=self.get_parameter("imu_age_warn").value,
            lost_timeout=self.get_parameter("lost_timeout").value,
            covariance_blowup=self.get_parameter("covariance_blowup").value,
        )

        self._last_odom_time: Optional[Time] = None
        self._last_imu_time: Optional[Time] = None
        self._previous_odom_stamp: Optional[Time] = None
        self._time_regression: bool = False
        self._covariance_finite: bool = True
        self._covariance_exceeds: bool = False
        self._last_valid_odom_time: Optional[Time] = None

        self._odom_sub = self.create_subscription(
            Odometry,
            "/localization/lio/odom",
            self._odom_callback,
            10,
        )
        imu_topic: str = self.get_parameter("imu_topic").value
        self._imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self._imu_callback,
            10,
        )

        self._health_pub = self.create_publisher(
            DiagnosticArray,
            "/localization/lio/health",
            10,
        )

        self._timer = self.create_timer(0.1, self._evaluate_and_publish)

    # ------------------------------------------------------------------ #
    #  Callbacks                                                          #
    # ------------------------------------------------------------------ #

    def _odom_callback(self, msg: Odometry) -> None:
        stamp = Time.from_msg(msg.header.stamp)

        if self._previous_odom_stamp is not None:
            if stamp <= self._previous_odom_stamp:
                self._time_regression = True
            else:
                self._time_regression = False
        self._previous_odom_stamp = stamp

        self._last_odom_time = stamp
        self._last_valid_odom_time = stamp

        cov = tuple(msg.pose.covariance)
        self._covariance_finite = _finite_covariance(cov)
        self._covariance_exceeds = _covariance_blows_up(
            cov, self._thresholds.covariance_blowup,
        )

    def _imu_callback(self, msg: Imu) -> None:
        self._last_imu_time = Time.from_msg(msg.header.stamp)

    # ------------------------------------------------------------------ #
    #  Periodic evaluation                                                #
    # ------------------------------------------------------------------ #

    def _evaluate_and_publish(self) -> None:
        now = self.get_clock().now()

        odom_age_sec = (
            _duration_sec(now - self._last_odom_time)
            if self._last_odom_time is not None
            else None
        )
        imu_age_sec = (
            _duration_sec(now - self._last_imu_time)
            if self._last_imu_time is not None
            else None
        )
        no_odom_duration_sec = (
            _duration_sec(now - self._last_valid_odom_time)
            if self._last_valid_odom_time is not None
            else None
        )

        health = evaluate_health(
            odom_age_sec=odom_age_sec,
            imu_age_sec=imu_age_sec,
            time_regression=self._time_regression,
            covariance_finite=self._covariance_finite,
            covariance_exceeds=self._covariance_exceeds,
            no_odom_duration_sec=no_odom_duration_sec,
            thresholds=self._thresholds,
        )

        self._publish_diagnostics(health, odom_age_sec, imu_age_sec)

    def _publish_diagnostics(
        self,
        health: LIOHealth,
        odom_age_sec: Optional[float],
        imu_age_sec: Optional[float],
    ) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "lio_health"
        status.hardware_id = "fast_lio"

        if health == LIOHealth.HEALTHY:
            status.level = DiagnosticStatus.OK
            status.message = "LIO odometry healthy"
        elif health == LIOHealth.DEGRADED:
            status.level = DiagnosticStatus.WARN
            status.message = "LIO odometry degraded"
        elif health == LIOHealth.LOST:
            status.level = DiagnosticStatus.ERROR
            status.message = "LIO odometry lost"
        else:
            assert_never(health)

        status.values = [
            KeyValue(
                key="odom_age_sec",
                value=f"{odom_age_sec:.3f}" if odom_age_sec is not None else "N/A",
            ),
            KeyValue(
                key="imu_age_sec",
                value=f"{imu_age_sec:.3f}" if imu_age_sec is not None else "N/A",
            ),
            KeyValue(key="time_regression", value=str(self._time_regression)),
            KeyValue(key="covariance_finite", value=str(self._covariance_finite)),
            KeyValue(key="covariance_exceeds", value=str(self._covariance_exceeds)),
        ]

        msg.status = [status]
        self._health_pub.publish(msg)


# ------------------------------------------------------------------ #
#  Helpers                                                            #
# ------------------------------------------------------------------ #


def _duration_sec(duration: Duration) -> float:
    return duration.nanoseconds * 1e-9
