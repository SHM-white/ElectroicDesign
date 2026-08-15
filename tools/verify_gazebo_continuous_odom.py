#!/usr/bin/env python3
"""Bounded ROS probe for the Gazebo planar-LiDAR/FAST-LIO odometry chain."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import time

import rclpy
from ed_uav_interfaces.msg import LocalizationStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool


def _stamp_seconds(message: Odometry) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


@dataclass(slots=True)
class OdomSeries:
    count: int = 0
    stamps: list[float] = field(default_factory=list)
    positions: list[tuple[float, float, float]] = field(default_factory=list)
    wall_times: list[float] = field(default_factory=list)
    non_finite_count: int = 0
    stamp_regressions: int = 0

    def add(self, message: Odometry) -> None:
        stamp = _stamp_seconds(message)
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self.non_finite_count += 1
        if self.stamps and stamp < self.stamps[-1]:
            self.stamp_regressions += 1
        self.count += 1
        self.stamps.append(stamp)
        self.positions.append((position.x, position.y, position.z))
        self.wall_times.append(time.monotonic())

    def summary(self) -> dict[str, float | int]:
        unique_stamps = len(set(self.stamps))
        planar_displacement = 0.0
        if len(self.positions) >= 2:
            first = self.positions[0]
            last = self.positions[-1]
            planar_displacement = math.hypot(last[0] - first[0], last[1] - first[1])
        maximum_wall_gap = max(
            (right - left for left, right in zip(self.wall_times, self.wall_times[1:])),
            default=0.0,
        )
        return {
            "count": self.count,
            "unique_stamps": unique_stamps,
            "stamp_regressions": self.stamp_regressions,
            "non_finite_count": self.non_finite_count,
            "planar_displacement_m": planar_displacement,
            "maximum_wall_gap_s": maximum_wall_gap,
        }


class ContinuousOdomProbe(Node):
    """Observe every stage and apply a short, deterministic excitation."""

    ODOM_TOPICS = {
        "ground_truth": "/simulation/ground_truth/odom",
        "car": "/simulation/car/odom",
        "fast_lio": "/fast_lio/odometry",
        "planar_raw": "/localization/lio/planar_raw",
        "planar_fused": "/localization/lio/odom",
        "continuous": "/localization/odom",
    }

    def __init__(self) -> None:
        super().__init__("gazebo_continuous_odom_probe")
        self.sensor_counts = {"raw_cloud": 0, "normalized_cloud": 0, "imu": 0}
        self.odom = {name: OdomSeries() for name in self.ODOM_TOPICS}
        self.active_lio_status_count = 0
        self.last_status_reason = "no status"
        self._enable = self.create_publisher(Bool, "/simulation/enable", 10)
        self._velocity = self.create_publisher(Twist, "/simulation/cmd_vel", 10)
        self.create_subscription(
            PointCloud2,
            "/lidar/points_raw",
            lambda _message: self._count_sensor("raw_cloud"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/lidar/points",
            lambda _message: self._count_sensor("normalized_cloud"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/lidar/imu",
            lambda _message: self._count_sensor("imu"),
            qos_profile_sensor_data,
        )
        for name, topic in self.ODOM_TOPICS.items():
            self.create_subscription(
                Odometry,
                topic,
                lambda message, series=name: self.odom[series].add(message),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            LocalizationStatus,
            "/localization/status",
            self._on_status,
            10,
        )

    def _count_sensor(self, name: str) -> None:
        self.sensor_counts[name] += 1

    def _on_status(self, message: LocalizationStatus) -> None:
        self.last_status_reason = message.reason
        if (
            message.source == LocalizationStatus.SOURCE_LIO
            and message.state == LocalizationStatus.STATE_ACTIVE
        ):
            self.active_lio_status_count += 1

    def command(self, *, enabled: bool, forward_m_s: float, climb_m_s: float) -> None:
        enable = Bool()
        enable.data = enabled
        velocity = Twist()
        velocity.linear.x = forward_m_s
        velocity.linear.z = climb_m_s
        self._enable.publish(enable)
        self._velocity.publish(velocity)


def _vertical_rate_violations(series: OdomSeries, maximum_rate_m_s: float = 3.0) -> int:
    violations = 0
    for index in range(1, len(series.stamps)):
        elapsed = series.stamps[index] - series.stamps[index - 1]
        if elapsed <= 0.0:
            continue
        delta_z = abs(series.positions[index][2] - series.positions[index - 1][2])
        if delta_z > maximum_rate_m_s * elapsed + 0.03:
            violations += 1
    return violations


def _evaluate(probe: ContinuousOdomProbe) -> tuple[dict[str, object], list[str]]:
    summary: dict[str, object] = {
        "sensors": dict(probe.sensor_counts),
        "odometry": {name: series.summary() for name, series in probe.odom.items()},
        "active_lio_status_count": probe.active_lio_status_count,
        "last_status_reason": probe.last_status_reason,
    }
    failures: list[str] = []
    minimum_sensor_counts = {"raw_cloud": 30, "normalized_cloud": 30, "imu": 300}
    for name, minimum in minimum_sensor_counts.items():
        if probe.sensor_counts[name] < minimum:
            failures.append(f"{name} count {probe.sensor_counts[name]} < {minimum}")
    for name in ("ground_truth", "car"):
        if probe.odom[name].count < 30:
            failures.append(f"{name} odometry did not stream")
    for name in ("fast_lio", "planar_raw", "planar_fused", "continuous"):
        series = probe.odom[name]
        if series.count < 20:
            failures.append(f"{name} count {series.count} < 20")
        if len(set(series.stamps)) < 10:
            failures.append(f"{name} has fewer than 10 distinct timestamps")
        if series.stamp_regressions:
            failures.append(f"{name} timestamp regressed {series.stamp_regressions} times")
        if series.non_finite_count:
            failures.append(f"{name} published {series.non_finite_count} non-finite poses")
        if len(series.wall_times) >= 2 and series.summary()["maximum_wall_gap_s"] > 2.5:
            failures.append(f"{name} stopped publishing for more than 2.5 s")
    fused = probe.odom["planar_fused"]
    vertical_violations = _vertical_rate_violations(fused)
    summary["planar_fused_vertical_rate_violations"] = vertical_violations
    if vertical_violations:
        failures.append(f"planar_fused Z exceeded its rate bound {vertical_violations} times")
    if probe.active_lio_status_count < 1:
        failures.append("source supervisor never selected active LIO")
    if probe.odom["ground_truth"].summary()["planar_displacement_m"] < 0.10:
        failures.append("Gazebo vehicle did not execute the excitation")
    if probe.odom["fast_lio"].summary()["planar_displacement_m"] < 0.03:
        failures.append("FAST-LIO did not observe planar motion")
    if fused.positions and probe.odom["ground_truth"].positions:
        z_error = abs(fused.positions[-1][2] - probe.odom["ground_truth"].positions[-1][2])
        summary["final_altitude_error_m"] = z_error
        if z_error > 0.20:
            failures.append(f"fused altitude differs from simulator by {z_error:.3f} m")
    return summary, failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--imu-settle", type=float, default=23.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    rclpy.init()
    probe = ContinuousOdomProbe()
    started = time.monotonic()
    sensors_ready_at: float | None = None
    motion_finished_at: float | None = None
    try:
        while rclpy.ok() and time.monotonic() - started < args.timeout:
            rclpy.spin_once(probe, timeout_sec=0.05)
            now = time.monotonic()
            if sensors_ready_at is None and (
                probe.sensor_counts["raw_cloud"] >= 10
                and probe.sensor_counts["normalized_cloud"] >= 10
                and probe.sensor_counts["imu"] >= 100
            ):
                sensors_ready_at = now
            if sensors_ready_at is None or now - sensors_ready_at < args.imu_settle:
                continue
            phase = now - sensors_ready_at - args.imu_settle
            if phase < 3.0:
                probe.command(enabled=True, forward_m_s=0.0, climb_m_s=0.20)
            elif phase < 7.0:
                probe.command(enabled=True, forward_m_s=0.15, climb_m_s=0.0)
            else:
                probe.command(enabled=True, forward_m_s=0.0, climb_m_s=0.0)
                motion_finished_at = motion_finished_at or now
            if motion_finished_at is not None and now - motion_finished_at >= 8.0:
                summary, failures = _evaluate(probe)
                if not failures:
                    break
        probe.command(enabled=True, forward_m_s=0.0, climb_m_s=0.0)
        for _ in range(5):
            rclpy.spin_once(probe, timeout_sec=0.05)
        summary, failures = _evaluate(probe)
        summary["elapsed_wall_s"] = time.monotonic() - started
        summary["failures"] = failures
        rendered = json.dumps(summary, indent=2, sort_keys=True)
        print(rendered)
        if args.output is not None:
            args.output.write_text(rendered + "\n", encoding="utf-8")
        return 1 if failures else 0
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
