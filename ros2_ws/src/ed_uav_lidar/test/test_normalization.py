"""Normalization contract tests for the lidar transport boundary."""

from __future__ import annotations

import sys
from dataclasses import fields
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_lidar.contracts import MissingPointTiming, PointTimeRegression, normalize_mid360

from fake_messages import Header, LivoxCustomMsg, LivoxPoint


def mid360_packet(offset_times: tuple[int, ...]) -> LivoxCustomMsg:
    points = tuple(
        LivoxPoint(offset_time, 1.0, 2.0, 3.0, 17, 0, 0)
        for offset_time in offset_times
    )
    return LivoxCustomMsg(
        header=Header(stamp_ns=4_000_000_000, frame_id="lidar_link"),
        timebase=4_000_000_000,
        point_num=len(points),
        lidar_id=1,
        rsvd=(0, 0, 0),
        points=points,
    )


def test_exact_livox_fake_surface_preserves_upstream_fields() -> None:
    # Given: a fake packet matching the Livox CustomMsg and CustomPoint field surface.
    packet_fields = tuple(field.name for field in fields(LivoxCustomMsg))
    point_fields = tuple(field.name for field in fields(LivoxPoint))

    # When: the contract exposes its fake boundary.
    actual_packet_fields = packet_fields
    actual_point_fields = point_fields

    # Then: the raw timestamp-bearing fields cannot be omitted by the fake.
    assert actual_packet_fields == ("header", "timebase", "point_num", "lidar_id", "rsvd", "points")
    assert actual_point_fields == ("offset_time", "x", "y", "z", "reflectivity", "tag", "line")


def test_rejects_mid360_packet_when_per_point_timing_is_missing() -> None:
    # Given: a Mid-360 packet with no points carrying raw offset timestamps.
    packet = mid360_packet(())

    # When: the transport normalizes it for monitored PointCloud2 output.
    with pytest.raises(MissingPointTiming, match="per-point offset_time"):
        normalize_mid360(packet)

    # Then: activation cannot claim LIO eligibility without timing.


def test_rejects_mid360_packet_when_point_time_regresses() -> None:
    # Given: raw Livox offsets that regress within one packet.
    packet = mid360_packet((10, 9))

    # When: the transport normalizes it.
    with pytest.raises(PointTimeRegression, match="offset_time regression"):
        normalize_mid360(packet)

    # Then: the direct path is not silently normalized into a misleading cloud.


def test_normalizes_mid360_monitoring_cloud_without_mutating_direct_packet() -> None:
    # Given: a timestamped Mid-360 CustomMsg packet.
    packet = mid360_packet((10, 20))

    # When: the monitoring adapter creates a standard cloud.
    cloud = normalize_mid360(packet)

    # Then: the direct packet is retained and monitor timing is explicit.
    assert cloud.direct_custom is packet
    assert cloud.fields == ("x", "y", "z", "intensity", "offset_time")
    assert cloud.point_times_ns == (10, 20)
