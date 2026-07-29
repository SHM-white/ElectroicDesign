"""Lossless Mid-360 normalization contracts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Sequence


MONITORING_FIELDS = ("x", "y", "z", "intensity", "offset_time")


class LivoxPoint(Protocol):
    """Fields exposed by the pinned livox_ros_driver2 CustomPoint message."""

    offset_time: int
    x: float
    y: float
    z: float
    reflectivity: int
    tag: int
    line: int


class LivoxCustomMsg(Protocol):
    """Fields exposed by the pinned livox_ros_driver2 CustomMsg message."""

    timebase: int
    point_num: int
    lidar_id: int
    rsvd: Sequence[int]
    points: Sequence[LivoxPoint]


@dataclass(frozen=True, slots=True)
class MissingPointTiming(Exception):
    point_count: int

    def __str__(self) -> str:
        return f"per-point offset_time is required; packet has {self.point_count} points"


@dataclass(frozen=True, slots=True)
class PointTimeRegression(Exception):
    previous_offset_ns: int
    current_offset_ns: int

    def __str__(self) -> str:
        return (
            "offset_time regression: "
            f"{self.current_offset_ns} follows {self.previous_offset_ns}"
        )


@dataclass(frozen=True, slots=True)
class PacketShapeError(Exception):
    declared_point_count: int
    actual_point_count: int

    def __str__(self) -> str:
        return (
            "CustomMsg point_num mismatch: "
            f"declared {self.declared_point_count}, actual {self.actual_point_count}"
        )


@dataclass(frozen=True, slots=True)
class NormalizedPointCloud:
    """Monitored PointCloud2 metadata retaining raw CustomMsg timing provenance."""

    direct_custom: LivoxCustomMsg
    fields: tuple[str, ...]
    point_times_ns: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class GenericPointCloudAssessment:
    fields: tuple[str, ...]
    lio_eligible: bool


@dataclass(frozen=True, slots=True)
class GenericCloudFieldError(Exception):
    missing_fields: str

    def __str__(self) -> str:
        return f"PointCloud2 requires spatial fields: {self.missing_fields}"


def validate_offset_times(offset_times_ns: Sequence[int]) -> tuple[int, ...]:
    """Return ordered raw offsets or raise a deterministic timing contract error."""
    offsets = tuple(offset_times_ns)
    if not offsets:
        raise MissingPointTiming(point_count=0)
    previous = offsets[0]
    for current in offsets[1:]:
        if current < previous:
            raise PointTimeRegression(previous_offset_ns=previous, current_offset_ns=current)
        previous = current
    return offsets


def normalize_mid360(packet: LivoxCustomMsg) -> NormalizedPointCloud:
    """Build monitoring metadata without changing the direct FAST-LIO CustomMsg."""
    if packet.point_num != len(packet.points):
        raise PacketShapeError(
            declared_point_count=packet.point_num,
            actual_point_count=len(packet.points),
        )
    offset_times_ns = validate_offset_times(tuple(point.offset_time for point in packet.points))
    return NormalizedPointCloud(
        direct_custom=packet,
        fields=MONITORING_FIELDS,
        point_times_ns=offset_times_ns,
    )


def normalize_mid360_raw(packet: LivoxCustomMsg) -> NormalizedPointCloud:
    """Build monitoring metadata skipping offset_time monotonicity validation."""
    if packet.point_num != len(packet.points):
        raise PacketShapeError(
            declared_point_count=packet.point_num,
            actual_point_count=len(packet.points),
        )
    return NormalizedPointCloud(
        direct_custom=packet,
        fields=MONITORING_FIELDS,
        point_times_ns=tuple(point.offset_time for point in packet.points),
    )


def assess_generic_point_cloud(fields: Sequence[str]) -> GenericPointCloudAssessment:
    """Classify a generic PointCloud2 as monitoring-only unless timing is explicit."""
    normalized_fields = tuple(fields)
    required_spatial = {"x", "y", "z"}
    missing_spatial = required_spatial.difference(normalized_fields)
    if missing_spatial:
        missing_text = ", ".join(sorted(missing_spatial))
        raise GenericCloudFieldError(missing_fields=missing_text)
    return GenericPointCloudAssessment(
        fields=normalized_fields,
        lio_eligible="time" in normalized_fields or "offset_time" in normalized_fields,
    )

