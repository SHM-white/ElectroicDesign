"""Gazebo PointCloud2 to FAST-LIO type-2 normalization.

Accepts variable Gazebo GPU lidar schemas:
  - x/y/z only (point_step=12)
  - x/y/z/intensity (point_step=16)
  - x/y/z/intensity/ring (point_step=32, legacy)
Synthesizes missing intensity (1.0) and ring (0) fields.
"""

from dataclasses import dataclass
from enum import Enum, IntEnum
import math
import struct
from typing import Final


class PointFieldDatatype(IntEnum):
    """PointCloud2 datatypes used by the fixed Gazebo schema."""

    UINT16 = 4
    FLOAT32 = 7


class NormalizationFailure(str, Enum):
    """Reasons the fixed simulation input contract can be rejected."""

    INVALID_SCHEMA = "invalid_schema"
    INVALID_BUFFER = "invalid_buffer"
    INVALID_SCAN_RATE = "invalid_scan_rate"
    NO_USABLE_POINTS = "no_usable_points"
    NO_POSITIVE_TIME = "no_positive_time"


@dataclass(frozen=True, slots=True)
class PointCloudNormalizationError(Exception):
    """Typed rejection returned to the ROS boundary without publishing."""

    failure: NormalizationFailure
    detail: str

    def __str__(self) -> str:
        return f"{self.failure.value}: {self.detail}"


@dataclass(frozen=True, slots=True)
class PointFieldSpec:
    """ROS-independent PointCloud2 field metadata."""

    name: str
    offset: int
    datatype: int
    count: int


@dataclass(frozen=True, slots=True)
class SourcePointCloud:
    """The exact Gazebo PointCloud2 payload passed into this boundary."""

    width: int
    height: int
    point_step: int
    row_step: int
    is_bigendian: bool
    fields: tuple[PointFieldSpec, ...]
    data: bytes


@dataclass(frozen=True, slots=True)
class NormalizedPointCloud:
    """Canonical dense PointCloud2 metadata and little-endian point records."""

    width: int
    height: int
    point_step: int
    row_step: int
    is_bigendian: bool
    is_dense: bool
    fields: tuple[PointFieldSpec, ...]
    data: bytes


OUTPUT_POINT_STEP: Final = 32
OUTPUT_FIELDS: Final = (
    PointFieldSpec("x", 0, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("y", 4, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("z", 8, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("intensity", 16, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("time", 20, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("ring", 24, PointFieldDatatype.UINT16, 1),
)
OUTPUT_RECORD: Final = struct.Struct("<fff4xffH6x")


def _field_by_name(fields: tuple[PointFieldSpec, ...], name: str) -> PointFieldSpec | None:
    for f in fields:
        if f.name == name:
            return f
    return None


def normalize_gazebo_pointcloud(
    source: SourcePointCloud,
    scan_rate_hz: float,
) -> NormalizedPointCloud:
    """Filter one Gazebo cloud into FAST-LIO type-2 point records."""
    _validate_source(source, scan_rate_hz)
    byte_order = ">" if source.is_bigendian else "<"
    x_field = _field_by_name(source.fields, "x")
    y_field = _field_by_name(source.fields, "y")
    z_field = _field_by_name(source.fields, "z")
    intensity_field = _field_by_name(source.fields, "intensity")
    ring_field = _field_by_name(source.fields, "ring")
    records: list[bytes] = []
    has_positive_time = False
    for column in range(source.width):
        point_time = column / (source.width * scan_rate_hz)
        for row in range(source.height):
            offset = row * source.row_step + column * source.point_step
            x = struct.unpack_from(f"{byte_order}f", source.data, offset + x_field.offset)[0]
            y = struct.unpack_from(f"{byte_order}f", source.data, offset + y_field.offset)[0]
            z = struct.unpack_from(f"{byte_order}f", source.data, offset + z_field.offset)[0]
            if intensity_field is not None:
                intensity = struct.unpack_from(f"{byte_order}f", source.data, offset + intensity_field.offset)[0]
            else:
                intensity = 1.0
            if ring_field is not None:
                ring = struct.unpack_from(f"{byte_order}H", source.data, offset + ring_field.offset)[0]
            else:
                ring = 0
            if all(math.isfinite(value) for value in (x, y, z, intensity)):
                records.append(OUTPUT_RECORD.pack(x, y, z, intensity, point_time, ring))
                has_positive_time = has_positive_time or point_time > 0.0
    if not records:
        raise PointCloudNormalizationError(
            NormalizationFailure.NO_USABLE_POINTS,
            "the source cloud contains no finite x/y/z point",
        )
    if not has_positive_time:
        raise PointCloudNormalizationError(
            NormalizationFailure.NO_POSITIVE_TIME,
            "the source cloud contains no finite point after horizontal column zero",
        )
    data = b"".join(records)
    return NormalizedPointCloud(
        width=len(records),
        height=1,
        point_step=OUTPUT_POINT_STEP,
        row_step=len(data),
        is_bigendian=False,
        is_dense=True,
        fields=OUTPUT_FIELDS,
        data=data,
    )


def _validate_source(source: SourcePointCloud, scan_rate_hz: float) -> None:
    """Reject invalid source layout, accepting variable Gazebo GPU lidar schemas."""
    if not math.isfinite(scan_rate_hz) or scan_rate_hz <= 0.0:
        raise PointCloudNormalizationError(
            NormalizationFailure.INVALID_SCAN_RATE,
            "scan_rate_hz must be finite and greater than zero",
        )
    if source.height < 1:
        raise PointCloudNormalizationError(
            NormalizationFailure.INVALID_SCHEMA,
            "source height must be at least 1",
        )
    if source.width < 1:
        raise PointCloudNormalizationError(
            NormalizationFailure.INVALID_SCHEMA,
            "source width must be at least 1",
        )
    x_field = _field_by_name(source.fields, "x")
    y_field = _field_by_name(source.fields, "y")
    z_field = _field_by_name(source.fields, "z")
    if x_field is None or y_field is None or z_field is None:
        raise PointCloudNormalizationError(
            NormalizationFailure.INVALID_SCHEMA,
            "source cloud must have x, y, z fields",
        )
    if x_field.datatype != PointFieldDatatype.FLOAT32:
        raise PointCloudNormalizationError(
            NormalizationFailure.INVALID_SCHEMA,
            "x field must be FLOAT32",
        )
    minimum_row_step = source.width * source.point_step
    if source.row_step < minimum_row_step:
        raise PointCloudNormalizationError(
            NormalizationFailure.INVALID_SCHEMA,
            "row_step is smaller than the declared point row",
        )
    required_data_size = source.height * source.row_step
    if len(source.data) < required_data_size:
        raise PointCloudNormalizationError(
            NormalizationFailure.INVALID_BUFFER,
            "data does not cover every declared padded row",
        )
