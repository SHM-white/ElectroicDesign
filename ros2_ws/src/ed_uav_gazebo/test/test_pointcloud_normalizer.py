from __future__ import annotations

from dataclasses import replace
import struct

import pytest

from ed_uav_gazebo.pointcloud_normalizer import (
    PointCloudNormalizationError,
    PointFieldDatatype,
    PointFieldSpec,
    SourcePointCloud,
    normalize_gazebo_pointcloud,
)


WIDTH = 360
HEIGHT = 1
SCAN_RATE_HZ = 10.0

FIELDS_XYZ = (
    PointFieldSpec("x", 0, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("y", 4, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("z", 8, PointFieldDatatype.FLOAT32, 1),
)
POINT_STEP_XYZ = 12

FIELDS_XYZI = (
    PointFieldSpec("x", 0, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("y", 4, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("z", 8, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("intensity", 12, PointFieldDatatype.FLOAT32, 1),
)
POINT_STEP_XYZI = 16

FIELDS_XYZIR = (
    PointFieldSpec("x", 0, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("y", 4, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("z", 8, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("intensity", 16, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("ring", 24, PointFieldDatatype.UINT16, 1),
)
POINT_STEP_XYZIR = 32

OUTPUT_RECORD = struct.Struct("<fff4xffH6x")


def _source(
    point_step: int,
    fields: tuple[PointFieldSpec, ...],
    is_bigendian: bool = False,
    width: int = WIDTH,
    height: int = HEIGHT,
) -> SourcePointCloud:
    byte_order = ">" if is_bigendian else "<"
    row_step = width * point_step
    data = bytearray(height * row_step)
    has_intensity = any(f.name == "intensity" for f in fields)
    has_ring = any(f.name == "ring" for f in fields)
    intensity_field = next((f for f in fields if f.name == "intensity"), None)
    ring_field = next((f for f in fields if f.name == "ring"), None)
    for row in range(height):
        for column in range(width):
            offset = row * row_step + column * point_step
            struct.pack_into(f"{byte_order}f", data, offset, float(column))
            struct.pack_into(f"{byte_order}f", data, offset + 4, float(row))
            struct.pack_into(f"{byte_order}f", data, offset + 8, float(column + row))
            if has_intensity and intensity_field is not None:
                struct.pack_into(f"{byte_order}f", data, offset + intensity_field.offset, float(row + 0.5))
            if has_ring and ring_field is not None:
                struct.pack_into(f"{byte_order}H", data, offset + ring_field.offset, row)
    return SourcePointCloud(
        width=width,
        height=height,
        point_step=point_step,
        row_step=row_step,
        is_bigendian=is_bigendian,
        fields=fields,
        data=bytes(data),
    )


@pytest.mark.parametrize(
    "point_step,fields",
    [
        (POINT_STEP_XYZ, FIELDS_XYZ),
        (POINT_STEP_XYZI, FIELDS_XYZI),
        (POINT_STEP_XYZIR, FIELDS_XYZIR),
    ],
    ids=["xyz_only", "xyzi", "xyzir"],
)
@pytest.mark.parametrize("is_bigendian", (False, True))
def test_normalize_gazebo_pointcloud_valid(
    point_step: int,
    fields: tuple[PointFieldSpec, ...],
    is_bigendian: bool,
) -> None:
    source = _source(point_step, fields, is_bigendian)
    result = normalize_gazebo_pointcloud(source, SCAN_RATE_HZ)

    time_step = struct.unpack("<f", struct.pack("<f", 1.0 / (WIDTH * SCAN_RATE_HZ)))[0]
    records = tuple(OUTPUT_RECORD.unpack_from(result.data, index * 32) for index in range(result.width))

    assert result.height == 1
    assert result.width == WIDTH * HEIGHT
    assert result.point_step == 32
    assert result.is_bigendian is False
    assert result.is_dense is True
    assert len(records) == WIDTH

    has_intensity = any(f.name == "intensity" for f in fields)
    for record in records:
        assert math.isfinite(record[0])
        assert math.isfinite(record[1])
        assert math.isfinite(record[2])
        if has_intensity:
            assert record[3] == pytest.approx(0.5)
        else:
            assert record[3] == pytest.approx(1.0)
        assert record[5] == 0

    assert [record[4] for record in records[:7]] == pytest.approx(
        [index * time_step for index in range(7)]
    )
    assert all(left[4] <= right[4] for left, right in zip(records, records[1:]))


import math


def test_normalize_gazebo_pointcloud_with_nan_neighbors() -> None:
    source = _source(POINT_STEP_XYZ, FIELDS_XYZ)
    data = bytearray(source.data)
    struct.pack_into("<f", data, 0, float("nan"))
    struct.pack_into("<f", data, 180 * POINT_STEP_XYZ + 8, float("inf"))

    result = normalize_gazebo_pointcloud(replace(source, data=bytes(data)), SCAN_RATE_HZ)

    assert result.width == WIDTH * HEIGHT - 2
    assert result.is_dense is True


def test_normalize_gazebo_pointcloud_rejects_missing_xyz_fields() -> None:
    bad_fields = (
        PointFieldSpec("x", 0, PointFieldDatatype.FLOAT32, 1),
        PointFieldSpec("y", 4, PointFieldDatatype.FLOAT32, 1),
    )
    source = _source(8, bad_fields)

    with pytest.raises(PointCloudNormalizationError):
        normalize_gazebo_pointcloud(source, SCAN_RATE_HZ)


def test_normalize_gazebo_pointcloud_rejects_truncated_buffer() -> None:
    source = _source(POINT_STEP_XYZ, FIELDS_XYZ)
    truncated = replace(source, data=source.data[:-1])

    with pytest.raises(PointCloudNormalizationError):
        normalize_gazebo_pointcloud(truncated, SCAN_RATE_HZ)


def test_normalize_gazebo_pointcloud_rejects_all_nan_points() -> None:
    source = _source(POINT_STEP_XYZ, FIELDS_XYZ)
    data = bytearray(source.data)
    for column in range(WIDTH):
        struct.pack_into("<f", data, column * POINT_STEP_XYZ, float("nan"))

    with pytest.raises(PointCloudNormalizationError):
        normalize_gazebo_pointcloud(replace(source, data=bytes(data)), SCAN_RATE_HZ)


def test_normalize_gazebo_pointcloud_rejects_only_zero_time_points() -> None:
    source = _source(POINT_STEP_XYZ, FIELDS_XYZ)
    data = bytearray(source.data)
    for column in range(1, WIDTH):
        struct.pack_into("<f", data, column * POINT_STEP_XYZ, float("nan"))

    with pytest.raises(PointCloudNormalizationError):
        normalize_gazebo_pointcloud(replace(source, data=bytes(data)), SCAN_RATE_HZ)


def test_normalize_gazebo_pointcloud_synthesizes_intensity_and_ring() -> None:
    source = _source(POINT_STEP_XYZ, FIELDS_XYZ)
    result = normalize_gazebo_pointcloud(source, SCAN_RATE_HZ)

    records = tuple(OUTPUT_RECORD.unpack_from(result.data, i * 32) for i in range(result.width))
    for record in records:
        assert record[3] == pytest.approx(1.0)
        assert record[5] == 0
