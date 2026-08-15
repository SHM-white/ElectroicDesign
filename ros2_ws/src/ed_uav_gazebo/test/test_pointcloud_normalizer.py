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
POINT_STEP = 32
ROW_PADDING = 8
ROW_STEP = WIDTH * POINT_STEP + ROW_PADDING
SCAN_RATE_HZ = 10.0
SOURCE_FIELDS = (
    PointFieldSpec("x", 0, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("y", 4, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("z", 8, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("intensity", 16, PointFieldDatatype.FLOAT32, 1),
    PointFieldSpec("ring", 24, PointFieldDatatype.UINT16, 1),
)
OUTPUT_RECORD = struct.Struct("<fff4xffH6x")


def _source(is_bigendian: bool) -> SourcePointCloud:
    byte_order = ">" if is_bigendian else "<"
    data = bytearray(HEIGHT * ROW_STEP)
    for row in range(HEIGHT):
        for column in range(WIDTH):
            offset = row * ROW_STEP + column * POINT_STEP
            struct.pack_into(f"{byte_order}f", data, offset, float(column))
            struct.pack_into(f"{byte_order}f", data, offset + 4, float(row))
            struct.pack_into(f"{byte_order}f", data, offset + 8, float(column + row))
            struct.pack_into(f"{byte_order}f", data, offset + 16, float(row + 0.5))
            struct.pack_into(f"{byte_order}H", data, offset + 24, row)
    return SourcePointCloud(
        width=WIDTH,
        height=HEIGHT,
        point_step=POINT_STEP,
        row_step=ROW_STEP,
        is_bigendian=is_bigendian,
        fields=SOURCE_FIELDS,
        data=bytes(data),
    )


@pytest.mark.parametrize("is_bigendian", (False, True))
def test_normalize_gazebo_pointcloud_when_valid_points_have_nonfinite_neighbors(
    is_bigendian: bool,
) -> None:
    # Given: a padded single-ring planar Gazebo cloud with one NaN and one infinity.
    source = _source(is_bigendian)
    byte_order = ">" if is_bigendian else "<"
    data = bytearray(source.data)
    struct.pack_into(f"{byte_order}f", data, 0, float("nan"))
    struct.pack_into(f"{byte_order}f", data, 180 * POINT_STEP + 16, float("inf"))

    # When: the cloud is normalized for FAST-LIO type 2.
    result = normalize_gazebo_pointcloud(replace(source, data=bytes(data)), SCAN_RATE_HZ)

    # Then: records are dense, canonical, column-major, finite, and timed exactly.
    records = tuple(OUTPUT_RECORD.unpack_from(result.data, index * POINT_STEP) for index in range(result.width))
    time_step = struct.unpack("<f", struct.pack("<f", 1.0 / (WIDTH * SCAN_RATE_HZ)))[0]
    assert result.height == 1
    assert result.width == WIDTH * HEIGHT - 2
    assert result.point_step == POINT_STEP
    assert result.row_step == result.width * POINT_STEP
    assert result.is_bigendian is False
    assert result.is_dense is True
    assert [(field.name, field.offset, field.datatype, field.count) for field in result.fields] == [
        ("x", 0, PointFieldDatatype.FLOAT32, 1),
        ("y", 4, PointFieldDatatype.FLOAT32, 1),
        ("z", 8, PointFieldDatatype.FLOAT32, 1),
        ("intensity", 16, PointFieldDatatype.FLOAT32, 1),
        ("time", 20, PointFieldDatatype.FLOAT32, 1),
        ("ring", 24, PointFieldDatatype.UINT16, 1),
    ]
    assert [(record[0], record[1], record[5]) for record in records[:7]] == [
        (1.0, 0.0, 0),
        (2.0, 0.0, 0),
        (3.0, 0.0, 0),
        (4.0, 0.0, 0),
        (5.0, 0.0, 0),
        (6.0, 0.0, 0),
        (7.0, 0.0, 0),
    ]
    assert [record[4] for record in records[:7]] == pytest.approx(
        [index * time_step for index in range(1, 8)]
    )
    assert records[-1][4] == struct.unpack("<f", struct.pack("<f", 359.0 / 3600.0))[0]
    assert all(left[4] <= right[4] for left, right in zip(records, records[1:]))


def test_normalize_gazebo_pointcloud_when_schema_is_not_the_observed_gazebo_schema() -> None:
    # Given: a cloud whose width differs from the fixed Gazebo source schema.
    source = replace(_source(False), width=WIDTH - 1)

    # When: the normalizer parses the source cloud.
    with pytest.raises(PointCloudNormalizationError):
        normalize_gazebo_pointcloud(source, SCAN_RATE_HZ)

    # Then: no fallback schema is accepted.


def test_normalize_gazebo_pointcloud_when_buffer_does_not_cover_padded_rows() -> None:
    # Given: a source cloud truncated before its declared padded row span.
    source = _source(False)
    truncated = replace(source, data=source.data[:-1])

    # When: the normalizer parses the truncated cloud.
    with pytest.raises(PointCloudNormalizationError):
        normalize_gazebo_pointcloud(truncated, SCAN_RATE_HZ)

    # Then: it rejects the unsafe buffer.


def test_normalize_gazebo_pointcloud_when_all_points_are_unusable() -> None:
    # Given: a source cloud whose every x coordinate is NaN.
    source = _source(False)
    data = bytearray(source.data)
    for row in range(HEIGHT):
        for column in range(WIDTH):
            struct.pack_into("<f", data, row * ROW_STEP + column * POINT_STEP, float("nan"))

    # When: the normalizer filters the source values.
    with pytest.raises(PointCloudNormalizationError):
        normalize_gazebo_pointcloud(replace(source, data=bytes(data)), SCAN_RATE_HZ)

    # Then: it refuses an empty output cloud.


def test_normalize_gazebo_pointcloud_when_only_zero_time_points_survive() -> None:
    # Given: only the source's first horizontal column remains finite.
    source = _source(False)
    data = bytearray(source.data)
    for row in range(HEIGHT):
        for column in range(1, WIDTH):
            struct.pack_into("<f", data, row * ROW_STEP + column * POINT_STEP, float("nan"))

    # When: the normalizer creates per-column times.
    with pytest.raises(PointCloudNormalizationError):
        normalize_gazebo_pointcloud(replace(source, data=bytes(data)), SCAN_RATE_HZ)

    # Then: it rejects clouds without a positive point time.
