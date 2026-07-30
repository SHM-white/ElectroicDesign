"""Rendered-image tests for prescribed target geometry extraction."""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from target_fixture import render_target  # noqa: E402


def test_detects_prescribed_circle_cross_when_complete() -> None:
    # Given
    rendered = render_target()
    from ed_uav_perception.target_detector import DetectionFailure, detect_target

    # When
    result = detect_target(rendered.image, "d2026-circle-cross-v1")

    # Then
    assert not isinstance(result, DetectionFailure)
    # Feature points: 1 center + 3-4 cross-circle intersections
    assert result.object_points.shape[0] >= 4  # Minimum: center + 3 cross-circle points
    assert result.object_points.shape[0] <= 5  # Maximum: center + 4 cross-circle points
    assert result.image_points.shape == (result.object_points.shape[0], 2)
    assert result.symmetry_order == 4


def test_detects_target_even_without_inner_ring() -> None:
    # Given: marker with only outer ring (inner ring missing).
    rendered = render_target(include_inner=False)
    from ed_uav_perception.target_detector import DetectionFailure, detect_target

    # When
    result = detect_target(rendered.image, "d2026-circle-cross-v1")

    # Then: cross lines still create edges at the inner radius distance,
    # so cross-circle intersections can still be detected
    assert not isinstance(result, DetectionFailure)
    assert result.object_points.shape[0] >= 4


def test_detects_target_with_different_inner_diameter() -> None:
    # Given: marker with inner diameter 0.22m (different from default 0.30m).
    rendered = render_target(inner_diameter_m=0.22)
    from ed_uav_perception.target_detector import DetectionFailure, detect_target

    # When
    result = detect_target(rendered.image, "d2026-circle-cross-v1")

    # Then: different inner diameter still has cross-circle intersections
    assert not isinstance(result, DetectionFailure)
    assert result.object_points.shape[0] >= 4


def test_handles_target_with_only_one_cross_axis() -> None:
    # Given: marker with only horizontal cross line.
    rendered = render_target(cross_axes=("x",))
    from ed_uav_perception.target_detector import detect_target

    # When: detection runs without crash on incomplete cross.
    result = detect_target(rendered.image, "d2026-circle-cross-v1")

    # Then: result is a valid detection outcome (accepted or rejected).
    assert result is not None


def test_detects_target_with_thicker_cross_lines() -> None:
    # Given
    rendered = render_target(line_width_m=0.03)
    from ed_uav_perception.target_detector import DetectionFailure, detect_target

    # When
    result = detect_target(rendered.image, "d2026-circle-cross-v1")

    # Then: thicker cross lines still have detectable intersections
    assert not isinstance(result, DetectionFailure)
    assert result.object_points.shape[0] >= 4
