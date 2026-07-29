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
    assert result.object_points.shape[0] >= 8
    assert result.image_points.shape == (result.object_points.shape[0], 2)
    assert result.symmetry_order == 4


def test_rejects_partial_target_when_inner_ring_missing() -> None:
    # Given
    rendered = render_target(include_inner=False)
    from ed_uav_perception.target_detector import DetectionFailure, detect_target

    # When
    result = detect_target(rendered.image, "d2026-circle-cross-v1")

    # Then
    assert isinstance(result, DetectionFailure)
    assert result.reason.value == "partial_geometry"


def test_rejects_wrong_ring_revision() -> None:
    # Given
    rendered = render_target(inner_diameter_m=0.22)
    from ed_uav_perception.target_detector import DetectionFailure, detect_target

    # When
    result = detect_target(rendered.image, "d2026-circle-cross-v1")

    # Then
    assert isinstance(result, DetectionFailure)
    assert result.reason.value == "wrong_revision"
