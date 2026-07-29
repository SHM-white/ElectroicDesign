"""Observation-level target quality and freshness tests."""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from target_fixture import render_target  # noqa: E402


def _request(
    *,
    now_sec: float = 20.10,
    calibrated: bool = True,
    revision: str = "d2026-circle-cross-v1",
):
    from ed_uav_perception.target_types import (
        CameraModel,
        FrameContext,
        MotionContext,
        ObservationRequest,
        PoseLimits,
    )

    rendered = render_target()
    return ObservationRequest(
        rendered.image,
        CameraModel(
            rendered.camera_matrix,
            rendered.distortion,
            640,
            480,
            "camera_optical",
            calibrated,
        ),
        FrameContext(20.0, now_sec, 17, revision),
        MotionContext(20.02, 0, 0.18, 0.6, None),
        PoseLimits(),
    )


def test_rendered_frame_returns_quality_observation() -> None:
    # Given
    request = _request()
    from ed_uav_perception.target_pipeline import observe_target
    from ed_uav_perception.target_types import AcceptedObservation

    # When
    result = observe_target(request)

    # Then
    assert isinstance(result, AcceptedObservation)
    assert result.frame_id == "camera_optical"
    assert result.source_sequence == 17
    assert result.candidate_count >= 1
    assert result.quality > 0.5


def test_rejects_stale_image_over_020_seconds() -> None:
    # Given
    request = _request(now_sec=20.201)
    from ed_uav_perception.target_pipeline import observe_target
    from ed_uav_perception.target_types import RejectedObservation

    # When
    result = observe_target(request)

    # Then
    assert isinstance(result, RejectedObservation)
    assert result.reject_reason.value == "stale_image"


def test_rejects_future_dated_image() -> None:
    # Given
    request = _request(now_sec=19.99)
    from ed_uav_perception.target_pipeline import observe_target
    from ed_uav_perception.target_types import RejectedObservation

    # When
    result = observe_target(request)

    # Then
    assert isinstance(result, RejectedObservation)
    assert result.reject_reason.value == "future_image"


def test_rejects_uncalibrated_camera_info() -> None:
    # Given
    request = _request(calibrated=False)
    from ed_uav_perception.target_pipeline import observe_target
    from ed_uav_perception.target_types import RejectedObservation

    # When
    result = observe_target(request)

    # Then
    assert isinstance(result, RejectedObservation)
    assert result.reject_reason.value == "uncalibrated"


def test_rejects_unsupported_target_revision() -> None:
    # Given
    request = _request(revision="circle-cross-v0")
    from ed_uav_perception.target_pipeline import observe_target
    from ed_uav_perception.target_types import RejectedObservation

    # When
    result = observe_target(request)

    # Then
    assert isinstance(result, RejectedObservation)
    assert result.reject_reason.value == "wrong_revision"


def test_rejects_stale_vehicle_context() -> None:
    # Given
    from dataclasses import replace

    request = _request()
    request = replace(request, motion=replace(request.motion, stamp_sec=19.89))
    from ed_uav_perception.target_pipeline import observe_target
    from ed_uav_perception.target_types import RejectedObservation

    # When
    result = observe_target(request)

    # Then
    assert isinstance(result, RejectedObservation)
    assert result.reject_reason.value == "stale_vehicle"


def test_rejects_future_dated_vehicle_context() -> None:
    # Given
    from dataclasses import replace

    request = _request()
    request = replace(request, motion=replace(request.motion, stamp_sec=20.11))
    from ed_uav_perception.target_pipeline import observe_target
    from ed_uav_perception.target_types import RejectedObservation

    # When
    result = observe_target(request)

    # Then
    assert isinstance(result, RejectedObservation)
    assert result.reject_reason.value == "future_vehicle"


def test_rejects_invalid_vehicle_turn_class() -> None:
    # Given
    from dataclasses import replace

    request = _request()
    request = replace(request, motion=replace(request.motion, turn_class=255))
    from ed_uav_perception.target_pipeline import observe_target
    from ed_uav_perception.target_types import RejectedObservation

    # When
    result = observe_target(request)

    # Then
    assert isinstance(result, RejectedObservation)
    assert result.reject_reason.value == "invalid_vehicle_context"


def test_rejects_malformed_image_raster() -> None:
    # Given
    from dataclasses import replace

    request = _request()
    request = replace(request, image=request.image[:100, :100])
    from ed_uav_perception.target_pipeline import observe_target
    from ed_uav_perception.target_types import RejectedObservation

    # When
    result = observe_target(request)

    # Then
    assert isinstance(result, RejectedObservation)
    assert result.reject_reason.value == "raster_mismatch"


def test_geometry_result_is_deterministic_across_repeated_runs() -> None:
    # Given
    request = _request()
    from ed_uav_perception.target_pipeline import observe_target
    from ed_uav_perception.target_types import AcceptedObservation

    # When
    results = [observe_target(request) for _ in range(25)]

    # Then
    assert all(isinstance(result, AcceptedObservation) for result in results)
    accepted = [result for result in results if isinstance(result, AcceptedObservation)]
    assert len({result.reprojection_rms_px for result in accepted}) == 1
    assert len({tuple(result.pose.translation_m) for result in accepted}) == 1
