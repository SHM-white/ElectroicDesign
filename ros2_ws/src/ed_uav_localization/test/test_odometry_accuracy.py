from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_localization.odometry_accuracy import (
    OdometryEvaluationMode,
    OdometrySample,
    OdometryValidationError,
    OdometryValidationIssue,
    LoopEvaluation,
    StationaryEvaluation,
    StraightLineEvaluation,
    evaluate_odometry_accuracy,
)


def test_stationary_evaluation_reports_relative_drift_excursion_path_and_yaw() -> None:
    # Given: a stationary run with an excursion before the final drift.
    samples = (
        OdometrySample(stamp_ns=0, frame_id="odom", x_m=0.0, y_m=0.0, z_m=0.0, yaw_rad=0.0),
        OdometrySample(stamp_ns=1_000_000_000, frame_id="odom", x_m=3.0, y_m=4.0, z_m=0.0, yaw_rad=0.5),
        OdometrySample(
            stamp_ns=2_000_000_000,
            frame_id="odom",
            x_m=0.3,
            y_m=0.4,
            z_m=1.2,
            yaw_rad=math.pi / 2.0,
        ),
    )

    # When: the stationary metric is evaluated.
    evaluation = evaluate_odometry_accuracy(OdometryEvaluationMode.STATIONARY, samples)

    # Then: endpoint drift, maximum excursion, path, yaw, and drift rate are explicit.
    assert isinstance(evaluation, StationaryEvaluation)
    assert evaluation.end_xy_drift_m == pytest.approx(0.5)
    assert evaluation.end_3d_drift_m == pytest.approx(1.3)
    assert evaluation.max_xy_excursion_m == pytest.approx(5.0)
    assert evaluation.path_length_m == pytest.approx(5.0 + math.sqrt(21.69))
    assert evaluation.yaw_delta_rad == pytest.approx(math.pi / 2.0)
    assert evaluation.xy_drift_rate_m_per_s == pytest.approx(0.25)


def test_loop_evaluation_reports_closure_residual_and_path() -> None:
    # Given: a loop whose final pose does not exactly return to its first pose.
    samples = (
        OdometrySample(stamp_ns=0, frame_id="odom", x_m=0.0, y_m=0.0, z_m=0.0, yaw_rad=0.0),
        OdometrySample(stamp_ns=1, frame_id="odom", x_m=3.0, y_m=4.0, z_m=0.0, yaw_rad=0.0),
        OdometrySample(stamp_ns=2, frame_id="odom", x_m=0.3, y_m=0.4, z_m=1.2, yaw_rad=0.0),
    )

    # When: the loop metric is evaluated.
    evaluation = evaluate_odometry_accuracy(OdometryEvaluationMode.LOOP, samples)

    # Then: the closure residual is measured relative to the initial odometry pose.
    assert isinstance(evaluation, LoopEvaluation)
    assert evaluation.xy_residual_m == pytest.approx(0.5)
    assert evaluation.three_dimensional_residual_m == pytest.approx(1.3)
    assert evaluation.path_length_m == pytest.approx(5.0 + math.sqrt(21.69))


def test_straight_line_evaluation_reports_displacement_and_known_distance_scale() -> None:
    # Given: a known eight metre straight-line course.
    samples = (
        OdometrySample(stamp_ns=0, frame_id="odom", x_m=0.0, y_m=0.0, z_m=0.0, yaw_rad=0.0),
        OdometrySample(stamp_ns=1, frame_id="odom", x_m=6.0, y_m=8.0, z_m=12.0, yaw_rad=0.0),
    )

    # When: the endpoint displacement is compared with the known distance.
    evaluation = evaluate_odometry_accuracy(
        OdometryEvaluationMode.STRAIGHT_LINE,
        samples,
        known_distance_m=8.0,
    )

    # Then: scale describes the odometry distance relative to the measured course.
    assert isinstance(evaluation, StraightLineEvaluation)
    assert evaluation.measured_xy_endpoint_displacement_m == pytest.approx(10.0)
    assert evaluation.measured_3d_endpoint_displacement_m == pytest.approx(math.sqrt(244.0))
    assert evaluation.scale_factor_xy == pytest.approx(1.25)
    assert evaluation.scale_error_percent == pytest.approx(25.0)


def test_sample_rejects_nonfinite_pose_values() -> None:
    # Given: a pose with a non-finite position component.
    # When: the sample is constructed.
    with pytest.raises(OdometryValidationError) as raised:
        OdometrySample(stamp_ns=0, frame_id="odom", x_m=math.nan, y_m=0.0, z_m=0.0, yaw_rad=0.0)

    # Then: the invalid pose is rejected at the pure-engine boundary.
    assert raised.value.issue is OdometryValidationIssue.NONFINITE_POSE


def test_sample_rejects_empty_frame() -> None:
    # Given: a pose without an odometry frame.
    # When: the sample is constructed.
    with pytest.raises(OdometryValidationError) as raised:
        OdometrySample(stamp_ns=0, frame_id="", x_m=0.0, y_m=0.0, z_m=0.0, yaw_rad=0.0)

    # Then: the frame invariant is enforced before evaluation.
    assert raised.value.issue is OdometryValidationIssue.EMPTY_FRAME


def test_evaluation_rejects_frame_change() -> None:
    # Given: samples from different coordinate frames.
    samples = (
        OdometrySample(stamp_ns=0, frame_id="odom", x_m=0.0, y_m=0.0, z_m=0.0, yaw_rad=0.0),
        OdometrySample(stamp_ns=1, frame_id="map", x_m=0.0, y_m=0.0, z_m=0.0, yaw_rad=0.0),
    )

    # When: a stationary metric is requested.
    with pytest.raises(OdometryValidationError) as raised:
        evaluate_odometry_accuracy(OdometryEvaluationMode.STATIONARY, samples)

    # Then: the evaluator does not compare coordinates from different frames.
    assert raised.value.issue is OdometryValidationIssue.FRAME_CHANGED


def test_evaluation_rejects_duplicate_stamp() -> None:
    # Given: two samples with the same nanosecond stamp.
    samples = (
        OdometrySample(stamp_ns=1, frame_id="odom", x_m=0.0, y_m=0.0, z_m=0.0, yaw_rad=0.0),
        OdometrySample(stamp_ns=1, frame_id="odom", x_m=0.0, y_m=0.0, z_m=0.0, yaw_rad=0.0),
    )

    # When: a stationary metric is requested.
    with pytest.raises(OdometryValidationError) as raised:
        evaluate_odometry_accuracy(OdometryEvaluationMode.STATIONARY, samples)

    # Then: time must increase strictly.
    assert raised.value.issue is OdometryValidationIssue.NON_INCREASING_STAMP


def test_evaluation_rejects_regressed_stamp() -> None:
    # Given: the second sample regresses in nanosecond time.
    samples = (
        OdometrySample(stamp_ns=2, frame_id="odom", x_m=0.0, y_m=0.0, z_m=0.0, yaw_rad=0.0),
        OdometrySample(stamp_ns=1, frame_id="odom", x_m=0.0, y_m=0.0, z_m=0.0, yaw_rad=0.0),
    )

    # When: a stationary metric is requested.
    with pytest.raises(OdometryValidationError) as raised:
        evaluate_odometry_accuracy(OdometryEvaluationMode.STATIONARY, samples)

    # Then: time regressions are rejected.
    assert raised.value.issue is OdometryValidationIssue.NON_INCREASING_STAMP


def test_evaluation_requires_at_least_two_samples() -> None:
    # Given: an otherwise valid single odometry sample.
    samples = (
        OdometrySample(stamp_ns=0, frame_id="odom", x_m=0.0, y_m=0.0, z_m=0.0, yaw_rad=0.0),
    )

    # When: a loop metric is requested.
    with pytest.raises(OdometryValidationError) as raised:
        evaluate_odometry_accuracy(OdometryEvaluationMode.LOOP, samples)

    # Then: endpoint metrics cannot be formed without a distinct end sample.
    assert raised.value.issue is OdometryValidationIssue.INSUFFICIENT_SAMPLES


def test_evaluation_rejects_known_distance_outside_straight_line_mode() -> None:
    # Given: a known distance supplied to a stationary evaluation.
    samples = (
        OdometrySample(stamp_ns=0, frame_id="odom", x_m=0.0, y_m=0.0, z_m=0.0, yaw_rad=0.0),
        OdometrySample(stamp_ns=1, frame_id="odom", x_m=0.0, y_m=0.0, z_m=0.0, yaw_rad=0.0),
    )

    # When: the unrelated known-distance argument is supplied.
    with pytest.raises(OdometryValidationError) as raised:
        evaluate_odometry_accuracy(
            OdometryEvaluationMode.STATIONARY,
            samples,
            known_distance_m=1.0,
        )

    # Then: only straight-line evaluation accepts a known physical distance.
    assert raised.value.issue is OdometryValidationIssue.UNEXPECTED_KNOWN_DISTANCE


@pytest.mark.parametrize("known_distance_m", (None, 0.0, math.inf))
def test_straight_line_evaluation_rejects_missing_or_invalid_known_distance(
    known_distance_m: float | None,
) -> None:
    # Given: a valid two-sample straight-line odometry trace.
    samples = (
        OdometrySample(stamp_ns=0, frame_id="odom", x_m=0.0, y_m=0.0, z_m=0.0, yaw_rad=0.0),
        OdometrySample(stamp_ns=1, frame_id="odom", x_m=1.0, y_m=0.0, z_m=0.0, yaw_rad=0.0),
    )

    # When: its known physical distance is absent, zero, or non-finite.
    with pytest.raises(OdometryValidationError) as raised:
        evaluate_odometry_accuracy(
            OdometryEvaluationMode.STRAIGHT_LINE,
            samples,
            known_distance_m=known_distance_m,
        )

    # Then: scale is only reported against a finite positive reference distance.
    assert raised.value.issue in {
        OdometryValidationIssue.MISSING_KNOWN_DISTANCE,
        OdometryValidationIssue.INVALID_KNOWN_DISTANCE,
    }
