from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from enum import Enum
from typing import Final, Literal, NoReturn, TypeAlias


INTERPRETATION: Final[str] = (
    "These metrics characterize relative odometry behavior and do not establish "
    "absolute accuracy without an external reference."
)
MINIMUM_SAMPLE_COUNT: Final[int] = 2


class OdometryEvaluationMode(str, Enum):
    STATIONARY = "stationary"
    LOOP = "loop"
    STRAIGHT_LINE = "straight_line"


class OdometryValidationIssue(str, Enum):
    EMPTY_FRAME = "empty_frame"
    NONFINITE_POSE = "nonfinite_pose"
    FRAME_CHANGED = "frame_changed"
    NON_INCREASING_STAMP = "non_increasing_stamp"
    INSUFFICIENT_SAMPLES = "insufficient_samples"
    UNEXPECTED_KNOWN_DISTANCE = "unexpected_known_distance"
    MISSING_KNOWN_DISTANCE = "missing_known_distance"
    INVALID_KNOWN_DISTANCE = "invalid_known_distance"


@dataclass(frozen=True, slots=True)
class OdometryValidationError(ValueError):
    issue: OdometryValidationIssue

    def __str__(self) -> str:
        return self.issue.value


@dataclass(frozen=True, slots=True)
class OdometrySample:
    stamp_ns: int
    frame_id: str
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        if not self.frame_id.strip():
            raise OdometryValidationError(OdometryValidationIssue.EMPTY_FRAME)
        if not all(
            math.isfinite(value)
            for value in (self.x_m, self.y_m, self.z_m, self.yaw_rad)
        ):
            raise OdometryValidationError(OdometryValidationIssue.NONFINITE_POSE)


@dataclass(frozen=True, slots=True)
class StationaryEvaluation:
    end_xy_drift_m: float
    end_3d_drift_m: float
    max_xy_excursion_m: float
    path_length_m: float
    yaw_delta_rad: float
    xy_drift_rate_m_per_s: float
    mode: Literal[OdometryEvaluationMode.STATIONARY] = field(
        default=OdometryEvaluationMode.STATIONARY,
        init=False,
    )


@dataclass(frozen=True, slots=True)
class LoopEvaluation:
    xy_residual_m: float
    three_dimensional_residual_m: float
    path_length_m: float
    mode: Literal[OdometryEvaluationMode.LOOP] = field(
        default=OdometryEvaluationMode.LOOP,
        init=False,
    )


@dataclass(frozen=True, slots=True)
class StraightLineEvaluation:
    measured_xy_endpoint_displacement_m: float
    measured_3d_endpoint_displacement_m: float
    scale_factor_xy: float
    scale_error_percent: float
    mode: Literal[OdometryEvaluationMode.STRAIGHT_LINE] = field(
        default=OdometryEvaluationMode.STRAIGHT_LINE,
        init=False,
    )


OdometryAccuracyEvaluation: TypeAlias = (
    StationaryEvaluation | LoopEvaluation | StraightLineEvaluation
)


def evaluate_odometry_accuracy(
    mode: OdometryEvaluationMode,
    samples: Sequence[OdometrySample],
    known_distance_m: float | None = None,
) -> OdometryAccuracyEvaluation:
    sample_snapshot = tuple(samples)
    _validate_samples(sample_snapshot)

    match mode:
        case OdometryEvaluationMode.STATIONARY:
            _reject_known_distance(known_distance_m)
            return _evaluate_stationary(sample_snapshot)
        case OdometryEvaluationMode.LOOP:
            _reject_known_distance(known_distance_m)
            return _evaluate_loop(sample_snapshot)
        case OdometryEvaluationMode.STRAIGHT_LINE:
            return _evaluate_straight_line(sample_snapshot, _known_distance(known_distance_m))
        case unreachable:
            _assert_never(unreachable)


def _validate_samples(samples: tuple[OdometrySample, ...]) -> None:
    if len(samples) < MINIMUM_SAMPLE_COUNT:
        raise OdometryValidationError(OdometryValidationIssue.INSUFFICIENT_SAMPLES)

    frame_id = samples[0].frame_id
    previous_stamp_ns = samples[0].stamp_ns
    for sample in samples[1:]:
        if sample.frame_id != frame_id:
            raise OdometryValidationError(OdometryValidationIssue.FRAME_CHANGED)
        if sample.stamp_ns <= previous_stamp_ns:
            raise OdometryValidationError(OdometryValidationIssue.NON_INCREASING_STAMP)
        previous_stamp_ns = sample.stamp_ns


def _reject_known_distance(known_distance_m: float | None) -> None:
    if known_distance_m is not None:
        raise OdometryValidationError(OdometryValidationIssue.UNEXPECTED_KNOWN_DISTANCE)


def _known_distance(known_distance_m: float | None) -> float:
    if known_distance_m is None:
        raise OdometryValidationError(OdometryValidationIssue.MISSING_KNOWN_DISTANCE)
    if not math.isfinite(known_distance_m) or known_distance_m <= 0.0:
        raise OdometryValidationError(OdometryValidationIssue.INVALID_KNOWN_DISTANCE)
    return known_distance_m


def _evaluate_stationary(samples: tuple[OdometrySample, ...]) -> StationaryEvaluation:
    start = samples[0]
    end = samples[-1]
    end_xy_drift_m = _xy_distance(start, end)
    duration_s = (end.stamp_ns - start.stamp_ns) * 1e-9
    return StationaryEvaluation(
        end_xy_drift_m=end_xy_drift_m,
        end_3d_drift_m=_three_dimensional_distance(start, end),
        max_xy_excursion_m=max(_xy_distance(start, sample) for sample in samples),
        path_length_m=_path_length(samples),
        yaw_delta_rad=_wrapped_yaw_delta(start.yaw_rad, end.yaw_rad),
        xy_drift_rate_m_per_s=end_xy_drift_m / duration_s,
    )


def _evaluate_loop(samples: tuple[OdometrySample, ...]) -> LoopEvaluation:
    return LoopEvaluation(
        xy_residual_m=_xy_distance(samples[0], samples[-1]),
        three_dimensional_residual_m=_three_dimensional_distance(samples[0], samples[-1]),
        path_length_m=_path_length(samples),
    )


def _evaluate_straight_line(
    samples: tuple[OdometrySample, ...],
    known_distance_m: float,
) -> StraightLineEvaluation:
    measured_xy_endpoint_displacement_m = _xy_distance(samples[0], samples[-1])
    scale_factor_xy = measured_xy_endpoint_displacement_m / known_distance_m
    return StraightLineEvaluation(
        measured_xy_endpoint_displacement_m=measured_xy_endpoint_displacement_m,
        measured_3d_endpoint_displacement_m=_three_dimensional_distance(
            samples[0], samples[-1]
        ),
        scale_factor_xy=scale_factor_xy,
        scale_error_percent=(scale_factor_xy - 1.0) * 100.0,
    )


def _xy_distance(left: OdometrySample, right: OdometrySample) -> float:
    return math.hypot(right.x_m - left.x_m, right.y_m - left.y_m)


def _three_dimensional_distance(left: OdometrySample, right: OdometrySample) -> float:
    return math.hypot(
        right.x_m - left.x_m,
        right.y_m - left.y_m,
        right.z_m - left.z_m,
    )


def _path_length(samples: tuple[OdometrySample, ...]) -> float:
    return sum(
        _three_dimensional_distance(previous, current)
        for previous, current in zip(samples, samples[1:])
    )


def _wrapped_yaw_delta(start_yaw_rad: float, end_yaw_rad: float) -> float:
    raw_delta_rad = end_yaw_rad - start_yaw_rad
    return math.atan2(math.sin(raw_delta_rad), math.cos(raw_delta_rad))


def _assert_never(value: NoReturn) -> NoReturn:
    raise AssertionError(f"Unexpected evaluation mode: {value}")
