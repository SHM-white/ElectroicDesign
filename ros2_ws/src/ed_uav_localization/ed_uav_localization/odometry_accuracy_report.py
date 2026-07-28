"""Pure configuration and result rendering for odometry accuracy trials."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Final, NoReturn, TypedDict

from ed_uav_localization.odometry_accuracy import (
    INTERPRETATION,
    LoopEvaluation,
    OdometryAccuracyEvaluation,
    OdometryEvaluationMode,
    OdometrySample,
    StationaryEvaluation,
    StraightLineEvaluation,
    evaluate_odometry_accuracy,
)

SCHEMA_VERSION: Final[int] = 1
RESULT_PREFIX: Final[str] = "ODOMETRY_ACCURACY_RESULT="
PASSED: Final[str] = "passed"
INTERRUPTED: Final[str] = "INTERRUPTED"
NO_SAMPLE_TIMEOUT: Final[str] = "NO_SAMPLE_TIMEOUT"
STALE_ODOMETRY: Final[str] = "STALE_ODOMETRY"
INSUFFICIENT_SAMPLES: Final[str] = "INSUFFICIENT_SAMPLES"
INVALID_CONFIGURATION: Final[str] = "INVALID_CONFIGURATION"
DEFAULT_ODOM_TOPIC: Final[str] = "/localization/odom"


class StationaryMetrics(TypedDict):
    mode: str
    end_xy_drift_m: float
    end_3d_drift_m: float
    max_xy_excursion_m: float
    path_length_m: float
    yaw_delta_rad: float
    xy_drift_rate_m_per_s: float


class LoopMetrics(TypedDict):
    mode: str
    xy_residual_m: float
    three_dimensional_residual_m: float
    path_length_m: float


class StraightLineMetrics(TypedDict):
    mode: str
    measured_xy_endpoint_displacement_m: float
    measured_3d_endpoint_displacement_m: float
    scale_factor_xy: float
    scale_error_percent: float


Metrics = StationaryMetrics | LoopMetrics | StraightLineMetrics


class TrialResult(TypedDict):
    schema_version: int
    status: str
    trial: str
    interpretation: str
    input_topic: str
    frame_id: str | None
    start_stamp_ns: int | None
    end_stamp_ns: int | None
    duration_sec: float | None
    sample_count: int
    rejected_count: int
    metrics: Metrics | None


@dataclass(frozen=True, slots=True)
class InvalidConfigurationError(ValueError):
    message: str

    def __str__(self) -> str:
        return self.message


@dataclass(frozen=True, slots=True)
class TrialConfiguration:
    mode: OdometryEvaluationMode
    odom_topic: str = DEFAULT_ODOM_TOPIC
    duration_sec: float = 30.0
    known_distance_m: float | None = None
    start_timeout_sec: float = 10.0
    stale_timeout_sec: float = 0.5
    min_samples: int = 2

    def __post_init__(self) -> None:
        if not self.odom_topic.strip():
            raise InvalidConfigurationError("odom topic must not be empty")
        if self.min_samples < 2:
            raise InvalidConfigurationError("min samples must be at least two")
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (self.duration_sec, self.start_timeout_sec, self.stale_timeout_sec)
        ):
            raise InvalidConfigurationError("durations must be finite and positive")
        _validate_engine_configuration(self.mode, self.known_distance_m)


def _validate_engine_configuration(
    mode: OdometryEvaluationMode, known_distance_m: float | None
) -> None:
    samples = (
        OdometrySample(0, "configuration", 0.0, 0.0, 0.0, 0.0),
        OdometrySample(1, "configuration", 0.0, 0.0, 0.0, 0.0),
    )
    evaluate_odometry_accuracy(mode, samples, known_distance_m)


def metrics_from_evaluation(evaluation: OdometryAccuracyEvaluation) -> Metrics:
    match evaluation:
        case StationaryEvaluation():
            return {
                "mode": evaluation.mode.value,
                "end_xy_drift_m": evaluation.end_xy_drift_m,
                "end_3d_drift_m": evaluation.end_3d_drift_m,
                "max_xy_excursion_m": evaluation.max_xy_excursion_m,
                "path_length_m": evaluation.path_length_m,
                "yaw_delta_rad": evaluation.yaw_delta_rad,
                "xy_drift_rate_m_per_s": evaluation.xy_drift_rate_m_per_s,
            }
        case LoopEvaluation():
            return {
                "mode": evaluation.mode.value,
                "xy_residual_m": evaluation.xy_residual_m,
                "three_dimensional_residual_m": evaluation.three_dimensional_residual_m,
                "path_length_m": evaluation.path_length_m,
            }
        case StraightLineEvaluation():
            return {
                "mode": evaluation.mode.value,
                "measured_xy_endpoint_displacement_m": evaluation.measured_xy_endpoint_displacement_m,
                "measured_3d_endpoint_displacement_m": evaluation.measured_3d_endpoint_displacement_m,
                "scale_factor_xy": evaluation.scale_factor_xy,
                "scale_error_percent": evaluation.scale_error_percent,
            }
        case unreachable:
            _assert_never(unreachable)


def format_result(result: TrialResult) -> str:
    return RESULT_PREFIX + json.dumps(result, allow_nan=False, separators=(",", ":"), sort_keys=True)


def _assert_never(value: NoReturn) -> NoReturn:
    raise AssertionError(f"Unexpected evaluation type: {value}")
