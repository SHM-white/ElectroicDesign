from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_localization.odometry_accuracy import (
    OdometryEvaluationMode,
    OdometryValidationError,
    OdometryValidationIssue,
)
from ed_uav_localization.odometry_accuracy_demo import (
    INSUFFICIENT_SAMPLES,
    INVALID_CONFIGURATION,
    NO_SAMPLE_TIMEOUT,
    STALE_ODOMETRY,
    OdometryAccuracyTrial,
    TrialConfiguration,
    configuration_from_argv,
    format_result,
    main,
    sample_from_odometry,
)


@dataclass(frozen=True, slots=True)
class Stamp:
    sec: int
    nanosec: int


@dataclass(frozen=True, slots=True)
class Header:
    stamp: Stamp
    frame_id: str


@dataclass(frozen=True, slots=True)
class Position:
    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class Orientation:
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True, slots=True)
class Pose:
    position: Position
    orientation: Orientation


@dataclass(frozen=True, slots=True)
class PoseWithCovariance:
    pose: Pose


@dataclass(frozen=True, slots=True)
class FakeOdometry:
    header: Header
    pose: PoseWithCovariance


def _odometry(
    *,
    stamp_ns: int,
    x_m: float = 0.0,
    frame_id: str = "odom",
    yaw_rad: float = 0.0,
) -> FakeOdometry:
    seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
    return FakeOdometry(
        header=Header(stamp=Stamp(sec=seconds, nanosec=nanoseconds), frame_id=frame_id),
        pose=PoseWithCovariance(
            pose=Pose(
                position=Position(x=x_m, y=0.0, z=0.0),
                orientation=Orientation(
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw_rad / 2.0),
                    w=math.cos(yaw_rad / 2.0),
                ),
            )
        ),
    )


def test_setup_exposes_the_odometry_accuracy_demo_console_script() -> None:
    setup_text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert (
        "odometry_accuracy_demo = ed_uav_localization.odometry_accuracy_demo:main"
        in setup_text
    )


def test_configuration_uses_cli_defaults_and_requested_mode() -> None:
    # Given: an explicit loop evaluation with no optional overrides.
    # When: the CLI boundary is parsed.
    configuration = configuration_from_argv(["--mode", "loop"])

    # Then: the reusable configuration holds the documented defaults.
    assert configuration.mode is OdometryEvaluationMode.LOOP
    assert configuration.odom_topic == "/localization/odom"
    assert configuration.duration_sec == 30.0
    assert configuration.start_timeout_sec == 10.0
    assert configuration.stale_timeout_sec == 0.5
    assert configuration.min_samples == 2


@pytest.mark.parametrize(
    ("arguments", "issue"),
    (
        (["--mode", "straight_line"], OdometryValidationIssue.MISSING_KNOWN_DISTANCE),
        (
            ["--mode", "stationary", "--known-distance-m", "10"],
            OdometryValidationIssue.UNEXPECTED_KNOWN_DISTANCE,
        ),
    ),
)
def test_configuration_enforces_mode_specific_known_distance(
    arguments: list[str], issue: OdometryValidationIssue
) -> None:
    # Given: a mode-specific known-distance configuration violation.
    # When: arguments enter the pure CLI boundary.
    with pytest.raises(OdometryValidationError) as raised:
        configuration_from_argv(arguments)

    # Then: the pure evaluation contract identifies the invalid mode relation.
    assert raised.value.issue is issue


def test_main_emits_one_honest_json_result_for_invalid_configuration(capsys: pytest.CaptureFixture[str]) -> None:
    # Given: a non-positive trial duration.
    # When: the executable boundary is invoked.
    exit_code = main(["--duration-sec", "0"])

    # Then: it fails without ROS and emits one parseable, non-speculative result.
    lines = capsys.readouterr().out.splitlines()
    assert exit_code != 0
    assert len(lines) == 1
    assert lines[0].startswith("ODOMETRY_ACCURACY_RESULT=")
    result = json.loads(lines[0].removeprefix("ODOMETRY_ACCURACY_RESULT="))
    assert result["status"] == INVALID_CONFIGURATION
    assert result["metrics"] is None
    assert result["frame_id"] is None
    assert result["start_stamp_ns"] is None
    assert result["end_stamp_ns"] is None


def test_main_emits_engine_validation_issue_as_structured_json(capsys: pytest.CaptureFixture[str]) -> None:
    # Given: a straight-line trial without its required physical reference distance.
    # When: the executable boundary parses the CLI request.
    exit_code = main(["--mode", "straight_line"])

    # Then: the engine's stable validation issue is emitted without opening a ROS node.
    result = json.loads(capsys.readouterr().out.removeprefix("ODOMETRY_ACCURACY_RESULT="))
    assert exit_code != 0
    assert result["status"] == OdometryValidationIssue.MISSING_KNOWN_DISTANCE.value
    assert result["metrics"] is None


def test_sample_from_odometry_converts_header_pose_and_quaternion_yaw() -> None:
    # Given: a ROS-shaped odometry message rotated ninety degrees about ENU up.
    # When: it crosses the runtime boundary.
    sample = sample_from_odometry(_odometry(stamp_ns=2_000_000_123, x_m=4.5, yaw_rad=math.pi / 2.0))

    # Then: the pure engine receives nanoseconds, position, frame, and yaw in SI units.
    assert sample.stamp_ns == 2_000_000_123
    assert sample.frame_id == "odom"
    assert sample.x_m == 4.5
    assert sample.yaw_rad == pytest.approx(math.pi / 2.0)


def test_trial_rejects_nonfinite_odometry_pose_after_start() -> None:
    # Given: a trial already started by a valid odometry sample.
    trial = OdometryAccuracyTrial(TrialConfiguration(mode=OdometryEvaluationMode.LOOP))
    trial.receive(_odometry(stamp_ns=0))

    # When: a non-finite pose crosses the ROS-shaped boundary.
    trial.receive(_odometry(stamp_ns=1, x_m=math.nan))

    # Then: the engine validation issue is terminal rather than silently discarded.
    result = trial.result()
    assert result["status"] == OdometryValidationIssue.NONFINITE_POSE.value
    assert result["rejected_count"] == 1


def test_trial_reports_no_sample_timeout_without_inventing_measurement_data() -> None:
    # Given: an unstarted stationary trial.
    trial = OdometryAccuracyTrial(TrialConfiguration(mode=OdometryEvaluationMode.STATIONARY))

    # When: its receive-time start deadline expires.
    trial.finish(NO_SAMPLE_TIMEOUT)

    # Then: the result has the required failure code and null measurement fields.
    result = trial.result()
    assert result["status"] == NO_SAMPLE_TIMEOUT
    assert result["sample_count"] == 0
    assert result["duration_sec"] is None
    assert result["metrics"] is None


def test_trial_reports_stale_odometry_after_the_first_accepted_sample() -> None:
    # Given: a trial that began from one valid odometry receipt.
    trial = OdometryAccuracyTrial(TrialConfiguration(mode=OdometryEvaluationMode.LOOP))
    trial.receive(_odometry(stamp_ns=50))

    # When: the runtime detects no further receipts before its stale deadline.
    trial.finish(STALE_ODOMETRY)

    # Then: the result reports the partial trace without pretending it has metrics.
    result = trial.result()
    assert result["status"] == STALE_ODOMETRY
    assert result["sample_count"] == 1
    assert result["start_stamp_ns"] == 50
    assert result["end_stamp_ns"] == 50
    assert result["metrics"] is None


def test_trial_reports_insufficient_samples_at_the_header_time_duration() -> None:
    # Given: a one-second trial requiring three samples.
    configuration = TrialConfiguration(
        mode=OdometryEvaluationMode.STATIONARY,
        duration_sec=1.0,
        min_samples=3,
    )
    trial = OdometryAccuracyTrial(configuration)
    trial.receive(_odometry(stamp_ns=0))

    # When: only the second sample reaches the fixed header-time duration.
    trial.receive(_odometry(stamp_ns=1_000_000_000))

    # Then: the run is terminal and correctly reports insufficient samples.
    result = trial.result()
    assert result["status"] == INSUFFICIENT_SAMPLES
    assert result["sample_count"] == 2
    assert result["metrics"] is None


def test_trial_rejects_frame_and_stamp_violations_after_start() -> None:
    # Given: trials with initial valid samples.
    frame_trial = OdometryAccuracyTrial(TrialConfiguration(mode=OdometryEvaluationMode.LOOP))
    stamp_trial = OdometryAccuracyTrial(TrialConfiguration(mode=OdometryEvaluationMode.LOOP))
    frame_trial.receive(_odometry(stamp_ns=0))
    stamp_trial.receive(_odometry(stamp_ns=1))

    # When: each receives an engine-invalid follow-up sample.
    frame_trial.receive(_odometry(stamp_ns=1, frame_id="map"))
    stamp_trial.receive(_odometry(stamp_ns=1))

    # Then: neither is silently dropped and each uses the engine issue value.
    assert frame_trial.result()["status"] == OdometryValidationIssue.FRAME_CHANGED.value
    assert stamp_trial.result()["status"] == OdometryValidationIssue.NON_INCREASING_STAMP.value
    assert frame_trial.result()["rejected_count"] == 1
    assert stamp_trial.result()["rejected_count"] == 1


def test_completed_synthetic_trial_emits_relative_metrics_as_single_line_json() -> None:
    # Given: a one-second stationary trace with a small relative drift.
    trial = OdometryAccuracyTrial(
        TrialConfiguration(mode=OdometryEvaluationMode.STATIONARY, duration_sec=1.0)
    )
    trial.receive(_odometry(stamp_ns=0, x_m=0.0))
    trial.receive(_odometry(stamp_ns=1_000_000_000, x_m=0.25))

    # When: the completed result is formatted for the CLI.
    result = trial.result()
    output = format_result(result)

    # Then: completion means measurement collection, with no absolute-accuracy claim.
    assert result["status"] == "passed"
    assert result["duration_sec"] == 1.0
    assert result["metrics"] is not None
    assert result["metrics"]["end_xy_drift_m"] == 0.25
    assert output.count("\n") == 0
    parsed = json.loads(output.removeprefix("ODOMETRY_ACCURACY_RESULT="))
    assert parsed["metrics"]["mode"] == "stationary"
    assert "absolute accuracy" in parsed["interpretation"]


def test_trial_live_summary_updates_on_each_accepted_sample(capsys: pytest.CaptureFixture[str]) -> None:
    trial = OdometryAccuracyTrial(TrialConfiguration(mode=OdometryEvaluationMode.LOOP))
    trial.receive(_odometry(stamp_ns=0, x_m=0.0))
    trial.receive(_odometry(stamp_ns=1_000_000_000, x_m=2.0))
    lines = [line for line in capsys.readouterr().out.splitlines() if line.startswith("ODOMETRY_ACCURACY_LIVE ")]
    assert len(lines) == 2
    assert "dx_m=0.000000" in lines[0]
    assert "xy_m=0.000000" in lines[0]
    assert "frame=odom" in lines[1]
    assert "dx_m=2.000000" in lines[1]

