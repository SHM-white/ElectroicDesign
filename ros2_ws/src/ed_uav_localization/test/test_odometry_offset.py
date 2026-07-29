from __future__ import annotations

import math
import sys
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_localization.odometry_accuracy import (
    OdometrySample,
    OdometryValidationError,
    OdometryValidationIssue,
)
from ed_uav_localization.odometry_offset import StartupRelativeOdometry


def sample(
    stamp_ns: int,
    *,
    frame_id: str = "odom",
    x_m: float = 0.0,
    y_m: float = 0.0,
    z_m: float = 0.0,
    yaw_rad: float = 0.0,
) -> OdometrySample:
    return OdometrySample(stamp_ns, frame_id, x_m, y_m, z_m, yaw_rad)


def test_startup_relative_odometry_captures_an_immutable_zero_origin() -> None:
    # Given: the first valid lidar odometry pose at an arbitrary map position.
    first = sample(100, x_m=10.0, y_m=-3.0, z_m=2.0, yaw_rad=math.pi - 0.1)

    # When: it is accepted by the startup-relative state machine.
    state = StartupRelativeOdometry().accept(first)

    # Then: the first pose remains a frozen origin and reports an immediate zero offset.
    assert state.origin == first
    assert state.last_accepted == first
    assert state.offset is not None
    assert state.offset.dx_m == 0.0
    assert state.offset.dy_m == 0.0
    assert state.offset.dz_m == 0.0
    assert state.offset.xy_distance_m == 0.0
    assert state.offset.distance_3d_m == 0.0
    assert state.offset.yaw_delta_rad == 0.0
    with pytest.raises(FrozenInstanceError):
        state.origin.x_m = 0.0


def test_startup_relative_odometry_reports_translation_and_wrapped_yaw() -> None:
    # Given: a startup origin and a later accepted pose in its frame.
    origin = sample(10, x_m=1.0, y_m=2.0, z_m=3.0, yaw_rad=math.pi - 0.1)
    later = sample(20, x_m=4.0, y_m=6.0, z_m=5.0, yaw_rad=-math.pi + 0.1)

    # When: the later pose advances the state.
    offset = StartupRelativeOdometry().accept(origin).accept(later).offset

    # Then: all distances are origin-relative and yaw uses the shortest wrapped delta.
    assert offset is not None
    assert offset.dx_m == 3.0
    assert offset.dy_m == 4.0
    assert offset.dz_m == 2.0
    assert offset.xy_distance_m == 5.0
    assert offset.distance_3d_m == pytest.approx(math.sqrt(29.0))
    assert offset.yaw_delta_rad == pytest.approx(0.2)


@pytest.mark.parametrize(
    ("invalid", "issue"),
    (
        (sample(11, frame_id="map"), OdometryValidationIssue.FRAME_CHANGED),
        (sample(10), OdometryValidationIssue.NON_INCREASING_STAMP),
        (sample(9), OdometryValidationIssue.NON_INCREASING_STAMP),
    ),
)
def test_startup_relative_odometry_rejects_invalid_followups_without_state_change(
    invalid: OdometrySample, issue: OdometryValidationIssue
) -> None:
    # Given: a state already initialized from one accepted lidar pose.
    started = StartupRelativeOdometry().accept(sample(10))

    # When: a frame or timestamp-invalid follow-up arrives.
    with pytest.raises(OdometryValidationError) as raised:
        started.accept(invalid)

    # Then: its precise validation issue is raised and the immutable state remains unchanged.
    assert raised.value.issue is issue
    assert started.origin == sample(10)
    assert started.last_accepted == sample(10)


def test_odometry_sample_rejects_nonfinite_input_before_offset_state() -> None:
    # Given: a non-finite lidar pose value at the ROS boundary.
    # When: it is parsed into the shared odometry sample value.
    with pytest.raises(OdometryValidationError) as raised:
        sample(10, x_m=math.nan)

    # Then: no offset state can be established from it.
    assert raised.value.issue is OdometryValidationIssue.NONFINITE_POSE
