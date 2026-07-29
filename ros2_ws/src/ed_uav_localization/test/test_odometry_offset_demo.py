from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_localization.odometry_offset_demo import (
    DEFAULT_ODOM_TOPIC,
    OffsetDemoConfiguration,
    StartupRelativeOffsetReceiver,
    configuration_from_argv,
    format_offset_line,
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


def odometry(
    stamp_ns: int,
    *,
    frame_id: str = "odom",
    x_m: float = 0.0,
    y_m: float = 0.0,
    z_m: float = 0.0,
    yaw_rad: float = 0.0,
) -> FakeOdometry:
    seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
    return FakeOdometry(
        Header(Stamp(seconds, nanoseconds), frame_id),
        PoseWithCovariance(
            Pose(
                Position(x_m, y_m, z_m),
                Orientation(0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)),
            )
        ),
    )


def test_configuration_resolves_cli_then_environment_then_default(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Given: an environment topic that differs from both the default and CLI topic.
    monkeypatch.setenv("ODOM_TOPIC", "/environment/odom")

    # When: each supported source is parsed at the CLI boundary.
    cli = configuration_from_argv(["--odom-topic", "/cli/odom"])
    environment = configuration_from_argv([])
    monkeypatch.delenv("ODOM_TOPIC")
    default = configuration_from_argv([])

    # Then: the documented precedence applies while output throttling retains its default.
    assert cli.odom_topic == "/cli/odom"
    assert environment.odom_topic == "/environment/odom"
    assert default.odom_topic == DEFAULT_ODOM_TOPIC == "/localization/odom"
    assert default.output_rate_hz == 2.0


def test_receiver_emits_zero_origin_then_throttles_accepted_output(
    capsys: pytest.CaptureFixture[str],
) -> None:
    # Given: a two-hertz receiver at deterministic receive times.
    receiver = StartupRelativeOffsetReceiver(OffsetDemoConfiguration(output_rate_hz=2.0))

    # When: it receives the origin and two valid follow-up samples.
    receiver.receive(odometry(10, x_m=5.0), received_at_sec=0.0)
    receiver.receive(odometry(11, x_m=6.0), received_at_sec=0.2)
    receiver.receive(odometry(12, x_m=7.0), received_at_sec=0.5)

    # Then: the zero line is immediate and later output is rate-limited without losing state.
    lines = capsys.readouterr().out.splitlines()
    assert len(lines) == 2
    assert lines[0].startswith("LIDAR_ODOMETRY_OFFSET ")
    assert "dx_m=0.000000" in lines[0]
    assert "dx_m=2.000000" in lines[1]
    assert receiver.last_accepted == sample_from_odometry(odometry(12, x_m=7.0))


def test_receiver_rejects_invalid_followup_without_moving_origin_or_last_state(
    capsys: pytest.CaptureFixture[str],
) -> None:
    # Given: a receiver with a valid startup origin and accepted follow-up.
    receiver = StartupRelativeOffsetReceiver(OffsetDemoConfiguration())
    receiver.receive(odometry(10, x_m=1.0), received_at_sec=0.0)
    receiver.receive(odometry(11, x_m=2.0), received_at_sec=1.0)
    origin = receiver.origin
    last_accepted = receiver.last_accepted
    capsys.readouterr()

    # When: a sample changes frame after startup.
    receiver.receive(odometry(12, frame_id="map", x_m=50.0), received_at_sec=2.0)

    # Then: one concise rejection is printed and both accepted state values remain unchanged.
    assert capsys.readouterr().err.splitlines() == [
        "LIDAR_ODOMETRY_OFFSET_REJECTED issue=frame_changed"
    ]
    assert receiver.origin == origin
    assert receiver.last_accepted == last_accepted


def test_offset_line_has_fixed_single_line_metric_format() -> None:
    # Given: a parsed ROS-shaped pose offset from its startup pose.
    offset = (
        StartupRelativeOffsetReceiver(OffsetDemoConfiguration())
        .state.accept(sample_from_odometry(odometry(10, x_m=1.0)))
        .accept(sample_from_odometry(odometry(11, x_m=4.0, y_m=4.0, z_m=3.0)))
        .offset
    )

    # When: it is rendered for the live console.
    assert offset is not None
    line = format_offset_line(offset)

    # Then: each required relative metric stays on a stable one-line surface.
    assert line.startswith("LIDAR_ODOMETRY_OFFSET ")
    assert line.count("\n") == 0
    assert "dx_m=3.000000" in line
    assert "dy_m=4.000000" in line
    assert "dz_m=3.000000" in line
    assert "xy_distance_m=5.000000" in line
    assert "distance_3d_m=5.830952" in line
    assert "yaw_delta_rad=0.000000" in line


def test_setup_registers_lidar_odometry_offset_demo_console_entry() -> None:
    # Given: the package installation metadata.
    setup_text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    # When: console scripts are inspected.
    # Then: ROS can execute the startup-relative live demo by its stable command name.
    assert (
        "lidar_odometry_offset_demo = ed_uav_localization.odometry_offset_demo:main"
        in setup_text
    )
