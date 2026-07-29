"""Live startup-relative lidar odometry console demo."""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Final, NoReturn, Protocol, Sequence

from ed_uav_localization.odometry_accuracy import (
    OdometrySample,
    OdometryValidationError,
)
from ed_uav_localization.odometry_offset import OdometryOffset, StartupRelativeOdometry

DEFAULT_ODOM_TOPIC: Final[str] = "/localization/odom"
DEFAULT_OUTPUT_RATE_HZ: Final[float] = 2.0
OUTPUT_PREFIX: Final[str] = "LIDAR_ODOMETRY_OFFSET"
REJECTION_PREFIX: Final[str] = "LIDAR_ODOMETRY_OFFSET_REJECTED"


class RosStamp(Protocol):
    sec: int
    nanosec: int


class RosHeader(Protocol):
    stamp: RosStamp
    frame_id: str


class RosPosition(Protocol):
    x: float
    y: float
    z: float


class RosOrientation(Protocol):
    x: float
    y: float
    z: float
    w: float


class RosPose(Protocol):
    position: RosPosition
    orientation: RosOrientation


class RosPoseWithCovariance(Protocol):
    pose: RosPose


class RosOdometry(Protocol):
    header: RosHeader
    pose: RosPoseWithCovariance


@dataclass(frozen=True, slots=True)
class OffsetDemoConfigurationError(ValueError):
    message: str

    def __str__(self) -> str:
        return self.message


@dataclass(frozen=True, slots=True)
class OffsetDemoConfiguration:
    odom_topic: str = DEFAULT_ODOM_TOPIC
    output_rate_hz: float = DEFAULT_OUTPUT_RATE_HZ

    def __post_init__(self) -> None:
        if not self.odom_topic.strip():
            raise OffsetDemoConfigurationError("odom topic must not be empty")
        if not math.isfinite(self.output_rate_hz) or self.output_rate_hz <= 0.0:
            raise OffsetDemoConfigurationError("output rate must be finite and positive")


class _ArgumentParser(argparse.ArgumentParser):
    def error(self, message: str) -> NoReturn:
        raise OffsetDemoConfigurationError(message)


def configuration_from_argv(argv: Sequence[str]) -> OffsetDemoConfiguration:
    """Parse CLI and environment topic settings with explicit precedence."""
    parser = _ArgumentParser(prog="lidar-odometry-offset-demo")
    parser.add_argument("--odom-topic")
    parser.add_argument("--output-rate-hz", type=float, default=DEFAULT_OUTPUT_RATE_HZ)
    arguments = parser.parse_args(argv)
    odom_topic = arguments.odom_topic
    if odom_topic is None:
        odom_topic = os.environ.get("ODOM_TOPIC", DEFAULT_ODOM_TOPIC)
    return OffsetDemoConfiguration(odom_topic=odom_topic, output_rate_hz=arguments.output_rate_hz)


def sample_from_odometry(message: RosOdometry) -> OdometrySample:
    """Parse a ROS odometry pose into the shared finite validated sample value."""
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return OdometrySample(
        stamp_ns=message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
        frame_id=message.header.frame_id,
        x_m=position.x,
        y_m=position.y,
        z_m=position.z,
        yaw_rad=math.atan2(siny_cosp, cosy_cosp),
    )


def format_offset_line(offset: OdometryOffset) -> str:
    """Render one stable, machine-readable live offset line."""
    return (
        f"{OUTPUT_PREFIX} frame_id={offset.frame_id} stamp_ns={offset.stamp_ns} "
        f"dx_m={offset.dx_m:.6f} dy_m={offset.dy_m:.6f} dz_m={offset.dz_m:.6f} "
        f"xy_distance_m={offset.xy_distance_m:.6f} "
        f"distance_3d_m={offset.distance_3d_m:.6f} "
        f"yaw_delta_rad={offset.yaw_delta_rad:.6f}"
    )


class StartupRelativeOffsetReceiver:
    """Mutable ROS callback receiver that only replaces state after validation."""

    def __init__(self, configuration: OffsetDemoConfiguration) -> None:
        self._output_interval_sec = 1.0 / configuration.output_rate_hz
        self._state = StartupRelativeOdometry()
        self._last_output_sec: float | None = None

    @property
    def state(self) -> StartupRelativeOdometry:
        return self._state

    @property
    def origin(self) -> OdometrySample | None:
        return self._state.origin

    @property
    def last_accepted(self) -> OdometrySample | None:
        return self._state.last_accepted

    def receive(self, message: RosOdometry, *, received_at_sec: float) -> None:
        """Parse, validate, and conditionally print one incoming odometry message."""
        try:
            sample = sample_from_odometry(message)
            next_state = self._state.accept(sample)
        except OdometryValidationError as error:
            print(f"{REJECTION_PREFIX} issue={error.issue.value}", file=sys.stderr, flush=True)
            return
        origin_was_unset = self._state.origin is None
        self._state = next_state
        offset = next_state.offset
        if offset is None:
            raise AssertionError("accepted state must produce an offset")
        if origin_was_unset or self._should_emit(received_at_sec):
            print(format_offset_line(offset), flush=True)
            self._last_output_sec = received_at_sec

    def _should_emit(self, received_at_sec: float) -> bool:
        if self._last_output_sec is None:
            return True
        return received_at_sec - self._last_output_sec >= self._output_interval_sec


def run_ros_demo(configuration: OffsetDemoConfiguration) -> int:
    """Run the foreground ROS subscription until interrupted or shutdown externally."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    rclpy.init(args=[])
    node = Node("lidar_odometry_offset_demo")
    receiver = StartupRelativeOffsetReceiver(configuration)

    def receive(message: RosOdometry) -> None:
        receiver.receive(message, received_at_sec=time.monotonic())

    node.create_subscription(Odometry, configuration.odom_topic, receive, 10)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    """Run the live lidar odometry offset demo command."""
    arguments = sys.argv[1:] if argv is None else argv
    try:
        configuration = configuration_from_argv(arguments)
    except OffsetDemoConfigurationError as error:
        print(f"{REJECTION_PREFIX} issue=invalid_configuration detail={error}", file=sys.stderr)
        return 2
    return run_ros_demo(configuration)


if __name__ == "__main__":
    raise SystemExit(main())
