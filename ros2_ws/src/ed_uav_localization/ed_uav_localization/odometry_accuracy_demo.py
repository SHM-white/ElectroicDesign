from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Protocol, Sequence

from ed_uav_localization.odometry_accuracy import (
    OdometryAccuracyEvaluation,
    OdometryEvaluationMode,
    OdometrySample,
    OdometryValidationError,
    evaluate_odometry_accuracy,
)
from ed_uav_localization.odometry_accuracy_live_report import LiveSampleSummary, live_sample_summary
from ed_uav_localization.odometry_accuracy_report import (
    DEFAULT_ODOM_TOPIC,
    INTERPRETATION,
    INSUFFICIENT_SAMPLES,
    INTERRUPTED,
    INVALID_CONFIGURATION,
    NO_SAMPLE_TIMEOUT,
    PASSED,
    SCHEMA_VERSION,
    STALE_ODOMETRY,
    InvalidConfigurationError,
    TrialConfiguration,
    TrialResult,
    format_result,
    metrics_from_evaluation,
)


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


class OdometryAccuracyTrial:
    def __init__(self, configuration: TrialConfiguration) -> None:
        self._configuration = configuration
        self._samples: list[OdometrySample] = []
        self._rejected_count = 0
        self._status = ""
        self._evaluation: OdometryAccuracyEvaluation | None = None
        self._live = LiveSampleSummary()
        self._first_receipt_sec: float | None = None

    @property
    def started(self) -> bool:
        return bool(self._samples)

    @property
    def finished(self) -> bool:
        return bool(self._status)

    def receive(self, message: RosOdometry) -> None:
        if self.finished:
            return
        if self._first_receipt_sec is None:
            self._first_receipt_sec = time.monotonic()
        try:
            sample = sample_from_odometry(message)
        except OdometryValidationError as error:
            self._reject(error.issue.value)
            return
        self._receive_sample(sample)

    def finish(self, status: str) -> None:
        if not self.finished:
            self._status = status

    def result(self) -> TrialResult:
        first = self._samples[0] if self._samples else None
        last = self._samples[-1] if self._samples else None
        duration_sec = None if first is None or last is None else (last.stamp_ns - first.stamp_ns) * 1e-9
        return {
            'schema_version': SCHEMA_VERSION,
            'status': self._status,
            'trial': self._configuration.mode.value,
            'interpretation': INTERPRETATION,
            'input_topic': self._configuration.odom_topic,
            'frame_id': first.frame_id if first is not None else None,
            'start_stamp_ns': first.stamp_ns if first is not None else None,
            'end_stamp_ns': last.stamp_ns if last is not None else None,
            'duration_sec': duration_sec,
            'sample_count': len(self._samples),
            'rejected_count': self._rejected_count,
            'metrics': metrics_from_evaluation(self._evaluation) if self._evaluation is not None else None,
            'dx_m': self._live.dx_m,
            'dy_m': self._live.dy_m,
            'dz_m': self._live.dz_m,
            'xy_m': self._live.xy_m,
            'three_d_m': self._live.three_d_m,
            'health': self._live.health,
            'age_sec': self._live.age_sec,
        }

    def _receive_sample(self, sample: OdometrySample) -> None:
        if not self._samples:
            self._samples.append(sample)
            self._live = LiveSampleSummary(frame_id=sample.frame_id, age_sec=0.0, health='live')
            self._emit_live(sample)
            return
        first = self._samples[0]
        try:
            evaluation = evaluate_odometry_accuracy(self._configuration.mode, (*self._samples, sample), self._configuration.known_distance_m)
        except OdometryValidationError as error:
            self._reject(error.issue.value)
            return
        self._samples.append(sample)
        receipt_age_sec = 0.0 if self._first_receipt_sec is None else max(0.0, time.monotonic() - self._first_receipt_sec)
        self._live = live_sample_summary(first, sample, receipt_age_sec)
        self._emit_live(sample)
        if (sample.stamp_ns - first.stamp_ns) * 1e-9 >= self._configuration.duration_sec:
            if len(self._samples) < self._configuration.min_samples:
                self.finish(INSUFFICIENT_SAMPLES)
                self._evaluation = None
            else:
                self._evaluation = evaluation
                self.finish(PASSED)

    def _reject(self, status: str) -> None:
        self._rejected_count += 1
        if self.started:
            self.finish(status)

    def _emit_live(self, sample: OdometrySample) -> None:
        print(
            'ODOMETRY_ACCURACY_LIVE '
            f'dx_m={self._live.dx_m:.6f} dy_m={self._live.dy_m:.6f} dz_m={self._live.dz_m:.6f} '
            f'xy_m={self._live.xy_m:.6f} three_d_m={self._live.three_d_m:.6f} '
            f'frame={sample.frame_id} age_sec={self._live.age_sec:.6f} health={self._live.health}',
            flush=True,
        )


def sample_from_odometry(message: RosOdometry) -> OdometrySample:
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


class _ArgumentParser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        raise InvalidConfigurationError(message)


def configuration_from_argv(argv: Sequence[str]) -> TrialConfiguration:
    parser = _ArgumentParser(prog='odometry-accuracy-demo')
    parser.add_argument('--mode', choices=tuple(mode.value for mode in OdometryEvaluationMode), default='stationary')
    parser.add_argument('--odom-topic', default=DEFAULT_ODOM_TOPIC)
    parser.add_argument('--duration-sec', type=float, default=30.0)
    parser.add_argument('--known-distance-m', type=float)
    parser.add_argument('--start-timeout-sec', type=float, default=10.0)
    parser.add_argument('--stale-timeout-sec', type=float, default=0.5)
    parser.add_argument('--min-samples', type=int, default=2)
    arguments = parser.parse_args(argv)
    return TrialConfiguration(
        mode=OdometryEvaluationMode(arguments.mode),
        odom_topic=arguments.odom_topic,
        duration_sec=arguments.duration_sec,
        known_distance_m=arguments.known_distance_m,
        start_timeout_sec=arguments.start_timeout_sec,
        stale_timeout_sec=arguments.stale_timeout_sec,
        min_samples=arguments.min_samples,
    )


def run_ros_trial(configuration: TrialConfiguration) -> TrialResult:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    rclpy.init(args=[])
    node = Node('odometry_accuracy_demo')
    trial = OdometryAccuracyTrial(configuration)
    last_receipt_sec: float | None = None

    def receive(message: Odometry) -> None:
        nonlocal last_receipt_sec
        last_receipt_sec = time.monotonic()
        trial.receive(message)

    node.create_subscription(Odometry, configuration.odom_topic, receive, 10)
    started_sec = time.monotonic()
    try:
        while rclpy.ok() and not trial.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
            now_sec = time.monotonic()
            if not trial.started:
                if now_sec - started_sec >= configuration.start_timeout_sec:
                    trial.finish(NO_SAMPLE_TIMEOUT)
            elif last_receipt_sec is not None and now_sec - last_receipt_sec >= configuration.stale_timeout_sec:
                trial.finish(STALE_ODOMETRY)
        if not trial.finished:
            trial.finish(STALE_ODOMETRY if trial.started else NO_SAMPLE_TIMEOUT)
        return trial.result()
    except KeyboardInterrupt:
        trial.finish(INTERRUPTED)
        return trial.result()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _configuration_failure(status: str) -> int:
    fallback = OdometryAccuracyTrial(TrialConfiguration(OdometryEvaluationMode.STATIONARY))
    fallback.finish(status)
    print(format_result(fallback.result()))
    return 1


def main(argv: Sequence[str] | None = None) -> int:
    arguments = sys.argv[1:] if argv is None else argv
    try:
        configuration = configuration_from_argv(arguments)
    except OdometryValidationError as error:
        return _configuration_failure(error.issue.value)
    except InvalidConfigurationError:
        return _configuration_failure(INVALID_CONFIGURATION)
    result = run_ros_trial(configuration)
    print(format_result(result))
    return 0 if result['status'] == PASSED else 1


if __name__ == '__main__':
    raise SystemExit(main())
