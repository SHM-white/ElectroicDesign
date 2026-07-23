"""Deterministic fake dual-camera surface for non-hardware transport acceptance."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass

from .health import CameraHealth, HealthReport
from .model import CameraRole

NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True, slots=True)
class FakeCliError(Exception):
    """Raised for invalid fake-image scenario arguments."""

    detail: str

    def __str__(self) -> str:
        return self.detail


@dataclass(frozen=True, slots=True)
class FakeRunConfig:
    """A deterministic non-hardware stream timeline measured in simulated seconds."""

    duration_seconds: int
    wide_unplug_at_seconds: int
    wide_reconnect_at_seconds: int
    restart_wide: bool
    inject_nonmonotonic_wide: bool


def simulate(config: FakeRunConfig) -> tuple[HealthReport, HealthReport, bool]:
    """Run independent narrow and wide simulated cameras without wall-clock sleeps."""
    _validate_config(config)
    duration_ns = config.duration_seconds * NANOSECONDS_PER_SECOND
    narrow_period_ns = NANOSECONDS_PER_SECOND // 20
    wide_period_ns = NANOSECONDS_PER_SECOND // 15
    narrow = _simulate_stream(CameraRole.NARROW, narrow_period_ns, duration_ns, None, None, False)
    wide = _simulate_stream(
        CameraRole.WIDE,
        wide_period_ns,
        duration_ns,
        config.wide_unplug_at_seconds * NANOSECONDS_PER_SECOND,
        config.wide_reconnect_at_seconds * NANOSECONDS_PER_SECOND,
        config.restart_wide,
    )
    if config.inject_nonmonotonic_wide:
        _inject_nonmonotonic_stamp(wide, duration_ns)
    narrow_report = narrow.snapshot(duration_ns, stale_after_ns=narrow_period_ns * 2)
    wide_report = wide.snapshot(duration_ns, stale_after_ns=wide_period_ns * 2)
    narrow_survived = narrow.accepted_frames > config.wide_unplug_at_seconds * 20
    return narrow_report, wide_report, narrow_survived


def _simulate_stream(
    role: CameraRole,
    period_ns: int,
    duration_ns: int,
    unplug_at_ns: int | None,
    reconnect_at_ns: int | None,
    restart_after_reconnect: bool,
) -> CameraHealth:
    health = CameraHealth(role, period_ns)
    unplugged = False
    restarted = False
    frame_count = duration_ns // period_ns
    for frame_index in range(1, frame_count + 1):
        stamp_ns = frame_index * period_ns
        in_outage = (
            unplug_at_ns is not None
            and reconnect_at_ns is not None
            and unplug_at_ns <= stamp_ns < reconnect_at_ns
        )
        if in_outage:
            if not unplugged:
                health.mark_unplugged(stamp_ns)
                unplugged = True
            continue
        after_reconnect = reconnect_at_ns is not None and stamp_ns >= reconnect_at_ns
        if after_reconnect and restart_after_reconnect and not restarted:
            health.mark_restarted(stamp_ns)
            restarted = True
        health.record_frame(stamp_ns, stamp_ns)
    return health


def _inject_nonmonotonic_stamp(health: CameraHealth, observed_steady_ns: int) -> None:
    last_stamp = health.last_acquisition_stamp_ns
    if last_stamp is None:
        raise FakeCliError("cannot inject nonmonotonic stamp before a frame")
    health.record_frame(last_stamp - 1, observed_steady_ns)


def _validate_config(config: FakeRunConfig) -> None:
    if config.duration_seconds <= 0:
        raise FakeCliError("duration must be positive")
    if config.wide_unplug_at_seconds < 0:
        raise FakeCliError("wide unplug time must be non-negative")
    if config.wide_reconnect_at_seconds <= config.wide_unplug_at_seconds:
        raise FakeCliError("wide reconnect time must be after unplug time")
    if config.wide_reconnect_at_seconds > config.duration_seconds:
        raise FakeCliError("wide reconnect time must be inside duration")


def _report_as_json(
    config: FakeRunConfig,
    narrow: HealthReport,
    wide: HealthReport,
    narrow_survived: bool,
) -> str:
    report = {
        "outcome": "green" if wide.rejected_nonmonotonic_frames == 0 else "rejected",
        "simulated_duration_seconds": config.duration_seconds,
        "timestamp_provenance": "camera_acquisition_ros_time",
        "narrow_healthy_during_wide_unplug": narrow_survived,
        "narrow": _stream_as_json(narrow),
        "wide": _stream_as_json(wide),
    }
    return json.dumps(report, sort_keys=True) + "\n"


def _stream_as_json(report: HealthReport) -> dict[str, int | str]:
    role = report.role.value
    return {
        "topic": f"/camera/{role}/image_raw",
        "camera_info_topic": f"/camera/{role}/camera_info",
        "health": report.code.value,
        "accepted_frames": report.accepted_frames,
        "rejected_nonmonotonic_frames": report.rejected_nonmonotonic_frames,
        "inferred_drops": report.inferred_drops,
        "max_jitter_ns": report.max_jitter_ns,
        "restart_count": report.restart_count,
    }


def parse_arguments(argv: list[str]) -> FakeRunConfig:
    """Parse the CLI boundary into an immutable fake-camera scenario."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-seconds", type=int, required=True)
    parser.add_argument("--wide-unplug-at-seconds", type=int, required=True)
    parser.add_argument("--wide-reconnect-at-seconds", type=int, required=True)
    parser.add_argument("--restart-wide", action="store_true")
    parser.add_argument("--inject-nonmonotonic-wide", action="store_true")
    arguments = parser.parse_args(argv)
    return FakeRunConfig(
        arguments.duration_seconds,
        arguments.wide_unplug_at_seconds,
        arguments.wide_reconnect_at_seconds,
        arguments.restart_wide,
        arguments.inject_nonmonotonic_wide,
    )


def main(argv: list[str] | None = None) -> int:
    """Run the reproducible fake image-device surface and emit truthful JSON."""
    arguments = sys.argv[1:] if argv is None else argv
    try:
        config = parse_arguments(arguments)
        narrow, wide, narrow_survived = simulate(config)
    except FakeCliError as error:
        print(f"FAKE CAMERA: RED: {error}", file=sys.stderr)
        return 64
    output = _report_as_json(config, narrow, wide, narrow_survived)
    sys.stdout.write(output)
    return 0 if wide.rejected_nonmonotonic_frames == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
