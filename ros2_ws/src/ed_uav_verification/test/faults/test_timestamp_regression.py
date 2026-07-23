"""Non-monotonic timestamp regression detection and rejection."""

from __future__ import annotations

from ed_uav_verification.assertions import LaunchAssertions
from ed_uav_verification.faults import FaultKind, FaultWindow
from ed_uav_verification.model import EventType, ScenarioConfig, Stream
from ed_uav_verification.scenario import DeterministicScenario


def test_time_regression_is_detected_and_rejected() -> None:
    """Given a TIME_REGRESSION fault, when timestamps go backward, then they are rejected."""
    config = ScenarioConfig(
        seed=70,
        duration_seconds=2,
        rate_hz=20,
        faults=(FaultWindow(FaultKind.TIME_REGRESSION, Stream.LIDAR_IMU, start_tick=5, duration_ticks=4),),
    )

    report = DeterministicScenario(config).run()

    assert report.has_fault_activation(FaultKind.TIME_REGRESSION)
    assert report.has_fault_recovery(FaultKind.TIME_REGRESSION)
    assert report.has_degradation(FaultKind.TIME_REGRESSION)
    assert report.has_stream_recovery(Stream.LIDAR_IMU)

    LaunchAssertions(report).assert_no_stale_reuse()
    LaunchAssertions(report).assert_fault_matrix(config.faults)


def test_multiple_regressions_on_independent_streams_never_overlap() -> None:
    """Given two concurrent regressions on independent streams, when replayed, then both degrade independently."""
    config = ScenarioConfig(
        seed=71,
        duration_seconds=2,
        rate_hz=20,
        faults=(
            FaultWindow(FaultKind.TIME_REGRESSION, Stream.LIDAR_IMU, start_tick=4, duration_ticks=4),
            FaultWindow(FaultKind.TIME_REGRESSION, Stream.LIDAR_POINTS, start_tick=8, duration_ticks=4),
        ),
    )

    report = DeterministicScenario(config).run()

    # Both streams independently detect regression and recover
    assert report.has_stream_recovery(Stream.LIDAR_IMU)
    assert report.has_stream_recovery(Stream.LIDAR_POINTS)

    # No stale reuse across either stream
    LaunchAssertions(report).assert_no_stale_reuse()


def test_time_regression_never_produces_nondecreasing_accepted_sequences() -> None:
    """Given time regression, when replayed, then every accepted sequence is strictly increasing per-stream."""
    config = ScenarioConfig(
        seed=72,
        duration_seconds=2,
        rate_hz=20,
        faults=(FaultWindow(FaultKind.TIME_REGRESSION, Stream.ODOM, start_tick=6, duration_ticks=3),),
    )

    report = DeterministicScenario(config).run()
    accepted = report.accepted_sequences(Stream.ODOM)

    # Accepted sequences for a single stream must be monotonically increasing
    for i in range(1, len(accepted)):
        assert accepted[i] > accepted[i - 1], (
            f"non-monotonic accepted sequence at index {i}: {accepted[i-1]} -> {accepted[i]}"
        )


def test_acquisition_timestamps_are_monotonic_in_accepted_events() -> None:
    """Given a clean replay, when examining accepted events, then acquisition timestamps never regress."""
    config = ScenarioConfig(seed=73, duration_seconds=2, rate_hz=20)
    report = DeterministicScenario(config).run()

    per_stream_last_ns: dict[Stream, int] = {}
    for event in report.events:
        if event.event_type is EventType.SAMPLE and event.accepted:
            last = per_stream_last_ns.get(event.stream)
            if last is not None:
                assert event.acquisition_time_ns >= last, (
                    f"time regression in {event.stream.value}: "
                    f"sequence {event.sequence} stamp {event.acquisition_time_ns} < {last}"
                )
            per_stream_last_ns[event.stream] = event.acquisition_time_ns


def test_regression_survivor_stream_is_byte_identical_to_clean_replay() -> None:
    """Given one regressing stream among several, when replayed, then survivor streams match clean replay."""
    # Clean replay
    clean_config = ScenarioConfig(seed=74, duration_seconds=1, rate_hz=20)
    clean = DeterministicScenario(clean_config).run()

    # Regression on one unrelated stream
    regression_config = ScenarioConfig(
        seed=74,
        duration_seconds=1,
        rate_hz=20,
        faults=(FaultWindow(FaultKind.TIME_REGRESSION, Stream.GPIO, start_tick=5, duration_ticks=4),),
    )
    regression = DeterministicScenario(regression_config).run()

    # Extract accepted event digests per-stream for unaffected streams
    def _accepted_digests(report, stream: Stream) -> tuple[str, ...]:
        return tuple(
            e.payload_sha256
            for e in report.events
            if e.event_type is EventType.SAMPLE and e.stream is stream and e.accepted and e.payload_sha256
        )

    # LIDAR_POINTS shouldn't be affected by GPIO regression
    unaffected = Stream.LIDAR_POINTS
    assert _accepted_digests(clean, unaffected) == _accepted_digests(regression, unaffected)


def test_negative_timestamp_delta_is_bounded() -> None:
    """Given TIME_REGRESSION on an IMU stream, when examining rejected events, then the delta magnitude is bounded."""
    config = ScenarioConfig(
        seed=75,
        duration_seconds=2,
        rate_hz=20,
        faults=(FaultWindow(FaultKind.TIME_REGRESSION, Stream.LIDAR_IMU, start_tick=5, duration_ticks=4),),
    )

    report = DeterministicScenario(config).run()

    # Collect acquisition timestamps for rejected LIDAR_IMU events
    rejected_stamps = [
        e.acquisition_time_ns
        for e in report.events
        if e.event_type is EventType.MESSAGE_REJECTED and e.stream is Stream.LIDAR_IMU
    ]

    # The rejected stamps should all be from the regression window
    # Regression offset is FAULT_LATENCY_NS = 1_000_000_000 (1 second)
    # Each rejected stamp should be within that bound of the simulated time
    assert len(rejected_stamps) > 0, "expected rejected events from time regression"

    # Bounded: no stamp should regress by more than the fault latency bound
    MAX_REGRESSION_NS = 2_000_000_000  # generous 2-second bound
    for stamp in rejected_stamps:
        assert stamp >= config.start_time_ns - MAX_REGRESSION_NS, (
            f"rejected stamp {stamp} regresses beyond {MAX_REGRESSION_NS} ns bound"
        )
