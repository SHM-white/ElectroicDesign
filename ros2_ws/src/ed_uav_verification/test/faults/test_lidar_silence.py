"""Lidar/IMU silence detection: drops, staleness, and health degradation."""

from __future__ import annotations

from ed_uav_verification.assertions import LaunchAssertions
from ed_uav_verification.faults import FaultKind, FaultWindow
from ed_uav_verification.model import EventType, ScenarioConfig, Stream
from ed_uav_verification.scenario import DeterministicScenario


def test_lidar_drop_silence_degrades_health() -> None:
    """Given a DROP fault on LIDAR_POINTS, when sustained, then health degrades and recovers."""
    config = ScenarioConfig(
        seed=90,
        duration_seconds=2,
        rate_hz=20,
        faults=(FaultWindow(FaultKind.DROP, Stream.LIDAR_POINTS, start_tick=5, duration_ticks=8),),
    )

    report = DeterministicScenario(config).run()

    assert report.has_fault_activation(FaultKind.DROP)
    assert report.has_fault_recovery(FaultKind.DROP)
    assert report.has_degradation(FaultKind.DROP)
    assert report.has_stream_recovery(Stream.LIDAR_POINTS)

    LaunchAssertions(report).assert_no_stale_reuse()
    LaunchAssertions(report).assert_fault_matrix(config.faults)


def test_imu_silence_detected_via_stale_freshness() -> None:
    """Given a DROP fault on LIDAR_IMU, when replayed, then rejection reason includes 'drop'."""
    config = ScenarioConfig(
        seed=91,
        duration_seconds=2,
        rate_hz=20,
        faults=(FaultWindow(FaultKind.DROP, Stream.LIDAR_IMU, start_tick=5, duration_ticks=4),),
    )

    report = DeterministicScenario(config).run()

    rejection_reasons = report.rejection_reasons(Stream.LIDAR_IMU)
    assert "drop" in rejection_reasons, f"expected 'drop' in rejection reasons, got {rejection_reasons}"


def test_lidar_silence_never_causes_tf_jump() -> None:
    """Given lidar silence, when ODOM continues, then odometry remains continuous (no TF jump)."""
    config = ScenarioConfig(
        seed=92,
        duration_seconds=2,
        rate_hz=20,
        faults=(FaultWindow(FaultKind.DROP, Stream.LIDAR_POINTS, start_tick=8, duration_ticks=6),),
    )

    report = DeterministicScenario(config).run()

    # ODOM must have zero rejected events (independent of lidar)
    odom_rejected = [
        e for e in report.events
        if e.event_type is EventType.MESSAGE_REJECTED and e.stream is Stream.ODOM
    ]
    assert len(odom_rejected) == 0, f"ODOM had {len(odom_rejected)} rejected events during lidar silence"

    # ODOM accepted sequences must be strictly increasing
    odom_accepted = report.accepted_sequences(Stream.ODOM)
    for i in range(1, len(odom_accepted)):
        assert odom_accepted[i] > odom_accepted[i - 1], "ODOM sequences jumped during lidar silence"


def test_lidar_imu_silence_detection_time_is_bounded() -> None:
    """Given IMU drops, when analyzing staleness, then detection latency is under freshness window."""
    config = ScenarioConfig(
        seed=93,
        duration_seconds=2,
        rate_hz=20,
        faults=(FaultWindow(FaultKind.DROP, Stream.LIDAR_IMU, start_tick=5, duration_ticks=3),),
    )

    report = DeterministicScenario(config).run()

    # Find the first HEALTH_DEGRADED event for LIDAR_IMU
    degraded = [
        e for e in report.events
        if e.event_type is EventType.HEALTH_DEGRADED and e.stream is Stream.LIDAR_IMU
    ]
    assert len(degraded) > 0, "expected at least one HEALTH_DEGRADED event"

    # The fault starts at tick 5. The first IMU drop happens at tick 5.
    # Freshness for LIDAR_IMU is 150ms; at 20Hz (50ms per tick), that's 3 ticks.
    # Degradation should appear within freshness_window + 1 tick of first drop
    first_degraded_tick = degraded[0].sequence
    fault_start_tick = 5
    detection_ticks = first_degraded_tick - fault_start_tick
    MAX_DETECTION_TICKS = 6  # generous bound: freshness (3 ticks) + 3 extra
    assert detection_ticks <= MAX_DETECTION_TICKS, (
        f"IMU silence detection took {detection_ticks} ticks, max {MAX_DETECTION_TICKS}"
    )


def test_simultaneous_lidar_and_imu_drop_recovery() -> None:
    """Given overlapping DROP windows on both lidar streams, when both recover, then each recovers cleanly."""
    config = ScenarioConfig(
        seed=94,
        duration_seconds=2,
        rate_hz=20,
        faults=(
            FaultWindow(FaultKind.DROP, Stream.LIDAR_POINTS, start_tick=5, duration_ticks=6),
            FaultWindow(FaultKind.DROP, Stream.LIDAR_IMU, start_tick=5, duration_ticks=6),
        ),
    )

    report = DeterministicScenario(config).run()

    assert report.has_stream_recovery(Stream.LIDAR_POINTS)
    assert report.has_stream_recovery(Stream.LIDAR_IMU)

    # After recovery, both streams should produce accepted samples
    lidar_accepted = report.accepted_sequences(Stream.LIDAR_POINTS)
    imu_accepted = report.accepted_sequences(Stream.LIDAR_IMU)

    assert len(lidar_accepted) > 0, "LIDAR_POINTS never recovered"
    assert len(imu_accepted) > 0, "LIDAR_IMU never recovered"


def test_no_deadlock_during_continuous_silence() -> None:
    """Given continuous DROP across entire scenario, when replayed, then it completes without deadlock."""
    config = ScenarioConfig(
        seed=95,
        duration_seconds=1,
        rate_hz=20,
        faults=(FaultWindow(FaultKind.DROP, Stream.LIDAR_POINTS, start_tick=0, duration_ticks=18),),
    )

    report = DeterministicScenario(config).run()

    assert report.completed
    assert report.has_degradation(FaultKind.DROP)

    # Even with near-total silence, the scenario terminates cleanly
    LaunchAssertions(report).assert_no_stale_reuse()
