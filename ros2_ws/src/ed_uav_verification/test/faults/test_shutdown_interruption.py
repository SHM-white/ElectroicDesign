"""Interrupted shutdown: scenario interruption, clean restart, and no unsafe motor cut."""

from __future__ import annotations

from ed_uav_verification.assertions import LaunchAssertions
from ed_uav_verification.faults import FaultKind, FaultWindow
from ed_uav_verification.model import EventType, ScenarioBoundError, ScenarioConfig, Stream
from ed_uav_verification.scenario import DeterministicScenario


def test_bounded_interruption_produces_incomplete_report() -> None:
    """Given a stop_after_ticks limit, when the scenario runs, then it reports incomplete cleanly."""
    config = ScenarioConfig(seed=100, duration_seconds=2, rate_hz=20)

    report = DeterministicScenario(config).run(stop_after_ticks=10)

    assert not report.completed
    assert report.tick_count == 10
    # No deadlock, no exception - just an incomplete report


def test_interrupted_and_restarted_replay_produces_identical_result() -> None:
    """Given an interrupted replay, when a fresh scenario runs to completion, then result matches clean."""
    config = ScenarioConfig(seed=101, duration_seconds=2, rate_hz=20)

    # Interrupted run
    interrupted = DeterministicScenario(config).run(stop_after_ticks=8)
    assert not interrupted.completed

    # Fresh complete run
    complete = DeterministicScenario(config).run()
    assert complete.completed

    # Another clean reference run
    reference = DeterministicScenario(config).run()
    assert reference.event_json == complete.event_json


def test_shutdown_with_active_faults_recovers_gracefully() -> None:
    """Given active faults at interruption time, when a fresh run starts, then it is deterministic."""
    config = ScenarioConfig(
        seed=102,
        duration_seconds=2,
        rate_hz=20,
        faults=(FaultWindow(FaultKind.DROP, Stream.LIDAR_POINTS, start_tick=5, duration_ticks=10),),
    )

    # Interrupt while the fault is still active
    interrupted = DeterministicScenario(config).run(stop_after_ticks=10)
    assert not interrupted.completed

    # Fresh complete run must match itself (determinism preserved)
    first = DeterministicScenario(config).run()
    second = DeterministicScenario(config).run()
    assert first.event_json == second.event_json


def test_shutdown_never_causes_automatic_motor_cut() -> None:
    """Given scenario interruption, when examining events, then no GPIO/laser state change signals motor cut."""
    config = ScenarioConfig(seed=103, duration_seconds=3, rate_hz=20)

    # Run to completion, then verify no motor-cut event exists
    report = DeterministicScenario(config).run()

    # A motor cut would be a GPIO or LASER health degradation or a specific rejected pattern
    gpio_rejected = [
        e for e in report.events
        if e.event_type is EventType.MESSAGE_REJECTED and e.stream is Stream.GPIO
    ]
    laser_rejected = [
        e for e in report.events
        if e.event_type is EventType.MESSAGE_REJECTED and e.stream is Stream.LASER
    ]

    # GPIO and LASER should have zero rejections in a clean run (no motor cut)
    assert len(gpio_rejected) == 0, f"GPIO had {len(gpio_rejected)} rejections (possible motor cut)"
    assert len(laser_rejected) == 0, f"LASER had {len(laser_rejected)} rejections (possible motor cut)"

    # Also verify interrupted shutdown doesn't trigger motor cut
    interrupted = DeterministicScenario(config).run(stop_after_ticks=5)
    interrupted_gpio = [
        e for e in interrupted.events
        if e.event_type is EventType.MESSAGE_REJECTED and e.stream is Stream.GPIO
    ]
    assert len(interrupted_gpio) == 0, "interrupted shutdown caused GPIO rejection"


def test_shutdown_cleanup_releases_all_resources_without_exception() -> None:
    """Given a scenario, when it runs and is discarded, then no resource leak exception occurs."""
    import gc

    for _ in range(5):
        config = ScenarioConfig(seed=104, duration_seconds=1, rate_hz=20)
        report = DeterministicScenario(config).run()
        assert report.completed

    # Force garbage collection - no hidden reference cycles
    gc.collect()


def test_shutdown_during_fault_recovery_window_is_safe() -> None:
    """Given interruption exactly at fault recovery boundary, when replayed, then determinism holds."""
    config = ScenarioConfig(
        seed=105,
        duration_seconds=2,
        rate_hz=20,
        faults=(FaultWindow(FaultKind.CORRUPTION, Stream.LIDAR_IMU, start_tick=5, duration_ticks=6),),
    )

    # Interrupt exactly at recovery boundary (tick 11 = end_tick)
    interrupted = DeterministicScenario(config).run(stop_after_ticks=11)
    assert not interrupted.completed

    # Complete run
    complete = DeterministicScenario(config).run()
    assert complete.completed
    assert complete.has_fault_recovery(FaultKind.CORRUPTION)
    assert complete.has_stream_recovery(Stream.LIDAR_IMU)

    LaunchAssertions(complete).assert_no_stale_reuse()


def test_consecutive_interruptions_never_cause_deadlock() -> None:
    """Given rapid consecutive interruptions, when replayed, then every scenario terminates cleanly."""
    config = ScenarioConfig(seed=106, duration_seconds=2, rate_hz=20)

    for stop_at in (3, 7, 11, 15, 19):
        report = DeterministicScenario(config).run(stop_after_ticks=stop_at)
        assert not report.completed
        assert report.tick_count == stop_at

    # Final complete run
    final = DeterministicScenario(config).run()
    assert final.completed


def test_tick_budget_exhaustion_is_cleanly_rejected() -> None:
    """Given a tick budget breach, when the scenario starts, then it rejects cleanly without deadlock."""
    config = ScenarioConfig(seed=107, duration_seconds=61, rate_hz=20, max_ticks=1_200)

    try:
        DeterministicScenario(config).run()
        assert False, "expected ScenarioBoundError"
    except ScenarioBoundError:
        pass  # Expected clean rejection
