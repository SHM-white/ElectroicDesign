"""Bounded CPU usage under deterministic synthetic load."""

from __future__ import annotations

import time
from dataclasses import dataclass

from ed_uav_verification.clock import VirtualMonotonicClock
from ed_uav_verification.model import NANOSECONDS_PER_SECOND, ScenarioConfig, Stream
from ed_uav_verification.scenario import DeterministicScenario


@dataclass(frozen=True, slots=True)
class CpuBoundingError(Exception):
    """Raised when virtual-time processing exceeds the declared real-time budget."""

    latency_ns: int
    bound_ns: int

    def __str__(self) -> str:
        return f"processing latency {self.latency_ns} ns exceeds bound {self.bound_ns} ns"


def _measure_tick_latency(ticks: int = 200, rate_hz: int = 20) -> int:
    """Measure the wall-clock time consumed by one short deterministic replay batch."""
    config = ScenarioConfig(seed=41, duration_seconds=ticks // rate_hz, rate_hz=rate_hz)
    started = time.perf_counter_ns()
    DeterministicScenario(config).run()
    elapsed_ns = time.perf_counter_ns() - started
    return elapsed_ns // ticks


def test_cpu_bounded_under_reference_workload() -> None:
    """Given a 40-tick reference workload, when replayed, then per-tick wall time is bounded."""
    RATE_HZ = 20
    TICK_LATENCY_BOUND_NS = 50_000_000  # 50 ms per virtual tick is an extreme safety bound

    ticks = 40
    config = ScenarioConfig(seed=41, duration_seconds=ticks // RATE_HZ, rate_hz=RATE_HZ)

    started = time.perf_counter_ns()
    report = DeterministicScenario(config).run()
    elapsed_ns = time.perf_counter_ns() - started

    assert report.completed
    per_tick_ns = elapsed_ns // report.tick_count
    assert per_tick_ns < TICK_LATENCY_BOUND_NS, (
        f"per-tick wall latency {per_tick_ns} ns exceeds {TICK_LATENCY_BOUND_NS} ns bound"
    )


def test_virtual_clock_maintains_real_time_factor() -> None:
    """Given a virtual monotonic clock, when a scenario advances, then the RTF is >= 1.0."""
    DURATION_S = 2
    RATE_HZ = 20

    config = ScenarioConfig(seed=42, duration_seconds=DURATION_S, rate_hz=RATE_HZ)

    wall_start_ns = time.perf_counter_ns()
    report = DeterministicScenario(config).run()
    wall_elapsed_ns = time.perf_counter_ns() - wall_start_ns

    simulated_ns = report.simulated_duration_ns
    rtf = simulated_ns / max(wall_elapsed_ns, 1)
    assert rtf >= 1.0, f"real-time factor {rtf:.3f} is below 1.0"


def test_cpu_workload_scales_linearly_with_ticks() -> None:
    """Given increasing tick counts, when replayed, then wall time scales linearly in tick budget."""
    RATE_HZ = 20
    tick_counts = (20, 40, 80, 160)
    elapsed_per_tick: dict[int, int] = {}

    for ticks in tick_counts:
        config = ScenarioConfig(seed=43, duration_seconds=ticks // RATE_HZ, rate_hz=RATE_HZ)
        started = time.perf_counter_ns()
        DeterministicScenario(config).run()
        elapsed = time.perf_counter_ns() - started
        elapsed_per_tick[ticks] = elapsed // ticks

    # Per-tick cost must stay within 5x of the smallest workload (linear scaling)
    baseline = elapsed_per_tick[tick_counts[0]]
    for ticks in tick_counts:
        ratio = elapsed_per_tick[ticks] / max(baseline, 1)
        assert ratio < 5.0, f"per-tick cost at {ticks} ticks is {ratio:.2f}x baseline"


def test_concurrent_scenario_replays_never_deadlock() -> None:
    """Given concurrent deterministic replay instances, when all run, then every one completes."""
    from concurrent.futures import ThreadPoolExecutor

    configs = [
        ScenarioConfig(seed=s, duration_seconds=1, rate_hz=20)
        for s in range(44, 52)
    ]

    def _run(config: ScenarioConfig) -> bool:
        return DeterministicScenario(config).run().completed

    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(_run, configs))

    assert all(results)
    assert len(results) == len(configs)


def test_safety_event_p99_latency() -> None:
    """Given a replay, when measuring per-tick wall latency, then p99 is <= 0.10s."""
    RATE_HZ = 20
    SAMPLES = 100

    latency_samples: list[int] = []

    # Measure per-tick latency by running 1-second replays repeatedly
    for i in range(SAMPLES):
        sub = ScenarioConfig(seed=45 + i, duration_seconds=1, rate_hz=RATE_HZ)
        started = time.perf_counter_ns()
        DeterministicScenario(sub).run()
        latency_samples.append(time.perf_counter_ns() - started)

    latency_samples.sort()
    p99_index = int(len(latency_samples) * 0.99)
    p99_ns = latency_samples[min(p99_index, len(latency_samples) - 1)]

    BOUND_NS = 100_000_000  # 0.10s
    assert p99_ns <= BOUND_NS, f"p99 safety-event latency {p99_ns / 1e6:.3f}ms exceeds {BOUND_NS / 1e6:.0f}ms"
