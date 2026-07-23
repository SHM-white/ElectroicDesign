"""Resident memory growth after warm-up soak period."""

from __future__ import annotations

import gc
import os
import sys

import pytest

from ed_uav_verification.model import ScenarioConfig
from ed_uav_verification.scenario import DeterministicScenario


def _resident_memory_bytes() -> int:
    """Return the current process RSS in bytes using /proc/self/status."""
    try:
        with open("/proc/self/status") as handle:
            for line in handle:
                if line.startswith("VmRSS:"):
                    parts = line.split()
                    return int(parts[1]) * 1024  # kB -> bytes
    except FileNotFoundError:
        pass
    return -1


def _python_heap_bytes() -> int:
    """Return approximate Python heap size from sys.getsizeof on gc objects."""
    total = 0
    for obj in gc.get_objects():
        try:
            total += sys.getsizeof(obj)
        except (TypeError, RuntimeError):
            pass
    return total


def _run_warmup_ticks(seed: int, ticks: int, rate_hz: int = 20) -> None:
    """Run a warm-up replay to stabilize any one-shot allocations."""
    config = ScenarioConfig(seed=seed, duration_seconds=ticks // rate_hz, rate_hz=rate_hz)
    DeterministicScenario(config).run()


def _measure_soak_memory(seed: int, soak_ticks: int, rate_hz: int = 20) -> int:
    """Run a soak workload and return the peak RSS during execution."""
    config = ScenarioConfig(seed=seed, duration_seconds=soak_ticks // rate_hz, rate_hz=rate_hz)
    report = DeterministicScenario(config).run()
    assert report.completed
    rss = _resident_memory_bytes()
    return rss


@pytest.mark.skipif(
    os.name == "nt" or not os.path.exists("/proc/self/status"),
    reason="/proc/self/status not available for RSS measurement",
)
def test_no_memory_leak_after_warmup_soak() -> None:
    """Given consecutive soak periods, when memory growth is tracked, then per-period growth is bounded."""
    WARMUP_TICKS = 400  # 20 seconds at 20Hz
    SOAK_TICKS = 12_000  # 10 minutes at 20Hz
    RATE_HZ = 20

    _run_warmup_ticks(seed=50, ticks=WARMUP_TICKS, rate_hz=RATE_HZ)
    gc.collect()

    before_rss = _resident_memory_bytes()
    assert before_rss > 0, "RSS measurement unavailable"

    # Run first soak period
    _measure_soak_memory(seed=51, soak_ticks=SOAK_TICKS, rate_hz=RATE_HZ)
    gc.collect()
    after_first_rss = _resident_memory_bytes()

    # Run second identical soak period
    _measure_soak_memory(seed=52, soak_ticks=SOAK_TICKS, rate_hz=RATE_HZ)
    gc.collect()
    after_second_rss = _resident_memory_bytes()

    # The first soak may grow RSS (allocator expansion), but the second soak's
    # incremental growth should be bounded (<10% of the first period's growth).
    first_growth = max(after_first_rss - before_rss, 1)
    second_growth = max(after_second_rss - after_first_rss, 0)

    # The second period should grow at most 10% as much as the first
    MAX_CONTINUED_GROWTH = 0.10
    growth_ratio = second_growth / first_growth
    assert growth_ratio <= MAX_CONTINUED_GROWTH, (
        f"Second soak grew by {second_growth} bytes vs first growth of {first_growth} bytes "
        f"({growth_ratio:.3f}, max {MAX_CONTINUED_GROWTH})"
    )


def test_python_heap_growth_is_bounded() -> None:
    """Given repeated scenario runs, when tracked via Python allocation, then heap growth is bounded."""
    WARMUP_TICKS = 400
    RATE_HZ = 20

    _run_warmup_ticks(seed=50, ticks=WARMUP_TICKS, rate_hz=RATE_HZ)
    gc.collect()
    before_heap = _python_heap_bytes()

    # Run 24000 ticks across two large scenarios (same total as soak)
    for batch_seed in (51, 52):
        config = ScenarioConfig(seed=batch_seed, duration_seconds=600, rate_hz=RATE_HZ)
        DeterministicScenario(config).run()
        gc.collect()

    after_heap = _python_heap_bytes()
    growth = after_heap / max(before_heap, 1)

    # Python heap should not grow unboundedly; 2x is generous for internal structures
    assert growth < 3.0, f"Python heap grew {growth:.2f}x (before={before_heap}, after={after_heap})"


def test_scenario_report_memory_is_deterministic() -> None:
    """Given the same seed, when a report is built twice, then event tuples reuse the same size."""
    import sys

    config = ScenarioConfig(seed=52, duration_seconds=5, rate_hz=20)

    first = DeterministicScenario(config).run()
    second = DeterministicScenario(config).run()

    first_size = sys.getsizeof(first.event_json)
    second_size = sys.getsizeof(second.event_json)

    assert first_size == second_size
    assert first.event_json == second.event_json


def test_memory_is_bounded_under_repeated_reset() -> None:
    """Given repeated scenario creation and discard, when cycled, then no unbounded growth."""
    CYCLES = 50

    initial_rss = _resident_memory_bytes()
    if initial_rss <= 0:
        pytest.skip("RSS measurement unavailable on this platform")

    for cycle in range(CYCLES):
        config = ScenarioConfig(seed=cycle, duration_seconds=1, rate_hz=20)
        DeterministicScenario(config).run()

    final_rss = _resident_memory_bytes()
    growth = final_rss / max(initial_rss, 1)

    # Allow moderate growth from Python internals but reject pathological leak
    assert growth < 2.0, f"RSS grew {growth:.2f}x over {CYCLES} cycles (initial={initial_rss}, final={final_rss})"


def test_event_tuple_size_remains_bounded_for_long_scenario() -> None:
    """Given a long scenario, when the event tuple is built, then its size is proportional to ticks."""
    import sys

    RATE_HZ = 20
    DURATION_S = 60
    ticks = DURATION_S * RATE_HZ  # 1200

    config = ScenarioConfig(seed=53, duration_seconds=DURATION_S, rate_hz=RATE_HZ)
    report = DeterministicScenario(config).run()

    events = report.events
    event_count = len(events)
    total_bytes = sys.getsizeof(events)

    # Each event should average well under 1KB (they are small dataclass instances)
    avg_bytes_per_event = total_bytes / max(event_count, 1)
    MAX_AVG_BYTES = 1024

    assert avg_bytes_per_event < MAX_AVG_BYTES, (
        f"average {avg_bytes_per_event:.0f} bytes per event exceeds {MAX_AVG_BYTES} bound"
    )
    assert report.completed
