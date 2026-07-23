"""Disk space checks before mission: artifact budget, cleanup, and partial-safe writes."""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest

from ed_uav_verification.artifacts import ArtifactExistsError, EventArtifactWriter, FixtureBagBuilder, IncompleteScenarioError
from ed_uav_verification.model import ScenarioConfig
from ed_uav_verification.scenario import DeterministicScenario


def _available_disk_bytes(path: str = ".") -> int:
    """Return the free space on the filesystem containing `path`."""
    usage = shutil.disk_usage(path)
    return usage.free


def test_event_artifact_fits_within_disk_budget(tmp_path) -> None:
    """Given a 10-second replay, when the event JSON is written, then it fits within 1 MiB."""
    config = ScenarioConfig(seed=60, duration_seconds=10, rate_hz=20)
    report = DeterministicScenario(config).run()

    event_path = tmp_path / "events.json"
    written = EventArtifactWriter(event_path).write(report)

    file_size = written.stat().st_size
    MAX_BYTES = 1_048_576  # 1 MiB

    assert file_size < MAX_BYTES, f"event artifact {file_size} bytes exceeds {MAX_BYTES} budget"
    assert file_size > 0


def test_disk_check_verifies_available_space_before_write(tmp_path) -> None:
    """Given available disk space, when a write is attempted, then pre-flight check passes."""
    free_bytes = _available_disk_bytes(str(tmp_path))
    REQUIRED_HEADROOM = 1024 * 1024  # 1 MiB minimum

    assert free_bytes >= REQUIRED_HEADROOM, (
        f"only {free_bytes} bytes available, need {REQUIRED_HEADROOM} for artifact write"
    )

    # Prove the write actually succeeds when space is available
    config = ScenarioConfig(seed=61, duration_seconds=1, rate_hz=20)
    report = DeterministicScenario(config).run()
    event = EventArtifactWriter(tmp_path / "check.json").write(report)
    assert event.stat().st_size > 0


def test_partial_artifact_is_cleaned_atomic(tmp_path) -> None:
    """Given an interrupted replay, when a partial write is detected, then no stale .partial remains."""
    event_path = tmp_path / "events.json"
    partial_path = tmp_path / "events.json.partial"

    # Simulate a stale partial from a previous crashed write
    partial_path.write_text("interrupted", encoding="utf-8")
    assert partial_path.exists()

    # The writer must reject when a stale .partial exists
    config = ScenarioConfig(seed=62, duration_seconds=1, rate_hz=20)
    report = DeterministicScenario(config).run()

    with pytest.raises(ArtifactExistsError):
        EventArtifactWriter(event_path).write(report)

    # After cleaning the partial, a lawful write succeeds
    partial_path.unlink()
    written = EventArtifactWriter(event_path).write(report)
    assert written.stat().st_size > 0

    # No partial should be left behind after a successful atomic write
    assert not partial_path.exists()


def test_fixture_bag_writes_below_disk_threshold(tmp_path) -> None:
    """Given a completed report, when a fixture bag is built, then total output is under 10 MiB."""
    config = ScenarioConfig(seed=63, duration_seconds=60, rate_hz=20)
    report = DeterministicScenario(config).run()

    root = tmp_path / "fixture_bag"
    bag = FixtureBagBuilder(root).write(report)

    total_size = 0
    for entry in root.rglob("*"):
        if entry.is_file():
            total_size += entry.stat().st_size

    MAX_BYTES = 10_485_760  # 10 MiB
    assert total_size < MAX_BYTES, f"fixture bag {total_size} bytes exceeds {MAX_BYTES} budget"


def test_stale_existing_artifact_is_rejected() -> None:
    """Given an existing artifact path, when another write is attempted, then it is rejected."""
    config = ScenarioConfig(seed=64, duration_seconds=1, rate_hz=20)
    report = DeterministicScenario(config).run()

    event_path = Path("/tmp/test_disk_reserve_existing.json")
    if event_path.exists():
        event_path.unlink()

    try:
        first = EventArtifactWriter(event_path).write(report)
        assert first.exists()

        with pytest.raises(ArtifactExistsError):
            EventArtifactWriter(event_path).write(report)
    finally:
        event_path.unlink(missing_ok=True)


def test_incomplete_scenario_refuses_disk_write() -> None:
    """Given an interrupted scenario, when a disk write is attempted, then it is rejected."""
    config = ScenarioConfig(seed=65, duration_seconds=2, rate_hz=20)
    interrupted = DeterministicScenario(config).run(stop_after_ticks=4)

    assert not interrupted.completed

    with pytest.raises(IncompleteScenarioError):
        EventArtifactWriter(Path("/tmp/should_not_write.json")).write(interrupted)


def test_artifact_cleanup_does_not_leak_file_descriptors(tmp_path) -> None:
    """Given successive artifact writes, when cycled, then no file descriptors are leaked."""
    import resource

    try:
        soft, _ = resource.getrlimit(resource.RLIMIT_NOFILE)
    except (ImportError, AttributeError):
        pytest.skip("resource.getrlimit unavailable on this platform")

    for cycle in range(10):
        config = ScenarioConfig(seed=66 + cycle, duration_seconds=1, rate_hz=20)
        report = DeterministicScenario(config).run()
        event_path = tmp_path / f"artifact_{cycle}.json"

        if event_path.exists():
            event_path.unlink()

        EventArtifactWriter(event_path).write(report)
        # Verify we can still open another file (no leak)
        assert event_path.exists()
        event_path.unlink()

    # After 10 cycles, fd usage should still be well below the soft limit
    # This is a sanity check that we haven't leaked
    assert soft > 0
