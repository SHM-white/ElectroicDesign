from __future__ import annotations

import hashlib

import pytest

from ed_uav_verification.artifacts import ArtifactExistsError, FixtureBagBuilder, IncompleteScenarioError
from ed_uav_verification.faults import FaultKind, FaultWindow
from ed_uav_verification.fcu import DeterministicPtyFcu
from ed_uav_verification.io_fakes import FakeGpioLaser
from ed_uav_verification.model import ScenarioConfig, ScenarioBoundError, Stream
from ed_uav_verification.scenario import DeterministicScenario


def test_seeded_sixty_second_replay_is_byte_identical() -> None:
    """Given one seed, when a 60-second 20Hz replay runs twice, then JSON is exact."""
    config = ScenarioConfig(seed=17, duration_seconds=60, rate_hz=20)

    first = DeterministicScenario(config).run()
    second = DeterministicScenario(config).run()

    assert first.completed
    assert first.simulated_duration_ns == 60_000_000_000
    assert first.tick_count == 1_200
    assert first.event_json == second.event_json
    assert hashlib.sha256(first.event_json).hexdigest() == hashlib.sha256(second.event_json).hexdigest()


@pytest.mark.parametrize("kind", tuple(FaultKind), ids=lambda kind: kind.value)
def test_injector_emits_activation_degradation_and_recovery(kind: FaultKind) -> None:
    """Given every injector, when its fault window ends, then the source recovers."""
    config = ScenarioConfig(
        seed=3,
        duration_seconds=2,
        rate_hz=20,
        faults=(FaultWindow(kind=kind, stream=Stream.LIDAR_POINTS, start_tick=5, duration_ticks=8),),
    )

    report = DeterministicScenario(config).run()

    assert report.has_fault_activation(kind)
    assert report.has_fault_recovery(kind)
    assert report.has_degradation(kind)
    assert report.has_stream_recovery(Stream.LIDAR_POINTS)


def test_stale_freeze_is_rejected_without_reusing_the_last_message() -> None:
    """Given a frozen camera stamp, when freshness expires, then it is rejected."""
    config = ScenarioConfig(
        seed=9,
        duration_seconds=2,
        rate_hz=20,
        faults=(FaultWindow(kind=FaultKind.FREEZE, stream=Stream.NARROW_IMAGE, start_tick=4, duration_ticks=8),),
    )

    report = DeterministicScenario(config).run()

    assert report.rejection_reasons(Stream.NARROW_IMAGE) == ("stale",)
    assert report.accepted_sequences(Stream.NARROW_IMAGE)[-1] > report.rejected_sequences(Stream.NARROW_IMAGE)[-1]
    assert report.has_stream_recovery(Stream.NARROW_IMAGE)


def test_pty_fcu_writes_characterized_position_and_closes() -> None:
    """Given a PTY FCU, when it emits V7 position, then its endpoint is cleaned up."""
    with DeterministicPtyFcu() as fcu:
        frame = fcu.emit_position(x_cm=1234, y_cm=-567)
        received = fcu.read_slave_frame()

        assert received == frame
        assert received.hex() == "aaff0808d2040000c9fdffff5349"

    assert fcu.closed


def test_fake_gpio_laser_has_no_hardware_side_effect_and_records_state() -> None:
    """Given the fake output service, when laser state changes, then only its snapshot changes."""
    outputs = FakeGpioLaser()

    initial = outputs.snapshot()
    enabled = outputs.set_laser(True)
    disabled = outputs.set_laser(False)

    assert not initial.laser_enabled
    assert enabled.laser_enabled
    assert not disabled.laser_enabled


def test_fixture_builder_refuses_stale_artifact_and_cleans_incomplete_work(tmp_path) -> None:
    """Given incomplete and existing artifacts, when persisted, then stale output is rejected."""
    path = tmp_path / "fixture"
    complete = DeterministicScenario(ScenarioConfig(seed=4, duration_seconds=1, rate_hz=20)).run()
    interrupted = DeterministicScenario(ScenarioConfig(seed=4, duration_seconds=1, rate_hz=20)).run(stop_after_ticks=4)

    with pytest.raises(IncompleteScenarioError):
        FixtureBagBuilder(path).write(interrupted)

    first = FixtureBagBuilder(path).write(complete)

    with pytest.raises(ArtifactExistsError):
        FixtureBagBuilder(path).write(complete)

    assert first.event_path.read_bytes() == complete.event_json
    assert not list(tmp_path.glob("*.partial"))


def test_bounded_interruption_can_restart_without_timing_flake() -> None:
    """Given repeated bounded interruption, when a fresh harness resumes, then it is deterministic."""
    config = ScenarioConfig(seed=5, duration_seconds=1, rate_hz=20)

    interrupted = tuple(DeterministicScenario(config).run(stop_after_ticks=tick) for tick in (3, 7))
    resumed = DeterministicScenario(config).run()

    assert all(not report.completed for report in interrupted)
    assert resumed.completed
    assert resumed.event_json == DeterministicScenario(config).run().event_json


def test_hung_scenario_request_is_rejected_before_iteration() -> None:
    """Given a tick budget breach, when replay starts, then it fails in bounded time."""
    config = ScenarioConfig(seed=2, duration_seconds=61, rate_hz=20, max_ticks=1_200)

    with pytest.raises(ScenarioBoundError):
        DeterministicScenario(config).run()
