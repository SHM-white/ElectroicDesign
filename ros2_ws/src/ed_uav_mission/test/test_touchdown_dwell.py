from dataclasses import dataclass, replace

import pytest


@dataclass(frozen=True, slots=True)
class FakePayloadContactState:
    CONTRACT_VERSION = 1
    CONTACT_AIRBORNE = 0
    CONTACT_HOME = 1
    CONTACT_VEHICLE = 2

    contract_version: int = 1
    source_sequence: int = 1
    payload_state: int = 1
    contact_state: int = 2
    contact_stable: bool = True
    contact_duration_s: float = 0.0
    owner: str = "task2"
    frame_id: str = "base_link"


def _config(touchdown):
    return touchdown.PayloadBoundaryConfig(
        contract_version=1,
        freshness_timeout_s=0.2,
        actuator_timeout_s=0.5,
        minimum_standoff_m=0.5,
        contact_dwell_s=5.0,
        minimum_vehicle_speed_m_s=0.05,
    )


def _update(touchdown, now_s: float, observation):
    return touchdown.TouchdownUpdate(
        now_monotonic_s=now_s,
        target_observed_at_s=now_s - 0.1,
        vehicle_observed_at_s=now_s - 0.1,
        vehicle_speed_m_s=0.2,
        contact=observation,
        cancelled=False,
    )


def test_payload_contact_state_adapter_preserves_contract_fields() -> None:
    # Given: a Todo 1 PayloadContactState-shaped ROS message.
    from ed_uav_mission import touchdown

    message = FakePayloadContactState(source_sequence=7, contact_duration_s=12.0)

    # When: it crosses into the monotonic domain boundary.
    observation = touchdown.adapt_payload_contact_state(message, 50.0)

    # Then: the typed observation uses the contract state but not its claimed dwell clock.
    assert observation.sequence == 7
    assert observation.state is touchdown.ContactState.VEHICLE
    assert observation.observed_at_monotonic_s == 50.0


def test_continuous_moving_contact_completes_at_exactly_five_seconds() -> None:
    # Given: stable vehicle contact and fresh moving-platform state.
    from ed_uav_mission import touchdown

    tracker = touchdown.TouchdownDwellTracker(_config(touchdown))

    # When: advancing samples arrive every 0.2 seconds through the threshold.
    results = []
    for index in range(26):
        now_s = 10.0 + (index * 0.2)
        contact = touchdown.adapt_payload_contact_state(
            FakePayloadContactState(source_sequence=100 + index), now_s
        )
        results.append(tracker.update(_update(touchdown, now_s, contact)))

    # Then: 4.8 s is insufficient and 5.0 s succeeds deterministically.
    assert results[0].elapsed_s == 0.0
    assert results[-2].elapsed_s == pytest.approx(4.8)
    assert results[-2].completed is False
    assert results[-1].elapsed_s == 5.0
    assert results[-1].completed is True


def test_sparse_advancing_contact_samples_cannot_complete_dwell() -> None:
    # Given: stable moving contact with sequence numbers that advance.
    from ed_uav_mission import touchdown

    tracker = touchdown.TouchdownDwellTracker(_config(touchdown))
    first_contact = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=1), 10.0
    )
    tracker.update(_update(touchdown, 10.0, first_contact))

    # When: the next accepted-looking samples exceed the configured 0.2-s gap.
    sparse_contact = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=2), 10.201
    )
    sparse = tracker.update(_update(touchdown, 10.201, sparse_contact))
    much_later_contact = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=3), 15.0
    )
    much_later = tracker.update(_update(touchdown, 15.0, much_later_contact))

    # Then: both gaps interrupt continuity and never manufacture completion.
    assert sparse.reason is touchdown.DwellInterruptionReason.CONTACT_GAP
    assert much_later.reason is touchdown.DwellInterruptionReason.CONTACT_GAP
    assert much_later.completed is False


@pytest.mark.parametrize("replayed_offset", [0, -1])
def test_duplicate_or_replayed_sequence_cannot_complete_dwell(
    replayed_offset: int,
) -> None:
    # Given: dense advancing contact has accumulated 4.8 seconds.
    from ed_uav_mission import touchdown

    tracker = touchdown.TouchdownDwellTracker(_config(touchdown))
    for index in range(25):
        now_s = 20.0 + (index * 0.2)
        contact = touchdown.adapt_payload_contact_state(
            FakePayloadContactState(source_sequence=100 + index), now_s
        )
        tracker.update(_update(touchdown, now_s, contact))

    # When: a duplicate or replayed sequence arrives with a newer timestamp.
    replayed_contact = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=124 + replayed_offset), 24.9
    )
    interrupted = tracker.update(_update(touchdown, 24.9, replayed_contact))
    next_contact = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=125), 25.0
    )
    restarted = tracker.update(_update(touchdown, 25.0, next_contact))

    # Then: the invalid sample cannot complete, and valid contact restarts at zero.
    assert (
        interrupted.reason
        is touchdown.DwellInterruptionReason.CONTACT_SEQUENCE_INVALID
    )
    assert interrupted.completed is False
    assert restarted.elapsed_s == 0.0
    assert restarted.completed is False


def test_uint32_sequence_wrap_advances_contact_stream() -> None:
    # Given: the last uint32 sequence value has been accepted.
    from ed_uav_mission import touchdown

    tracker = touchdown.TouchdownDwellTracker(_config(touchdown))
    maximum = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=(2**32) - 1), 30.0
    )
    tracker.update(_update(touchdown, 30.0, maximum))

    # When: the next contact sequence wraps to zero within the freshness gap.
    wrapped = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=0), 30.1
    )
    result = tracker.update(_update(touchdown, 30.1, wrapped))

    # Then: wrap is strict forward advancement, not replay.
    assert result.elapsed_s == pytest.approx(0.1)
    assert result.completed is False


def test_contact_stamp_must_strictly_advance() -> None:
    # Given: one accepted contact observation timestamp.
    from ed_uav_mission import touchdown

    tracker = touchdown.TouchdownDwellTracker(_config(touchdown))
    first_contact = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=1), 40.0
    )
    tracker.update(_update(touchdown, 40.0, first_contact))

    # When: sequence advances but the contact observation stamp does not.
    repeated_stamp = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=2), 40.0
    )
    result = tracker.update(_update(touchdown, 40.1, repeated_stamp))

    # Then: replayed time interrupts dwell.
    assert result.reason is touchdown.DwellInterruptionReason.CONTACT_STAMP_INVALID


@pytest.mark.parametrize("sequence", [-1, 2**32])
def test_payload_contact_state_rejects_sequence_outside_uint32(sequence: int) -> None:
    # Given: a PayloadContactState-shaped message outside uint32 bounds.
    from ed_uav_mission import touchdown

    # When/Then: the adapter rejects it at the trust boundary.
    with pytest.raises(touchdown.PayloadContactContractError):
        touchdown.adapt_payload_contact_state(
            FakePayloadContactState(source_sequence=sequence), 45.0
        )


def test_contact_bounce_resets_continuous_dwell() -> None:
    # Given: two seconds of qualifying vehicle contact.
    from ed_uav_mission import touchdown

    tracker = touchdown.TouchdownDwellTracker(_config(touchdown))
    contact = touchdown.adapt_payload_contact_state(FakePayloadContactState(), 20.0)
    tracker.update(_update(touchdown, 20.0, contact))

    # When: contact bounces airborne before the five-second boundary.
    bounced_message = FakePayloadContactState(
        source_sequence=2, contact_state=FakePayloadContactState.CONTACT_AIRBORNE
    )
    bounced = touchdown.adapt_payload_contact_state(bounced_message, 20.1)
    interruption = tracker.update(_update(touchdown, 20.1, bounced))
    resumed = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=3), 20.2
    )
    progress = tracker.update(_update(touchdown, 20.2, resumed))

    # Then: the false pulse fails and resumed contact starts from zero.
    assert interruption.reason is touchdown.DwellInterruptionReason.CONTACT_LOST
    assert progress.elapsed_s == 0.0


def test_stopped_car_interrupts_contact_dwell() -> None:
    # Given: a valid contact observation but stopped vehicle telemetry.
    from ed_uav_mission import touchdown

    tracker = touchdown.TouchdownDwellTracker(_config(touchdown))
    contact = touchdown.adapt_payload_contact_state(FakePayloadContactState(), 30.0)
    update = replace(_update(touchdown, 30.0, contact), vehicle_speed_m_s=0.0)

    # When: the tracker evaluates the moving-contact requirement.
    result = tracker.update(update)

    # Then: stopped-platform contact fails with safe recovery guidance.
    assert result.reason is touchdown.DwellInterruptionReason.VEHICLE_STOPPED
    assert result.recovery.actions[0].value == "hover"


def test_stopped_car_sample_resets_accumulated_dwell() -> None:
    # Given: moving contact has started accumulating dwell.
    from ed_uav_mission import touchdown

    tracker = touchdown.TouchdownDwellTracker(_config(touchdown))
    first_contact = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=1), 35.0
    )
    tracker.update(_update(touchdown, 35.0, first_contact))

    # When: an advancing stopped-car sample is followed by moving contact.
    stopped_contact = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=2), 35.1
    )
    stopped = tracker.update(
        replace(
            _update(touchdown, 35.1, stopped_contact), vehicle_speed_m_s=0.0
        )
    )
    resumed_contact = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=2), 35.2
    )
    resumed = tracker.update(_update(touchdown, 35.2, resumed_contact))

    # Then: stopped motion is not accepted, so reused sequence 2 starts at zero.
    assert stopped.reason is touchdown.DwellInterruptionReason.VEHICLE_STOPPED
    assert resumed == touchdown.DwellProgress(elapsed_s=0.0)


def test_stale_contact_sample_resets_accumulated_dwell() -> None:
    # Given: one accepted contact sample starts dwell.
    from ed_uav_mission import touchdown

    tracker = touchdown.TouchdownDwellTracker(_config(touchdown))
    first_contact = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=1), 36.0
    )
    tracker.update(_update(touchdown, 36.0, first_contact))

    # When: a stale duplicate stamp is followed by a fresh advancing sample.
    stale_contact = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=2), 36.0
    )
    stale = tracker.update(_update(touchdown, 36.21, stale_contact))
    fresh_contact = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=3), 36.2
    )
    resumed = tracker.update(_update(touchdown, 36.3, fresh_contact))

    # Then: stale contact interrupts and the accepted fresh sample starts at zero.
    assert stale.reason is touchdown.DwellInterruptionReason.CONTACT_STALE
    assert resumed.elapsed_s == 0.0


def test_stale_target_or_cancellation_interrupts_descent_dwell() -> None:
    # Given: an active dwell tracker and a fresh contact observation.
    from ed_uav_mission import touchdown

    contact = touchdown.adapt_payload_contact_state(FakePayloadContactState(), 40.0)
    tracker = touchdown.TouchdownDwellTracker(_config(touchdown))

    # When: target state becomes stale, then a separate flow is cancelled.
    stale = replace(
        _update(touchdown, 40.0, contact), target_observed_at_s=39.79
    )
    stale_result = tracker.update(stale)
    cancelled_tracker = touchdown.TouchdownDwellTracker(_config(touchdown))
    cancelled = replace(_update(touchdown, 40.0, contact), cancelled=True)
    cancelled_result = cancelled_tracker.update(cancelled)

    # Then: both interruptions are explicit and recommend safe recovery only.
    assert stale_result.reason is touchdown.DwellInterruptionReason.TARGET_STALE
    assert cancelled_result.reason is touchdown.DwellInterruptionReason.CANCELLED


def test_localization_loss_interrupts_descent_dwell() -> None:
    # Given: fresh moving contact but localization has been lost.
    from ed_uav_mission import touchdown

    contact = touchdown.adapt_payload_contact_state(FakePayloadContactState(), 45.0)
    tracker = touchdown.TouchdownDwellTracker(_config(touchdown))
    update = replace(
        _update(touchdown, 45.0, contact), localization_valid=False
    )

    # When: the descent boundary evaluates the update.
    result = tracker.update(update)

    # Then: dwell is cancelled and recovery is only recommended to the runtime.
    assert result.reason is touchdown.DwellInterruptionReason.LOCALIZATION_LOST
    assert result.recovery.actions[0].value == "hover"


def test_monotonic_time_regression_fails_closed_without_flaky_sleep() -> None:
    # Given: one accepted contact update at monotonic time 50.0.
    from ed_uav_mission import touchdown

    tracker = touchdown.TouchdownDwellTracker(_config(touchdown))
    first_contact = touchdown.adapt_payload_contact_state(FakePayloadContactState(), 50.0)
    tracker.update(_update(touchdown, 50.0, first_contact))

    # When: a later callback carries a regressed monotonic timestamp.
    regressed_contact = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=2), 49.9
    )
    result = tracker.update(_update(touchdown, 49.9, regressed_contact))

    # Then: the clock fault resets dwell instead of manufacturing elapsed time.
    assert result.reason is touchdown.DwellInterruptionReason.CLOCK_INVALID
