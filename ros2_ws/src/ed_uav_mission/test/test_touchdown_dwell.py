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

    # When: qualifying contact remains continuous through the exact threshold.
    start = touchdown.adapt_payload_contact_state(FakePayloadContactState(), 10.0)
    before = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=2), 14.999
    )
    boundary = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=3), 15.0
    )
    first = tracker.update(_update(touchdown, 10.0, start))
    second = tracker.update(_update(touchdown, 14.999, before))
    third = tracker.update(_update(touchdown, 15.0, boundary))

    # Then: 4.999 s is insufficient and 5.000 s succeeds deterministically.
    assert first.elapsed_s == 0.0
    assert second.elapsed_s == pytest.approx(4.999)
    assert third.elapsed_s == 5.0
    assert third.completed is True


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
    bounced = touchdown.adapt_payload_contact_state(bounced_message, 22.0)
    interruption = tracker.update(_update(touchdown, 22.0, bounced))
    resumed = touchdown.adapt_payload_contact_state(
        FakePayloadContactState(source_sequence=3), 25.0
    )
    progress = tracker.update(_update(touchdown, 25.0, resumed))

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
