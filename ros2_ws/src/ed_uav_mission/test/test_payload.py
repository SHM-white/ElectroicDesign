import math
from dataclasses import replace

import pytest
from ed_uav_mission.mission_model import PayloadParams
from ed_uav_mission.plugins import payload
from ed_uav_mission.plugins.payload import PayloadPlugin


def test_payload_plugin_preserves_typed_trigger_params() -> None:
    # Given: the current typed payload trigger configuration.
    params = PayloadParams(action="laser_on", duration_sec=1.5)

    # When: the placeholder plugin generates its payload action.
    generated = PayloadPlugin().generate(params)

    # Then: the existing identity behavior is preserved before replacement.
    assert generated is params


def _release_context():
    return payload.ReleaseContext(
        request_id="task1-drop-001",
        now_monotonic_s=100.0,
        phase=payload.ReleasePhase.TASK1_RELEASE,
        target_observed_at_s=99.9,
        vehicle_observed_at_s=99.9,
        localization_observed_at_s=99.9,
        calibration_valid=True,
        standoff_m=0.75,
        cancelled=False,
    )


def _release_config():
    return payload.PayloadBoundaryConfig(
        contract_version=1,
        freshness_timeout_s=0.2,
        actuator_timeout_s=0.5,
        minimum_standoff_m=0.5,
        contact_dwell_s=5.0,
        minimum_vehicle_speed_m_s=0.05,
    )


def test_authorized_task1_release_is_acknowledged_exactly_once() -> None:
    # Given: every release interlock is fresh and an actuator will acknowledge.
    actuator = payload.FakePayloadActuator(
        outcomes=(payload.ActuatorAcknowledged(acknowledgement_id="ack-001"),)
    )
    latch = payload.ReleaseLatch()

    # When: reconstructed plugins share one mission-owned release latch.
    first = PayloadPlugin(latch).release(
        _release_context(), actuator, _release_config()
    )
    second = PayloadPlugin(latch).release(
        _release_context(), actuator, _release_config()
    )

    # Then: one hardware request succeeds and the latch rejects the duplicate.
    assert first == payload.ReleaseSucceeded(
        request_id="task1-drop-001", acknowledgement_id="ack-001"
    )
    assert second.reason is payload.ReleaseRejectionReason.ALREADY_ATTEMPTED
    assert len(actuator.commands) == 1


@pytest.mark.parametrize(
    ("changes", "reason"),
    [
        (
            {"phase": payload.ReleasePhase.TASK1_ESCORT},
            "bad_phase",
        ),
        (
            {"target_observed_at_s": 99.79},
            "target_stale",
        ),
        (
            {"vehicle_observed_at_s": 99.79},
            "vehicle_stale",
        ),
        (
            {"localization_observed_at_s": 99.79},
            "localization_stale",
        ),
        (
            {"calibration_valid": False},
            "calibration_invalid",
        ),
        (
            {"standoff_m": 0.49},
            "standoff_unsafe",
        ),
        (
            {"cancelled": True},
            "cancelled",
        ),
        (
            {"target_observed_at_s": math.nan},
            "clock_invalid",
        ),
    ],
)
def test_release_interlock_rejects_fault_without_calling_actuator(
    changes: dict[str, float | str | bool], reason: str
) -> None:
    # Given: one release prerequisite is unsafe.
    context = replace(_release_context(), **changes)
    actuator = payload.FakePayloadActuator(
        outcomes=(payload.ActuatorAcknowledged(acknowledgement_id="unused"),)
    )

    # When: the pure interlock and plugin evaluate the request.
    decision = payload.evaluate_release_interlock(context, _release_config())
    result = PayloadPlugin(payload.ReleaseLatch()).release(
        context, actuator, _release_config()
    )

    # Then: both boundaries reject with the same typed reason and no actuation.
    assert decision.reason.value == reason
    assert result.reason.value == reason
    assert result.recovery.actions == (
        payload.RecoveryAction.HOVER,
        payload.RecoveryAction.RETURN_HOME,
        payload.RecoveryAction.LAND,
    )
    assert actuator.commands == []


@pytest.mark.parametrize(
    ("outcome_name", "reason"),
    [
        ("rejected", "actuator_rejected"),
        ("timed_out", "actuator_timeout"),
        ("unknown", "actuator_unknown"),
    ],
)
def test_actuator_fault_latches_release_and_never_retries(
    outcome_name: str, reason: str
) -> None:
    # Given: an actuator returns a terminal non-ACK outcome.
    outcomes = {
        "rejected": payload.ActuatorRejected(detail="interlock open"),
        "timed_out": payload.ActuatorTimedOut(elapsed_s=0.5),
        "unknown": payload.ActuatorUnknown(detail="link lost after trigger"),
    }
    actuator_outcome = outcomes[outcome_name]
    actuator = payload.FakePayloadActuator(outcomes=(actuator_outcome,))
    latch = payload.ReleaseLatch()

    # When: release is attempted and a reconstructed plugin requests it again.
    first = PayloadPlugin(latch).release(
        _release_context(), actuator, _release_config()
    )
    second = PayloadPlugin(latch).release(
        _release_context(), actuator, _release_config()
    )

    # Then: the first fault is explicit and the unknown physical state is never retried.
    assert first.reason.value == reason
    assert second.reason is payload.ReleaseRejectionReason.ALREADY_ATTEMPTED
    assert len(actuator.commands) == 1


def test_release_without_mission_latch_fails_closed() -> None:
    # Given: an otherwise authorized release but no mission-owned latch.
    actuator = payload.FakePayloadActuator(
        outcomes=(payload.ActuatorAcknowledged(acknowledgement_id="unused"),)
    )

    # When: a legacy parameter-only plugin is asked to release.
    result = PayloadPlugin().release(
        _release_context(), actuator, _release_config()
    )

    # Then: hardware remains untouched because exactly-once ownership is absent.
    assert result.reason is payload.ReleaseRejectionReason.LATCH_UNAVAILABLE
    assert actuator.commands == []
