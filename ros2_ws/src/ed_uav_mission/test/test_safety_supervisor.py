"""Tests for the mission safety supervisor."""

from __future__ import annotations

import pytest

from ed_uav_mission.safety_supervisor import (
    CommandAckState,
    LockVerdict,
    SafetySupervisor,
    SupervisorAction,
    SupervisorState,
)


# ---------------------------------------------------------------------------
# ACK / retry tests
# ---------------------------------------------------------------------------

def test_ack_success() -> None:
    """ACK received within the timeout window — command accepted."""
    sup = SafetySupervisor()
    cid = sup.new_correlation_id()

    sup.record_send(correlation_id=cid, command_code=6, timestamp_s=100.0)
    # ACK arrives at 100.20 s (< 0.50 s timeout)
    accepted = sup.receive_ack(correlation_id=cid, timestamp_s=100.20)
    assert accepted is True

    # No timed-out commands
    expired = sup.poll_timeouts(timestamp_s=100.30)
    assert len(expired) == 0


def test_ack_retry_exhaustion() -> None:
    """No ACK arrives after 2 retries — retry budget exhausted."""
    sup = SafetySupervisor()
    cid1 = sup.new_correlation_id()

    # -- initial send --------------------------------------------------------
    sup.record_send(correlation_id=cid1, command_code=6, timestamp_s=100.0)

    # timeout at 0.50 s
    expired = sup.poll_timeouts(timestamp_s=100.51)
    assert len(expired) == 1
    assert expired[0].correlation_id == cid1
    assert expired[0].state == CommandAckState.TIMED_OUT

    # can retry
    assert sup.can_retry(expired[0]) is True
    cid2 = sup.new_correlation_id()
    sup.record_retry(expired[0], new_correlation_id=cid2, timestamp_s=100.60)
    assert expired[0].retries == 1

    # -- first retry times out -----------------------------------------------
    expired2 = sup.poll_timeouts(timestamp_s=101.11)
    assert len(expired2) == 1
    assert expired2[0].correlation_id == cid2
    assert expired2[0].retries == 1
    assert sup.can_retry(expired2[0]) is True

    cid3 = sup.new_correlation_id()
    sup.record_retry(expired2[0], new_correlation_id=cid3, timestamp_s=101.20)
    assert expired2[0].retries == 2

    # -- second retry times out → exhausted ----------------------------------
    expired3 = sup.poll_timeouts(timestamp_s=101.71)
    assert len(expired3) == 1
    assert expired3[0].correlation_id == cid3
    assert expired3[0].retries == 2
    assert sup.can_retry(expired3[0]) is False  # max retries reached


def test_late_ack() -> None:
    """ACK received after the timeout window is ignored."""
    sup = SafetySupervisor()
    cid = sup.new_correlation_id()

    sup.record_send(correlation_id=cid, command_code=6, timestamp_s=100.0)

    # timeout at 0.51 s
    expired = sup.poll_timeouts(timestamp_s=100.51)
    assert len(expired) == 1
    assert expired[0].state == CommandAckState.TIMED_OUT

    # ACK arrives late at 0.70 s
    accepted = sup.receive_ack(correlation_id=cid, timestamp_s=100.70)
    assert accepted is False  # late ACK → ignored


# ---------------------------------------------------------------------------
# Localisation-loss tests
# ---------------------------------------------------------------------------

def test_localization_loss_hover() -> None:
    """All localisation sources lost → supervisor decides HOVER."""
    sup = SafetySupervisor()
    sup.start_monitoring(timestamp_s=200.0)

    # All sources lost — should trigger hover
    verdict = sup.evaluate_localization(all_lost=True, timestamp_s=200.05)
    assert verdict.action == SupervisorAction.HOVER
    assert sup.state == SupervisorState.LOCALIZATION_LOST_HOVERING

    # Hover can be issued well within 0.20 s (the response is immediate when
    # evaluate_localization is called — the 0.20 s is the real-time
    # integration budget, not a delay the supervisor itself enforces).
    # We verify it was detected on the very first evaluation with no delay.


def test_localization_recovery_at_1_9s() -> None:
    """Recovery within the 2.0 s hold window → resume normal operation."""
    sup = SafetySupervisor()
    sup.start_monitoring(timestamp_s=200.0)

    # localisation lost at t=200.0
    verdict = sup.evaluate_localization(all_lost=True, timestamp_s=200.0)
    assert verdict.action == SupervisorAction.HOVER
    assert sup.state == SupervisorState.LOCALIZATION_LOST_HOVERING

    # recovery at t=201.9 (< 2.0 s hold)
    verdict2 = sup.evaluate_localization(all_lost=False, timestamp_s=201.9)
    assert verdict2.action is None  # no new action
    assert sup.state == SupervisorState.ACTIVE  # resumed


def test_localization_recovery_at_2_1s() -> None:
    """Recovery after the 2.0 s hold window → land was already initiated."""
    sup = SafetySupervisor()
    sup.start_monitoring(timestamp_s=200.0)

    # localisation lost at t=200.0
    verdict = sup.evaluate_localization(all_lost=True, timestamp_s=200.0)
    assert verdict.action == SupervisorAction.HOVER
    assert sup.state == SupervisorState.LOCALIZATION_LOST_HOVERING

    # t=202.1 s — still lost, hold window expired → transition to LANDING
    verdict2 = sup.evaluate_localization(all_lost=True, timestamp_s=202.1)
    assert verdict2.action == SupervisorAction.LAND
    assert sup.state == SupervisorState.LOCALIZATION_LOST_LANDING

    # Now localisation recovers at t=202.3, but we are already landing
    verdict3 = sup.evaluate_localization(all_lost=False, timestamp_s=202.3)
    assert verdict3.action is None
    assert sup.state == SupervisorState.LOCALIZATION_LOST_LANDING  # stays in landing


# ---------------------------------------------------------------------------
# Land-without-ACK test
# ---------------------------------------------------------------------------

def test_land_no_ack() -> None:
    """LAND command gets no ACK and no descent → CRITICAL after retries."""
    sup = SafetySupervisor()

    # Manually set supervisor into landing state (simulating that
    # localisation-loss or another transition already triggered landing).
    sup.state = SupervisorState.LOCALIZATION_LOST_LANDING
    sup._monitoring_active = True

    # No altitude data → _is_descending() is False
    t = 300.0

    # First evaluation in landing state: initial land command
    v1 = sup.evaluate_localization(all_lost=True, timestamp_s=t)
    assert v1.action == SupervisorAction.LAND
    assert sup._land_cmd_count == 1

    # t+1.0 s — retry 1 (no descent, no ACK)
    v2 = sup.evaluate_localization(all_lost=True, timestamp_s=t + 1.0)
    assert v2.action == SupervisorAction.LAND
    assert sup._land_cmd_count == 2

    # t+2.0 s — retry 2
    v3 = sup.evaluate_localization(all_lost=True, timestamp_s=t + 2.0)
    assert v3.action == SupervisorAction.LAND
    assert sup._land_cmd_count == 3

    # t+3.0 s — exhausted (3 retries reached, count=4 > MAX_LAND_RETRIES=3)
    v4 = sup.evaluate_localization(all_lost=True, timestamp_s=t + 3.0)
    assert v4.action == SupervisorAction.CRITICAL
    assert sup.state == SupervisorState.CRITICAL
    assert "manual takeover" in v4.reason.lower()


# ---------------------------------------------------------------------------
# Lock-in-air gate
# ---------------------------------------------------------------------------

def test_no_lock_in_air() -> None:
    """Altitude > 10 cm → lock is refused."""
    sup = SafetySupervisor()
    sup.update_altitude(0.50)  # 50 cm — definitely in the air

    verdict = sup.check_lock_safe()
    assert verdict.allowed is False
    assert "refusing lock" in verdict.reason.lower()


def test_lock_allowed_after_touchdown() -> None:
    """Altitude <= 10 cm → lock is permitted."""
    sup = SafetySupervisor()
    sup.update_altitude(0.05)  # 5 cm — on the ground

    verdict = sup.check_lock_safe()
    assert verdict.allowed is True
    assert "touchdown" in verdict.reason.lower()


def test_lock_refused_no_altitude_data() -> None:
    """No altitude data → lock is refused."""
    sup = SafetySupervisor()
    # Never called update_altitude
    verdict = sup.check_lock_safe()
    assert verdict.allowed is False
    assert "no altitude" in verdict.reason.lower()


# ---------------------------------------------------------------------------
# Descent detection (helper for land-no-ack)
# ---------------------------------------------------------------------------

def test_land_keeps_supervising_on_descent() -> None:
    """LAND with no ACK but descent observed → keep supervising, do NOT escalate."""
    sup = SafetySupervisor()
    sup.state = SupervisorState.LOCALIZATION_LOST_LANDING
    sup._monitoring_active = True

    t = 300.0

    # initial land command
    v1 = sup.evaluate_localization(all_lost=True, timestamp_s=t)
    assert v1.action == SupervisorAction.LAND
    assert sup._land_cmd_count == 1

    # Feed altitude so the supervisor sees descent
    sup.update_altitude(5.0)
    sup.update_altitude(4.8)  # descending

    # t+1.0 s — retry interval elapsed, but descent observed
    v2 = sup.evaluate_localization(all_lost=True, timestamp_s=t + 1.0)
    assert v2.action is None  # keeps supervising, resets the retry timer
    # land_cmd_count should NOT have incremented beyond 1
    assert sup._land_cmd_count == 1

    # t+2.0 s — still descending, still supervising
    sup.update_altitude(4.6)
    v3 = sup.evaluate_localization(all_lost=True, timestamp_s=t + 2.0)
    assert v3.action is None
    assert sup._land_cmd_count == 1


# ---------------------------------------------------------------------------
# Other safety transitions
# ---------------------------------------------------------------------------

def test_comm_loss_triggers_critical() -> None:
    """FCU communication loss during active mission → CRITICAL."""
    sup = SafetySupervisor()
    sup.start_monitoring(timestamp_s=400.0)

    verdict = sup.evaluate_comm_loss(comm_ok=False, timestamp_s=400.5)
    assert verdict.action == SupervisorAction.CRITICAL
    assert sup.state == SupervisorState.CRITICAL


def test_low_voltage_triggers_land() -> None:
    """Battery voltage below threshold → initiate land."""
    sup = SafetySupervisor()
    sup.start_monitoring(timestamp_s=400.0)

    verdict = sup.evaluate_low_voltage(voltage_v=9.8, timestamp_s=400.5)
    assert verdict.action == SupervisorAction.LAND
    assert sup.state == SupervisorState.LOCALIZATION_LOST_LANDING


def test_mission_timeout_triggers_land() -> None:
    """Mission duration exceeds timeout → initiate land."""
    sup = SafetySupervisor()
    sup.start_monitoring(timestamp_s=500.0, timeout_s=120.0)

    # Before timeout
    v1 = sup.evaluate_mission_timeout(timestamp_s=600.0)
    assert v1.action is None

    # After timeout
    v2 = sup.evaluate_mission_timeout(timestamp_s=621.0)
    assert v2.action == SupervisorAction.LAND
    assert sup.state == SupervisorState.LOCALIZATION_LOST_LANDING


# ---------------------------------------------------------------------------
# Lifecycle gating
# ---------------------------------------------------------------------------

def test_supervisor_ignores_when_not_monitoring() -> None:
    """When monitoring is inactive, no safety transitions fire."""
    sup = SafetySupervisor()
    # never called start_monitoring

    assert sup.evaluate_localization(all_lost=True, timestamp_s=0.0).action is None
    assert sup.evaluate_comm_loss(comm_ok=False, timestamp_s=0.0).action is None
    assert sup.evaluate_low_voltage(voltage_v=5.0, timestamp_s=0.0).action is None
    assert sup.evaluate_mission_timeout(timestamp_s=999.0).action is None
