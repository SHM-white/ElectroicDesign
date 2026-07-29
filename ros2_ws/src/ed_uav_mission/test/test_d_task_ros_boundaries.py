from __future__ import annotations

import math
from pathlib import Path

import pytest
from ed_uav_fcu_bridge.authority import ProgrammableCapabilityError
from ed_uav_interfaces.msg import MissionStatus, TargetObservation, VehicleTelemetry
from ed_uav_mission import d_task_capability
from ed_uav_mission.d_task_inputs import (
    DTaskInputError,
    adapt_target_observation,
    adapt_vehicle_telemetry,
)
from ed_uav_mission.d_task_model import DTaskPhase
from ed_uav_mission.d_task_status import mission_status_state


def _vehicle() -> VehicleTelemetry:
    message = VehicleTelemetry()
    message.contract_version = VehicleTelemetry.CONTRACT_VERSION
    message.source_sequence = 7
    message.start_event = True
    message.heartbeat_alive = True
    message.motion_kind = VehicleTelemetry.MOTION_WHEEL_SPEED
    message.wheel_speed_m_s = 0.2
    message.route_stage = VehicleTelemetry.ROUTE_B
    return message


def _target() -> TargetObservation:
    message = TargetObservation()
    message.contract_version = TargetObservation.CONTRACT_VERSION
    message.source_sequence = 9
    message.target_revision = "d2026-circle-cross-v1"
    message.valid = True
    message.status = TargetObservation.STATUS_VALID
    message.pose.pose.position.x = 0.3
    message.pose.pose.position.y = 0.4
    message.pose.pose.position.z = 1.5
    message.quality = 0.9
    return message


def test_vehicle_adapter_preserves_car_event_prediction_inputs() -> None:
    snapshot = adapt_vehicle_telemetry(_vehicle(), 10.0)

    assert snapshot.started is True
    assert snapshot.speed_m_s == pytest.approx(0.2)
    assert snapshot.route_stage.value == VehicleTelemetry.ROUTE_B


@pytest.mark.parametrize("field", ["wheel_speed_m_s", "heading_rad", "yaw_rate_rad_s"])
def test_vehicle_adapter_rejects_nonfinite_input(field: str) -> None:
    message = _vehicle()
    setattr(message, field, math.nan)

    with pytest.raises(DTaskInputError, match="nonfinite"):
        adapt_vehicle_telemetry(message, 10.0)


def test_target_adapter_uses_typed_pnp_pose_for_relative_error() -> None:
    snapshot = adapt_target_observation(
        _target(),
        10.0,
        "d2026-circle-cross-v1",
    )

    assert snapshot.relative_error_m == pytest.approx(0.5)
    assert snapshot.relative_z_m == pytest.approx(1.5)


def test_target_adapter_rejects_wrong_revision_and_nonfinite_pose() -> None:
    wrong_revision = _target()
    wrong_revision.target_revision = "other-target"
    with pytest.raises(DTaskInputError, match="revision"):
        adapt_target_observation(wrong_revision, 10.0, "d2026-circle-cross-v1")

    nonfinite = _target()
    nonfinite.pose.pose.position.x = math.inf
    with pytest.raises(DTaskInputError, match="nonfinite"):
        adapt_target_observation(nonfinite, 10.0, "d2026-circle-cross-v1")


def test_status_mapping_exposes_safe_and_terminal_phases() -> None:
    assert mission_status_state(DTaskPhase.SAFE_HOVER) == MissionStatus.STATE_RETURNING_HOME
    assert mission_status_state(DTaskPhase.SAFE_LAND) == MissionStatus.STATE_LANDING_HOME
    assert mission_status_state(DTaskPhase.SUCCEEDED) == MissionStatus.STATE_SUCCEEDED
    assert mission_status_state(DTaskPhase.ABORTED) == MissionStatus.STATE_ABORTED


def test_simulation_capability_allows_fake_while_field_missing_fails_closed(
    tmp_path: Path,
) -> None:
    simulation = d_task_capability.evaluate_d_task_capability(
        simulation_only=True,
        report_path=tmp_path / "missing.json",
        device_identity="",
        environment={},
    )
    field = d_task_capability.evaluate_d_task_capability(
        simulation_only=False,
        report_path=tmp_path / "missing.json",
        device_identity="v7-001",
        environment={},
    )

    assert simulation.ready is True
    assert field.ready is False
    assert "provenance" in field.reason


def test_field_red_capability_reason_is_retained(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        d_task_capability,
        "capability_trust_from_environment",
        lambda report_path, device_identity, environment: None,
    )

    def reject(enabled, trust):
        raise ProgrammableCapabilityError("capability report is red: V7 timeout")

    monkeypatch.setattr(d_task_capability, "require_programmable_capability", reject)

    decision = d_task_capability.evaluate_d_task_capability(
        simulation_only=False,
        report_path=Path("red.json"),
        device_identity="v7-001",
        environment={},
    )

    assert decision.ready is False
    assert decision.reason == "capability report is red: V7 timeout"
