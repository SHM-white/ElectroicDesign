from __future__ import annotations

from dataclasses import replace

from ed_uav_mission.d_task_model import DTaskKind, DTaskSelection, RouteStage
from ed_uav_mission.d_task_reducer import TargetSnapshot, VehicleSnapshot
from ed_uav_mission.payload_config import PayloadBoundaryConfig
from ed_uav_mission.touchdown import ContactObservation, ContactState, TouchdownUpdate


def selection(task: DTaskKind) -> DTaskSelection:
    return DTaskSelection(
        mission_id="simulation-d2026",
        mission_profile_id="d2026-profile",
        deployment_preset_id="simulation",
        target_revision="d2026-circle-cross-v1",
        task=task,
        committed_at_s=0.0,
    )


def payload_config() -> PayloadBoundaryConfig:
    return PayloadBoundaryConfig(
        contract_version=1,
        freshness_timeout_s=0.2,
        actuator_timeout_s=0.5,
        minimum_standoff_m=0.5,
        contact_dwell_s=5.0,
        minimum_vehicle_speed_m_s=0.05,
    )


def vehicle(now_s: float, stage: RouteStage = RouteStage.START) -> VehicleSnapshot:
    return VehicleSnapshot(
        observed_at_s=now_s,
        sequence=round(now_s * 10) + 1,
        started=True,
        heartbeat_alive=True,
        speed_m_s=0.2,
        displacement_m=now_s,
        heading_rad=0.0,
        yaw_rate_rad_s=0.0,
        route_stage=stage,
    )


def target(now_s: float) -> TargetSnapshot:
    return TargetSnapshot(
        observed_at_s=now_s,
        sequence=round(now_s * 10) + 1,
        valid=True,
        relative_x_m=0.1,
        relative_y_m=-0.1,
        relative_z_m=1.5,
        relative_error_m=0.15,
    )


def contact_update(now_s: float, sequence: int) -> TouchdownUpdate:
    contact = ContactObservation(
        sequence=sequence,
        state=ContactState.VEHICLE,
        stable=True,
        owner="task2",
        frame_id="base_link",
        observed_at_monotonic_s=now_s,
    )
    return TouchdownUpdate(
        now_monotonic_s=now_s,
        target_observed_at_s=now_s - 0.1,
        vehicle_observed_at_s=now_s - 0.1,
        vehicle_speed_m_s=0.2,
        contact=contact,
        cancelled=False,
    )


def stale_target(now_s: float) -> TargetSnapshot:
    return replace(target(now_s), observed_at_s=now_s - 0.21)
