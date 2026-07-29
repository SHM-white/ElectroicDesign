"""Trust-boundary adapters for D-task telemetry and PnP observations."""

from __future__ import annotations

import math

from ed_uav_interfaces.msg import TargetObservation, VehicleTelemetry

from ed_uav_mission.d_task_events import TargetSnapshot, VehicleSnapshot
from ed_uav_mission.d_task_model import RouteStage


class DTaskInputError(ValueError):
    """A ROS contract could not be adapted into the mission domain."""


def adapt_vehicle_telemetry(
    message: VehicleTelemetry,
    observed_at_s: float,
) -> VehicleSnapshot:
    """Parse one vehicle message into finite monotonic mission state."""
    values = (
        observed_at_s,
        message.displacement_m,
        message.wheel_speed_m_s,
        message.heading_rad,
        message.yaw_rate_rad_s,
    )
    if message.contract_version != VehicleTelemetry.CONTRACT_VERSION:
        raise DTaskInputError("unsupported vehicle telemetry contract")
    if not all(math.isfinite(value) for value in values):
        raise DTaskInputError("vehicle telemetry contains nonfinite values")
    try:
        stage = RouteStage(message.route_stage)
    except ValueError as error:
        raise DTaskInputError("vehicle route stage is invalid") from error
    return VehicleSnapshot(
        observed_at_s=observed_at_s,
        sequence=int(message.source_sequence),
        started=bool(message.start_event),
        heartbeat_alive=bool(message.heartbeat_alive),
        speed_m_s=abs(float(message.wheel_speed_m_s)),
        displacement_m=float(message.displacement_m),
        heading_rad=float(message.heading_rad),
        yaw_rate_rad_s=float(message.yaw_rate_rad_s),
        route_stage=stage,
    )


def adapt_target_observation(
    message: TargetObservation,
    observed_at_s: float,
    expected_revision: str,
) -> TargetSnapshot:
    """Parse PnP pose fields into a typed relative tracking error."""
    position = message.pose.pose.position
    values = (observed_at_s, position.x, position.y, position.z, message.quality)
    if message.contract_version != TargetObservation.CONTRACT_VERSION:
        raise DTaskInputError("unsupported target observation contract")
    if message.target_revision != expected_revision:
        raise DTaskInputError("target revision does not match mission profile")
    if not all(math.isfinite(value) for value in values):
        raise DTaskInputError("target observation contains nonfinite values")
    return TargetSnapshot(
        observed_at_s=observed_at_s,
        sequence=int(message.source_sequence),
        valid=bool(message.valid and message.status == TargetObservation.STATUS_VALID),
        relative_x_m=float(position.x),
        relative_y_m=float(position.y),
        relative_z_m=float(position.z),
        relative_error_m=math.hypot(float(position.x), float(position.y)),
        rejection_reason=str(message.rejection_reason),
    )
