"""Validation helpers for target-node ROS input contracts."""

from __future__ import annotations

import math

from ed_uav_interfaces.msg import VehicleTelemetry
from sensor_msgs.msg import CameraInfo, Image

from ed_uav_perception.target_types import RejectReason

_UINT32_MODULUS = 1 << 32
_UINT32_HALF_RANGE = 1 << 31


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def validate_vehicle(
    message: VehicleTelemetry,
    previous_sequence: int | None,
    previous_acquisition_sec: float | None,
) -> RejectReason | None:
    """Validate one telemetry message before it replaces cached context."""
    if message.contract_version != VehicleTelemetry.CONTRACT_VERSION:
        return RejectReason.VEHICLE_CONTRACT_VERSION
    if not message.heartbeat_alive:
        return RejectReason.VEHICLE_HEARTBEAT_LOST
    if message.frame_id != "vehicle_start":
        return RejectReason.WRONG_VEHICLE_FRAME
    if message.motion_kind not in (
        VehicleTelemetry.MOTION_DISPLACEMENT,
        VehicleTelemetry.MOTION_WHEEL_SPEED,
    ) or message.turn_class not in (
        VehicleTelemetry.TURN_STRAIGHT,
        VehicleTelemetry.TURN_SMALL,
        VehicleTelemetry.TURN_LARGE,
    ):
        return RejectReason.INVALID_VEHICLE_CONTEXT
    values = (
        float(message.displacement_m),
        float(message.wheel_speed_m_s),
        float(message.heading_rad),
        float(message.yaw_rate_rad_s),
    )
    if (
        not all(math.isfinite(value) for value in values)
        or message.displacement_m < 0.0
        or message.wheel_speed_m_s < 0.0
        or not -math.pi <= message.heading_rad <= math.pi
        or abs(message.yaw_rate_rad_s) > 10.0
    ):
        return RejectReason.INVALID_VEHICLE_CONTEXT
    if message.turn_class == VehicleTelemetry.TURN_STRAIGHT:
        if abs(message.yaw_rate_rad_s) > 0.15:
            return RejectReason.INVALID_VEHICLE_CONTEXT
    elif abs(message.yaw_rate_rad_s) < 0.01:
        return RejectReason.INVALID_VEHICLE_CONTEXT
    if previous_sequence is not None:
        delta = (int(message.source_sequence) - previous_sequence) % _UINT32_MODULUS
        if delta == 0 or delta >= _UINT32_HALF_RANGE:
            return RejectReason.REPLAYED_VEHICLE_SEQUENCE
    acquisition_sec = stamp_seconds(message.acquisition_stamp)
    if (
        previous_acquisition_sec is not None
        and acquisition_sec < previous_acquisition_sec - 0.5
    ):
        return RejectReason.VEHICLE_ACQUISITION_REGRESSION
    return None


def validate_camera_binding(info: CameraInfo, image: Image) -> RejectReason | None:
    """Require CameraInfo to describe this exact acquired image frame."""
    if not image.header.frame_id or info.header.frame_id != image.header.frame_id:
        return RejectReason.CAMERA_INFO_FRAME_MISMATCH
    if int(info.width) != int(image.width) or int(info.height) != int(image.height):
        return RejectReason.CAMERA_INFO_RASTER_MISMATCH
    if (
        info.header.stamp.sec != image.header.stamp.sec
        or info.header.stamp.nanosec != image.header.stamp.nanosec
    ):
        return RejectReason.CAMERA_INFO_STAMP_MISMATCH
    return None
