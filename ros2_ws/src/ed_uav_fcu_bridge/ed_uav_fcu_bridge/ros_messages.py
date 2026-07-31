"""ROS message builders for FCU bridge telemetry publications."""

from __future__ import annotations

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ed_uav_interfaces.msg import FcuState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

from .telemetry import TelemetrySnapshot


def state_message(snapshot: TelemetrySnapshot, stamp: Time) -> FcuState:
    message = FcuState()
    message.acquisition_stamp = stamp
    message.source = FcuState.SOURCE_V7
    message.frame_id = "base_link"
    message.communication_ok = snapshot.link.valid
    message.altitude_m = snapshot.altitude_m or 0.0
    message.battery_voltage_v = snapshot.battery_voltage_v or 0.0
    message.aux1_us = snapshot.aux1_us
    message.aux1_valid = snapshot.aux1_valid
    message.task3_control_allowed = snapshot.task3_control_allowed
    message.emergency_lock_active = snapshot.emergency_lock_active
    if snapshot.position is not None:
        message.source_sequence = snapshot.position.source_sequence
        message.optical_flow_position_m.x = snapshot.position.right_m
        message.optical_flow_position_m.y = snapshot.position.forward_m
    else:
        message.source_sequence = snapshot.link.source_sequence
    if snapshot.status is not None:
        message.mode = snapshot.status.mode
        message.motors_armed = snapshot.status.motors_armed
    return message


def battery_message(snapshot: TelemetrySnapshot, stamp: Time) -> BatteryState | None:
    if snapshot.battery_voltage_v is None:
        return None
    message = BatteryState()
    message.header.stamp = stamp
    message.header.frame_id = "base_link"
    message.voltage = snapshot.battery_voltage_v
    return message


def odom_message(snapshot: TelemetrySnapshot, stamp: Time) -> Odometry | None:
    position = snapshot.position
    if position is None or not position.valid:
        return None
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    message.pose.pose.position.x = position.right_m
    message.pose.pose.position.y = position.forward_m
    message.pose.pose.orientation.w = 1.0
    return message


def diagnostic_message(snapshot: TelemetrySnapshot, stamp: Time) -> DiagnosticArray | None:
    diagnostic = snapshot.flow_diagnostic
    if diagnostic is None:
        return None
    message = DiagnosticArray()
    message.header.stamp = stamp
    status = DiagnosticStatus()
    status.name = "ed_uav_fcu_bridge/v7_0x51"
    status.hardware_id = "lingxiao-v7"
    status.level = DiagnosticStatus.OK
    status.message = "separate optical-flow diagnostic"
    status.values = [
        KeyValue(key="source", value="V7_0x51"),
        KeyValue(key="source_sequence", value=str(diagnostic.source_sequence)),
        KeyValue(key="mode", value=str(diagnostic.mode)),
        KeyValue(key="state", value=str(diagnostic.state)),
    ]
    message.status.append(status)
    return message
