"""Strict typed models for external D-task contract documents."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from ipaddress import ip_network
from math import pi
from typing import Annotated, Final, Literal, TypeAlias

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    FiniteFloat,
    IPvAnyAddress,
    StringConstraints,
    TypeAdapter,
    model_validator,
)
from typing_extensions import Self


Identifier = Annotated[
    str,
    StringConstraints(pattern=r"^[a-z][a-z0-9-]{0,63}$", min_length=1, max_length=64),
]
TargetRevision = Literal["d2026-circle-cross-v1"]


@dataclass(frozen=True, slots=True)
class DTaskSemanticError(ValueError):
    reason: str

    def __str__(self) -> str:
        return self.reason


class StrictContractModel(BaseModel):
    """Reject undeclared fields and mutation after boundary parsing."""

    model_config = ConfigDict(extra="forbid", frozen=True)


class Units(StrictContractModel):
    length: Literal["m"]
    speed: Literal["m/s"]
    time: Literal["s"]


class Frame(StrictContractModel):
    id: Literal["map"]
    convention: Literal["ENU"]


class Freshness(StrictContractModel):
    vehicle_telemetry_ms: int = Field(gt=0, le=2000)
    target_observation_ms: int = Field(gt=0, le=2000)
    mission_status_ms: int = Field(gt=0, le=5000)


class Timing(StrictContractModel):
    takeoff_deadline_s: FiniteFloat = Field(gt=0.0, le=15.0)
    mission_deadline_s: FiniteFloat = Field(gt=0.0, le=90.0)
    vehicle_dwell_s: FiniteFloat = Field(ge=5.0, le=15.0)


class MissionProfile(StrictContractModel):
    version: Literal[1]
    mission_profile_id: Identifier
    task: Literal["payload_drop", "dynamic_landing"]
    owner: Literal["ed_uav_mission"]
    units: Units
    frame: Frame
    freshness: Freshness
    target_revision: TargetRevision
    route_order: tuple[Literal["B"], Literal["D"], Literal["A"]]
    cruise_altitude_m: FiniteFloat = Field(ge=1.4, le=1.6)
    timing: Timing


class TargetGeometry(StrictContractModel):
    outer_diameter_m: Literal[0.5]
    inner_diameter_m: Literal[0.3]
    line_width_m: FiniteFloat = Field(ge=0.018, le=0.022)
    pattern: Literal["concentric_circles_cross"]


class TargetProfile(StrictContractModel):
    version: Literal[1]
    target_revision: TargetRevision
    owner: Literal["wheel_vehicle_platform"]
    frame_id: Literal["vehicle_platform"]
    units: Literal["m"]
    geometry: TargetGeometry


class DocumentationDeployment(StrictContractModel):
    version: Literal[1]
    preset_kind: Literal["documentation_example"]
    preset_id: Identifier
    mission_profile_id: Identifier
    target_revision: TargetRevision
    requires_local_manifest: Literal[True]
    local_manifest: Literal["deployment_preset.local.yaml"]


class Mid360Deployment(StrictContractModel):
    owner: Literal["ed_uav_lidar"]
    serial: Annotated[str, StringConstraints(min_length=4, max_length=64)]
    sensor_ip: IPvAnyAddress
    host_ip: IPvAnyAddress
    firmware: Annotated[str, StringConstraints(min_length=2, max_length=64)]


class GroundStationDeployment(StrictContractModel):
    owner: Literal["ground_station_esp32s3"]
    transport: Literal["esp_now"]
    peer_id: Annotated[
        str,
        StringConstraints(pattern=r"^[0-9A-F]{2}(?::[0-9A-F]{2}){5}$", max_length=17),
    ]


_DOCUMENTATION_NETWORKS: Final = (
    ip_network("192.0.2.0/24"),
    ip_network("198.51.100.0/24"),
    ip_network("203.0.113.0/24"),
)
_PLACEHOLDER_VALUES: Final = frozenset(
    {"replace_me", "placeholder", "unknown", "changeme"}
)
_ZERO_ESP_PEER: Final = "00:00:00:00:00:00"
_UINT32_MODULUS: Final = 1 << 32
_UINT32_HALF_RANGE: Final = 1 << 31


class FieldDeployment(StrictContractModel):
    version: Literal[1]
    preset_kind: Literal["field"]
    preset_id: Identifier
    mission_profile_id: Identifier
    target_revision: TargetRevision
    mid360: Mid360Deployment
    ground_station: GroundStationDeployment

    @model_validator(mode="after")
    def reject_placeholders(self) -> Self:
        if self.ground_station.peer_id == _ZERO_ESP_PEER:
            raise DTaskSemanticError(
                reason="placeholder deployment value: zero ESP peer"
            )
        text_values = (
            self.mid360.serial,
            self.mid360.firmware,
            self.ground_station.peer_id,
        )
        if any(value.strip().lower() in _PLACEHOLDER_VALUES for value in text_values):
            raise DTaskSemanticError(reason="placeholder deployment value")
        addresses = (self.mid360.sensor_ip, self.mid360.host_ip)
        if any(
            address.version == 4
            and any(address in network for network in _DOCUMENTATION_NETWORKS)
            for address in addresses
        ):
            raise DTaskSemanticError(
                reason="placeholder deployment value: documentation IP range"
            )
        return self


Deployment: TypeAlias = Annotated[
    DocumentationDeployment | FieldDeployment,
    Field(discriminator="preset_kind"),
]


class RouteStage(str, Enum):
    START = "START"
    B = "B"
    D = "D"
    A = "A"
    COMPLETE = "COMPLETE"


class VehicleTelemetryPayload(StrictContractModel):
    started: bool
    heartbeat: bool
    motion_kind: Literal["displacement", "wheel_speed"]
    displacement_m: FiniteFloat
    wheel_speed_m_s: FiniteFloat = Field(ge=0.0)
    turn_class: Literal["straight", "small_turn", "large_turn"]
    heading_rad: FiniteFloat = Field(ge=-pi, le=pi)
    yaw_rate_rad_s: FiniteFloat = Field(ge=-10.0, le=10.0)
    route_stage: RouteStage
    lap_complete: bool

    @model_validator(mode="after")
    def require_completion_consistency(self) -> Self:
        if self.lap_complete != (self.route_stage is RouteStage.COMPLETE):
            raise DTaskSemanticError(reason="lap_complete requires route_stage COMPLETE")
        if self.turn_class == "straight" and abs(self.yaw_rate_rad_s) > 0.15:
            raise DTaskSemanticError(reason="invalid straight turn yaw rate")
        if self.turn_class != "straight" and abs(self.yaw_rate_rad_s) < 0.01:
            raise DTaskSemanticError(reason="turn requires signed yaw rate")
        return self


class Esp32Frame(StrictContractModel):
    version: Literal[1]
    frame_type: Literal["vehicle_telemetry"]
    sequence: int = Field(ge=0, le=4_294_967_295)
    acquisition_time_ms: int = Field(ge=0)
    checksum_crc16: int = Field(ge=0, le=65_535)
    payload: VehicleTelemetryPayload


class Esp32FrameWindow(StrictContractModel):
    version: Literal[1]
    owner: Literal["wheel-vehicle"]
    freshness_limit_ms: int = Field(gt=0, le=2000)
    frames: tuple[Esp32Frame, ...] = Field(min_length=1, max_length=256)

    @model_validator(mode="after")
    def require_ordered_fresh_frames(self) -> Self:
        route_order = tuple(RouteStage)
        seen_sequences: set[int] = set()
        previous_sequence: int | None = None
        previous_time = -1
        previous_stage_index = 0
        for frame in self.frames:
            if frame.sequence in seen_sequences:
                raise DTaskSemanticError(
                    reason=f"duplicate telemetry sequence: {frame.sequence}"
                )
            if previous_sequence is not None:
                sequence_delta = (
                    frame.sequence - previous_sequence
                ) % _UINT32_MODULUS
                if sequence_delta >= _UINT32_HALF_RANGE:
                    raise DTaskSemanticError(
                        reason=f"stale telemetry sequence: {frame.sequence} "
                        f"after {previous_sequence}"
                    )
            if frame.acquisition_time_ms <= previous_time:
                raise DTaskSemanticError(
                    reason="stale telemetry acquisition time: "
                    f"{frame.acquisition_time_ms} <= {previous_time}"
                )
            gap_ms = frame.acquisition_time_ms - previous_time
            if previous_time >= 0 and gap_ms > self.freshness_limit_ms:
                raise DTaskSemanticError(
                    reason=f"stale telemetry gap: {gap_ms} > {self.freshness_limit_ms} ms"
                )
            stage_index = route_order.index(frame.payload.route_stage)
            if stage_index not in (previous_stage_index, previous_stage_index + 1):
                previous_stage = route_order[previous_stage_index]
                raise DTaskSemanticError(
                    reason=f"invalid route order: {previous_stage.value} -> "
                    f"{frame.payload.route_stage.value}"
                )
            seen_sequences.add(frame.sequence)
            previous_sequence = frame.sequence
            previous_time = frame.acquisition_time_ms
            previous_stage_index = stage_index
        return self


MISSION_ADAPTER: Final = TypeAdapter(MissionProfile)
TARGET_ADAPTER: Final = TypeAdapter(TargetProfile)
DEPLOYMENT_ADAPTER: Final = TypeAdapter(Deployment)
ESP32_FRAMES_ADAPTER: Final = TypeAdapter(Esp32FrameWindow)
