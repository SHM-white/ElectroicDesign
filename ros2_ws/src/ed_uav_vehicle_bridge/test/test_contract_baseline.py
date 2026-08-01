from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
INTERFACES_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_interfaces"
BRIDGE_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_vehicle_bridge"
MISSION_EXECUTOR = (
    REPOSITORY_ROOT
    / "ros2_ws"
    / "src"
    / "ed_uav_mission"
    / "ed_uav_mission"
    / "executor.py"
)


def test_vehicle_telemetry_contract_remains_owned_and_typed() -> None:
    # Given: the D-task vehicle telemetry interface.
    contract = (INTERFACES_ROOT / "msg" / "VehicleTelemetry.msg").read_text(
        encoding="utf-8"
    )

    # When: its wire-facing fields are characterized before bridge work.
    fields = {
        line.split(maxsplit=1)[1]
        for line in contract.splitlines()
        if line
        and not line.startswith("#")
        and "=" not in line.split(maxsplit=1)[1]
    }

    # Then: the bridge can map once without changing the shared interface.
    assert fields == {
        "contract_version",
        "start_stamp",
        "acquisition_stamp",
        "source_sequence",
        "checksum_crc16",
        "vehicle_id",
        "start_event",
        "heartbeat_alive",
        "motion_kind",
        "displacement_m",
        "wheel_speed_m_s",
        "heading_rad",
        "yaw_rate_rad_s",
        "turn_class",
        "route_stage",
        "lap_complete",
        "frame_id",
    }


def test_mission_surfaces_remain_mission_owned() -> None:
    # Given: the existing mission executor and D-task selection contract.
    executor = MISSION_EXECUTOR.read_text(encoding="utf-8")
    selection = (INTERFACES_ROOT / "srv" / "SelectDTaskMission.srv").read_text(
        encoding="utf-8"
    )

    # When: ownership and authority surfaces are characterized.
    # Then: the vehicle bridge must call mission and never bypass it for FCU control.
    assert 'ActionServer(\n            self, ExecuteMission, "/mission/execute"' in executor
    assert 'ActionClient(\n            self, FlightCommand, "/fcu/flight_command"' in executor
    assert "The server must reject requests after arming" in selection


def test_bridge_protocol_uses_current_d_task_wire_contract() -> None:
    # Given
    protocol = (BRIDGE_ROOT / "ed_uav_vehicle_bridge" / "protocol.py").read_text(
        encoding="utf-8"
    )
    payloads = (BRIDGE_ROOT / "ed_uav_vehicle_bridge" / "payloads.py").read_text(
        encoding="utf-8"
    )
    models = (BRIDGE_ROOT / "ed_uav_vehicle_bridge" / "models.py").read_text(
        encoding="utf-8"
    )

    # When / Then
    assert "MAGIC: Final = 0x4454" in protocol
    assert 'HEADER: Final = struct.Struct("<HBBHIIII")' in protocol
    assert "MAX_PAYLOAD_BYTES: Final = 64" in protocol
    assert "HMAC_TAG_BYTES: Final = 8" in protocol
    assert 'CAR_TELEMETRY: Final = struct.Struct("<BBBHHihhH")' in payloads
    # TASK_SELECTION 含 mode 字节: <IIBB = selection_id + car_boot_id + task + mode
    # (1=实飞, 2=模拟飞); 旧 12B 帧由 TASK_SELECTION_LEGACY 兼容
    assert 'TASK_SELECTION: Final = struct.Struct("<IIBB")' in payloads
    assert 'TASK_SELECTION_LEGACY: Final = struct.Struct("<IIB")' in payloads
    assert "encode_task_selection" in payloads
    assert "mode must be 1 or 2" in payloads
    assert 'MISSION_STATUS: Final = struct.Struct("<IIIBBHH")' in payloads
    assert "SelectionAckValue" not in models
