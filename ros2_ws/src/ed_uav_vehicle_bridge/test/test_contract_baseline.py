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


def test_bridge_docs_describe_current_d_task_telemetry() -> None:
    # Given
    protocol = (BRIDGE_ROOT / "PROTOCOL.md").read_text(encoding="utf-8")
    mapping = (BRIDGE_ROOT / "ed_uav_vehicle_bridge" / "ros_mapping.py").read_text(
        encoding="utf-8"
    )

    # When / Then
    assert "`>HBBffffBB`" in protocol
    assert "heading_rad" in protocol
    assert "yaw_rate_rad_s" in protocol
    assert "Todo 1" not in protocol
    assert "Todo 1" not in mapping
