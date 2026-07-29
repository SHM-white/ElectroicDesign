from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
INTERFACES_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_interfaces"
MISSION_EXECUTOR = (
    REPOSITORY_ROOT
    / "ros2_ws"
    / "src"
    / "ed_uav_mission"
    / "ed_uav_mission"
    / "executor.py"
)


def test_vehicle_telemetry_contract_remains_owned_and_typed() -> None:
    # Given: the Todo 1 vehicle telemetry interface.
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
        "turn_class",
        "route_stage",
        "lap_complete",
        "frame_id",
    }


def test_mission_surfaces_remain_mission_owned() -> None:
    # Given: the existing mission executor and Todo 1 selection contract.
    executor = MISSION_EXECUTOR.read_text(encoding="utf-8")
    selection = (INTERFACES_ROOT / "srv" / "SelectDTaskMission.srv").read_text(
        encoding="utf-8"
    )

    # When: ownership and authority surfaces are characterized.
    # Then: the vehicle bridge must call mission and never bypass it for FCU control.
    assert 'ActionServer(\n            self, ExecuteMission, "/mission/execute"' in executor
    assert 'ActionClient(\n            self, FlightCommand, "/fcu/flight_command"' in executor
    assert "The server must reject requests after arming" in selection
