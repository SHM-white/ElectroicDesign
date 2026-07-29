import ast
from pathlib import Path
import runpy
from unittest.mock import patch


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
SETUP_PATH = PACKAGE_ROOT / "setup.py"
NODE_PATH = PACKAGE_ROOT / "ed_uav_vehicle_bridge" / "node.py"
BRINGUP_LAUNCH = (
    REPOSITORY_ROOT
    / "ros2_ws"
    / "src"
    / "ed_uav_bringup"
    / "launch"
    / "vehicle_bridge.launch.py"
)


def test_setup_registers_bridge_and_fake_processes(monkeypatch) -> None:
    monkeypatch.chdir(PACKAGE_ROOT)
    with patch("setuptools.setup") as setup_mock:
        runpy.run_path(str(SETUP_PATH))
    scripts = setup_mock.call_args.kwargs["entry_points"]["console_scripts"]
    assert scripts == [
        "vehicle_bridge = ed_uav_vehicle_bridge.entrypoint:main",
        "fake_vehicle_source = ed_uav_vehicle_bridge.fake_source:main",
    ]


def test_node_graph_is_typed_and_has_no_fcu_command_authority() -> None:
    source = NODE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]

    for contract_type, graph_name in (
        ("VehicleTelemetry", "/d_task/vehicle/telemetry"),
        ("FcuState", "/fcu/state"),
        ("MissionStatus", "/d_task/mission_status"),
        ("SelectDTaskMission", "/d_task/pre_arm/select_mission"),
        ("ExecuteMission", "/mission/execute"),
    ):
        assert contract_type in source
        assert graph_name in source
    assert "MutuallyExclusiveCallbackGroup()" in source
    assert "SimpleQueue" in source
    assert all(getattr(call.func, "attr", "") != "create_service" for call in calls)
    assert "/fcu/" + "flight_command" not in source
    assert "Flight" + "Command" not in source


def test_bringup_registers_dedicated_bridge_launch() -> None:
    source = BRINGUP_LAUNCH.read_text(encoding="utf-8")
    assert 'get_package_share_directory("ed_uav_vehicle_bridge")' in source
    assert '"vehicle_bridge.launch.py"' in source
