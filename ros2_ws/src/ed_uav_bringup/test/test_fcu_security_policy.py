from __future__ import annotations

import runpy
import xml.etree.ElementTree as ET
import ast
from pathlib import Path
from unittest.mock import patch

import pytest

REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
BRINGUP_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_bringup"
MISSION_SOURCE = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_mission" / "ed_uav_mission" / "executor.py"
POLICY_PATH = BRINGUP_ROOT / "security" / "fcu_command.policy.xml"
SETUP_PATH = BRINGUP_ROOT / "setup.py"
FCU_DRY_RUN_LAUNCH = BRINGUP_ROOT / "launch" / "fcu_dry_run.launch.py"
MISSION_LAUNCH = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_mission" / "launch" / "mission_executor.launch.py"
ACTION_NAME = "fcu/flight_command"
MISSION_ACTION_NAME = "mission/execute"
BRIDGE_ENCLAVE = "/ed_uav_fcu_bridge"
MISSION_ENCLAVE = "/ed_uav_mission_executor"
BRIDGE_NODE = "ed_uav_fcu_bridge"
MISSION_NODE = "mission_executor"


def _profile_topics(profile: ET.Element, permission: str) -> set[str]:
    return {
        topic.text or ""
        for topics in profile.findall(f'topics[@{permission}="ALLOW"]')
        for topic in topics.findall("topic")
    }


def _profile_actions(profile: ET.Element, permission: str) -> set[str]:
    return {
        action.text or ""
        for actions in profile.findall(f'actions[@{permission}="ALLOW"]')
        for action in actions.findall("action")
    }


def _node_arguments_for(
    package_name: str,
    executable_name: str,
    launch_path: Path = FCU_DRY_RUN_LAUNCH,
) -> list[str]:
    tree = ast.parse(launch_path.read_text(encoding="utf-8"))
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call) or getattr(call.func, "id", None) != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in call.keywords}
        package = keywords.get("package")
        executable = keywords.get("executable")
        if not (
            isinstance(package, ast.Constant)
            and package.value == package_name
            and isinstance(executable, ast.Constant)
            and executable.value == executable_name
        ):
            continue
        arguments = keywords.get("arguments")
        assert isinstance(arguments, ast.List)
        return [element.value for element in arguments.elts if isinstance(element, ast.Constant)]
    raise AssertionError(f"missing launch_ros Node for {package_name}/{executable_name}")


def test_fcu_policy_grants_only_bridge_execute_and_mission_call() -> None:
    # Given: the package-owned, self-contained FCU policy template.
    assert POLICY_PATH.is_file(), f"missing policy template: {POLICY_PATH}"

    # When: the policy is parsed as XML rather than matched as text.
    root = ET.parse(POLICY_PATH).getroot()
    enclaves = root.findall("./enclaves/enclave")
    profiles = root.findall("./enclaves/enclave/profiles/profile")
    execute_grants = root.findall('.//actions[@execute="ALLOW"]')
    call_grants = root.findall('.//actions[@call="ALLOW"]')

    # Then: only the bridge executes and only the mission executor calls the action.
    assert root.attrib == {"version": "0.2.0"}
    assert [enclave.attrib for enclave in enclaves] == [
        {"path": BRIDGE_ENCLAVE},
        {"path": MISSION_ENCLAVE},
    ]
    assert [profile.attrib for profile in profiles] == [
        {"ns": "/", "node": BRIDGE_NODE},
        {"ns": "/", "node": MISSION_NODE},
    ]
    assert len(execute_grants) == 2
    assert len(call_grants) == 1
    assert _profile_actions(profiles[0], "execute") == {ACTION_NAME}
    assert _profile_actions(profiles[1], "call") == {ACTION_NAME}
    assert _profile_actions(profiles[1], "execute") == {MISSION_ACTION_NAME}
    assert [grant.findtext("action") for grant in call_grants] == [ACTION_NAME]
    assert len(root.findall(".//actions")) == 3
    assert len(root.findall(".//action")) == 3
    assert root.findall(".//{http://www.w3.org/2001/XInclude}include") == []

    assert _profile_topics(profiles[0], "publish") == {
        "rosout",
        "/parameter_events",
        "/fcu/state",
        "/fcu/battery",
        "/fcu/optical_flow/odom",
        "/fcu/diagnostics",
    }
    assert _profile_topics(profiles[0], "subscribe") == set()
    assert _profile_topics(profiles[1], "publish") == {"rosout", "/parameter_events"}
    assert _profile_topics(profiles[1], "subscribe") == {
        "/fcu/state",
        "/localization/status",
    }

    mission_source = MISSION_SOURCE.read_text(encoding="utf-8")
    assert f'super().__init__("{MISSION_NODE}")' in mission_source
    assert f'ActionServer(\n            self, ExecuteMission, "/{MISSION_ACTION_NAME}"' in mission_source
    assert f'ActionClient(\n            self, FlightCommand, "/{ACTION_NAME}"' in mission_source
    assert 'create_subscription(FcuState, "/fcu/state"' in mission_source
    assert 'create_subscription(\n            LocalizationStatus, "/localization/status"' in mission_source

    policy_text = POLICY_PATH.read_text(encoding="utf-8").casefold()
    for forbidden in (
        "private key",
        "private_key",
        "begin private",
        "certificate",
        "token",
    ):
        assert forbidden not in policy_text


def test_setup_installs_policy_template(monkeypatch: pytest.MonkeyPatch) -> None:
    # Given: setup.py evaluated from the package directory.
    monkeypatch.chdir(BRINGUP_ROOT)

    # When: setuptools receives the package installation declaration.
    with patch("setuptools.setup") as setup_mock:
        runpy.run_path(str(SETUP_PATH))

    # Then: the policy template is installed in the package share directory.
    data_files = setup_mock.call_args.kwargs["data_files"]
    assert (f"share/{BRINGUP_ROOT.name}/security", ["security/fcu_command.policy.xml"]) in data_files


def test_fcu_bridge_launch_uses_explicit_policy_enclave() -> None:
    # Given: dry-run is the package-owned launch surface that starts the bridge.
    arguments = _node_arguments_for("ed_uav_fcu_bridge", "ed_uav_fcu_bridge")

    # When: the launch arguments are compared with the policy enclave path.
    # Then: SROS2 does not depend on node-name-to-enclave inference.
    assert arguments == ["--ros-args", "--enclave", BRIDGE_ENCLAVE]


def test_mission_launch_uses_explicit_policy_enclave() -> None:
    arguments = _node_arguments_for(
        "ed_uav_mission",
        "mission_executor",
        MISSION_LAUNCH,
    )

    assert arguments == ["--ros-args", "--enclave", MISSION_ENCLAVE]


def test_setup_registers_pytest_tests_for_colcon(monkeypatch: pytest.MonkeyPatch) -> None:
    # Given: setup.py evaluated from the package directory.
    monkeypatch.chdir(BRINGUP_ROOT)

    # When: setuptools receives the package installation declaration.
    with patch("setuptools.setup") as setup_mock:
        runpy.run_path(str(SETUP_PATH))

    # Then: normal colcon test discovery has the pytest dependency metadata.
    assert setup_mock.call_args.kwargs["tests_require"] == ["pytest"]
