from __future__ import annotations

import runpy
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import pytest

REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
BRINGUP_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_bringup"
MISSION_SOURCE = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_mission" / "ed_uav_mission" / "executor.py"
POLICY_PATH = BRINGUP_ROOT / "security" / "fcu_command.policy.xml"
SETUP_PATH = BRINGUP_ROOT / "setup.py"
ACTION_NAME = "fcu/flight_command"


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
        {"path": "/ed_uav_fcu_bridge"},
        {"path": "/ed_uav_mission_executor"},
    ]
    assert [profile.attrib for profile in profiles] == [
        {"ns": "/", "node": "ed_uav_fcu_bridge"},
        {"ns": "/", "node": "mission_executor"},
    ]
    assert [actions.attrib for actions in profiles[0].findall("actions")] == [
        {"execute": "ALLOW"},
    ]
    assert [actions.attrib for actions in profiles[1].findall("actions")] == [
        {"call": "ALLOW"},
    ]
    assert len(execute_grants) == 1
    assert len(call_grants) == 1
    assert [grant.findtext("action") for grant in execute_grants] == [ACTION_NAME]
    assert [grant.findtext("action") for grant in call_grants] == [ACTION_NAME]
    assert len(root.findall(".//actions")) == 2
    assert len(root.findall(".//action")) == 2
    assert root.findall(".//{http://www.w3.org/2001/XInclude}include") == []

    mission_source = MISSION_SOURCE.read_text(encoding="utf-8")
    assert 'super().__init__("mission_executor")' in mission_source

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
