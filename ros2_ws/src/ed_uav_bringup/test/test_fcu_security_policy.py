"""SROS2 artifacts are optional references, never runtime admission gates."""

from __future__ import annotations

from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
BRINGUP_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_bringup"
MISSION_LAUNCH = (
    REPOSITORY_ROOT
    / "ros2_ws"
    / "src"
    / "ed_uav_mission"
    / "launch"
    / "mission_executor.launch.py"
)
ACTIVE_LAUNCHES = (
    BRINGUP_ROOT / "launch" / "full_competition.launch.py",
    BRINGUP_ROOT / "launch" / "task3_flight_test.launch.py",
    BRINGUP_ROOT / "launch" / "fcu_dry_run.launch.py",
    MISSION_LAUNCH,
)


def test_active_launches_do_not_wire_sros_or_enclaves() -> None:
    forbidden = (
        "ROS_SECURITY_ENABLE",
        "ROS_SECURITY_STRATEGY",
        "ROS_SECURITY_KEYSTORE",
        "--enclave",
    )

    for launch_path in ACTIVE_LAUNCHES:
        source = launch_path.read_text(encoding="utf-8")
        for token in forbidden:
            assert token not in source, f"{launch_path.name} still wires {token}"


def test_optional_policy_template_contains_no_credentials() -> None:
    policy_path = BRINGUP_ROOT / "security" / "fcu_command.policy.xml"
    assert policy_path.is_file()
    policy_text = policy_path.read_text(encoding="utf-8").casefold()
    for forbidden in (
        "private key",
        "private_key",
        "begin private",
        "certificate",
        "token",
    ):
        assert forbidden not in policy_text
