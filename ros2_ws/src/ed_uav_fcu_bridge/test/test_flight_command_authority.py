"""Flight-command admission is explicit and independent of in-process SROS."""

from __future__ import annotations

import ast
from collections.abc import Callable, Mapping
from pathlib import Path

from ed_uav_fcu_bridge import node as bridge_node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_SOURCE = PACKAGE_ROOT / "ed_uav_fcu_bridge" / "node.py"
BRINGUP_LAUNCH_DIR = PACKAGE_ROOT.parent / "ed_uav_bringup" / "launch"
AUTHORITY_GUARD_NAME = "require_flight_command_authority"

AuthorityGuard = Callable[[bool, Mapping[str, str]], bool]


def _authority_guard() -> AuthorityGuard:
    guard = getattr(bridge_node, AUTHORITY_GUARD_NAME, None)
    assert callable(guard)
    return guard


def _is_call_named(candidate: ast.AST, name: str) -> bool:
    if not isinstance(candidate, ast.Call):
        return False
    return (
        getattr(candidate.func, "id", None) == name
        or getattr(candidate.func, "attr", None) == name
    )


def _contains_call(candidate: ast.AST, name: str) -> bool:
    return any(_is_call_named(descendant, name) for descendant in ast.walk(candidate))


def test_explicit_operator_setting_is_the_only_local_admission_input() -> None:
    malformed_security = {
        "ROS_SECURITY_ENABLE": "false",
        "ROS_SECURITY_STRATEGY": "Permissive",
        "ROS_SECURITY_KEYSTORE": "/does/not/exist",
    }

    assert _authority_guard()(False, malformed_security) is False
    assert _authority_guard()(True, malformed_security) is True
    assert not hasattr(bridge_node, "FlightCommandAuthorityError")


def test_node_parameter_defaults_flight_commands_to_disabled() -> None:
    tree = ast.parse(NODE_SOURCE.read_text(encoding="utf-8"))
    declarations = [
        call
        for call in ast.walk(tree)
        if _is_call_named(call, "declare_parameter")
        and len(call.args) >= 2
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == "enable_flight_commands"
    ]

    assert len(declarations) == 1
    assert isinstance(declarations[0].args[1], ast.Constant)
    assert declarations[0].args[1].value is False


def test_explicit_setting_directly_controls_action_server_creation() -> None:
    tree = ast.parse(NODE_SOURCE.read_text(encoding="utf-8"))
    authority_assignments = [
        statement
        for statement in ast.walk(tree)
        if isinstance(statement, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "commands_enabled"
            for target in statement.targets
        )
        and _contains_call(statement.value, AUTHORITY_GUARD_NAME)
    ]
    guarded_servers = [
        conditional
        for conditional in ast.walk(tree)
        if isinstance(conditional, ast.If)
        and isinstance(conditional.test, ast.Name)
        and conditional.test.id == "commands_enabled"
        and any(_contains_call(statement, "ActionServer") for statement in conditional.body)
    ]

    assert authority_assignments
    assert guarded_servers


def test_primary_competition_launch_contains_no_sros_runtime_wiring() -> None:
    source = (BRINGUP_LAUNCH_DIR / "full_competition.launch.py").read_text(
        encoding="utf-8"
    )

    for token in (
        "ROS_SECURITY_ENABLE",
        "ROS_SECURITY_STRATEGY",
        "ROS_SECURITY_KEYSTORE",
        "--enclave",
        "programmable_capability_report",
    ):
        assert token not in source
    assert '"enable_flight_commands": True' in source
    assert '"enable_realtime_control": True' in source
