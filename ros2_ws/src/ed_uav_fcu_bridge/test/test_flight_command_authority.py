"""Default-deny authority regressions for the FCU flight-command action."""

from __future__ import annotations

import ast
from collections.abc import Callable, Mapping
from pathlib import Path

import pytest
from ed_uav_fcu_bridge import node as bridge_node

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_SOURCE = PACKAGE_ROOT / "ed_uav_fcu_bridge" / "node.py"
BRINGUP_LAUNCH_DIR = PACKAGE_ROOT.parent / "ed_uav_bringup" / "launch"
AUTHORITY_GUARD_NAME = "require_flight_command_authority"


AuthorityGuard = Callable[[bool, Mapping[str, str]], bool]


def _authority_guard() -> AuthorityGuard:
    guard = getattr(bridge_node, AUTHORITY_GUARD_NAME, None)
    assert callable(guard), (
        f"ed_uav_fcu_bridge.node must expose callable {AUTHORITY_GUARD_NAME}"
    )
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


def test_existing_node_module_imports_before_authority_contract_is_checked() -> None:
    # Given: the existing ROS bridge module was imported during collection.
    # When: its public node class is inspected.
    node_class = bridge_node.FcuBridgeNode

    # Then: RED cannot be attributed to an import or collection failure.
    assert node_class.__module__ == "ed_uav_fcu_bridge.node"


def test_node_exposes_callable_flight_command_authority_guard() -> None:
    # Given: the imported bridge module.
    # When: the authority boundary is resolved without a direct missing import.
    guard = getattr(bridge_node, AUTHORITY_GUARD_NAME, None)

    # Then: production exposes the callable needed by node startup.
    assert callable(guard)


def test_disabled_flight_commands_require_no_sros2_configuration() -> None:
    # Given: flight commands are default-denied and no security variables exist.
    environment: Mapping[str, str] = {}

    # When: startup evaluates whether to expose the action.
    enabled = _authority_guard()(False, environment)

    # Then: the action remains disabled without credentials or a keystore.
    assert enabled is False


def test_enabled_flight_commands_require_sros2_enforce(tmp_path: Path) -> None:
    # Given: all caller-authorization preconditions are explicitly enforced.
    environment: Mapping[str, str] = {
        "ROS_SECURITY_ENABLE": "true",
        "ROS_SECURITY_STRATEGY": "Enforce",
        "ROS_SECURITY_KEYSTORE": str(tmp_path),
    }

    # When: startup evaluates whether to expose the action.
    enabled = _authority_guard()(True, environment)

    # Then: the action boundary may be created; SROS2 authorizes callers.
    assert enabled is True


@pytest.mark.parametrize(
    "environment",
    (
        {},
        {"ROS_SECURITY_ENABLE": "true"},
        {
            "ROS_SECURITY_ENABLE": "true",
            "ROS_SECURITY_STRATEGY": "Enforce",
        },
        {
            "ROS_SECURITY_ENABLE": "true",
            "ROS_SECURITY_STRATEGY": "Enforce",
            "ROS_SECURITY_KEYSTORE": "",
        },
        {
            "ROS_SECURITY_ENABLE": "true",
            "ROS_SECURITY_STRATEGY": "Permissive",
            "ROS_SECURITY_KEYSTORE": "/tmp/nonexistent-test-keystore",
        },
    ),
    ids=("unset", "partial", "missing-keystore", "empty-keystore", "permissive"),
)
def test_enabled_flight_commands_reject_incomplete_or_permissive_sros2(
    environment: Mapping[str, str],
) -> None:
    # Given: flight commands are enabled with incomplete or permissive security.
    guard = _authority_guard()
    authority_error = getattr(bridge_node, "FlightCommandAuthorityError", None)
    assert isinstance(authority_error, type)
    assert issubclass(authority_error, RuntimeError)

    # When: startup checks the action authority boundary.
    with pytest.raises(authority_error):
        guard(True, environment)

    # Then: the typed runtime error prevents action-server creation.


def test_enabled_flight_commands_reject_nonexistent_keystore_directory() -> None:
    # Given: security is enabled but the keystore path is only a string.
    environment: Mapping[str, str] = {
        "ROS_SECURITY_ENABLE": "true",
        "ROS_SECURITY_STRATEGY": "Enforce",
        "ROS_SECURITY_KEYSTORE": "/tmp/nonexistent-test-keystore",
    }

    # When: startup checks the action authority boundary.
    with pytest.raises(bridge_node.FlightCommandAuthorityError):
        _authority_guard()(True, environment)

    # Then: command serving cannot start without a real SROS2 keystore directory.


def test_enabled_flight_commands_reject_keystore_file(tmp_path: Path) -> None:
    # Given: security is enabled but the keystore path is a file, not a directory.
    keystore_file = tmp_path / "keystore-file"
    keystore_file.write_text("not a directory", encoding="utf-8")
    environment: Mapping[str, str] = {
        "ROS_SECURITY_ENABLE": "true",
        "ROS_SECURITY_STRATEGY": "Enforce",
        "ROS_SECURITY_KEYSTORE": str(keystore_file),
    }

    # When: startup checks the action authority boundary.
    with pytest.raises(bridge_node.FlightCommandAuthorityError):
        _authority_guard()(True, environment)

    # Then: command serving cannot mistake an arbitrary file for a keystore.


def test_node_parameter_defaults_flight_commands_to_disabled() -> None:
    # Given: the bridge node source tree.
    tree = ast.parse(NODE_SOURCE.read_text(encoding="utf-8"))

    # When: parameter declarations are inspected.
    declarations = [
        call
        for call in ast.walk(tree)
        if _is_call_named(call, "declare_parameter")
        and len(call.args) >= 2
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == "enable_flight_commands"
    ]

    # Then: omission is safe and cannot expose the hardware-capable action.
    assert len(declarations) == 1
    assert isinstance(declarations[0].args[1], ast.Constant)
    assert declarations[0].args[1].value is False


def test_action_server_creation_is_conditional_on_authority_guard() -> None:
    # Given: the bridge node source tree.
    tree = ast.parse(NODE_SOURCE.read_text(encoding="utf-8"))

    # When: the authority result assignment and action-server conditional are inspected.
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

    # Then: the authority guard directly controls action exposure.
    assert authority_assignments
    assert guarded_servers


def test_authority_guard_runs_before_serial_open() -> None:
    # Given: the bridge node constructor source tree.
    tree = ast.parse(NODE_SOURCE.read_text(encoding="utf-8"))
    constructor = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "__init__"
    )

    # When: startup side effects are ordered by source position.
    guard_line = next(
        call.lineno
        for call in ast.walk(constructor)
        if _is_call_named(call, AUTHORITY_GUARD_NAME)
    )
    open_line = next(
        call.lineno
        for call in ast.walk(constructor)
        if _is_call_named(call, "open")
    )

    # Then: insecure command-enabled startup fails before the serial endpoint is touched.
    assert guard_line < open_line


def test_bringup_launch_files_do_not_declare_static_authority_tokens() -> None:
    # Given: every Python launch file owned by ed_uav_bringup.
    launch_files = sorted(BRINGUP_LAUNCH_DIR.glob("*.launch.py"))
    assert launch_files

    # When: all launch argument declarations are scanned.
    offenders: list[Path] = []
    for launch_file in launch_files:
        tree = ast.parse(launch_file.read_text(encoding="utf-8"))
        if any(
            _is_call_named(call, "DeclareLaunchArgument")
            and call.args
            and isinstance(call.args[0], ast.Constant)
            and call.args[0].value == "authority_token"
            for call in ast.walk(tree)
        ):
            offenders.append(launch_file)

    # Then: dead app-level tokens cannot be mistaken for caller authorization.
    assert offenders == []


def test_fcu_dry_run_explicitly_disables_flight_commands() -> None:
    # Given: the credential-free offline FCU dry-run launch source.
    launch_file = BRINGUP_LAUNCH_DIR / "fcu_dry_run.launch.py"
    tree = ast.parse(launch_file.read_text(encoding="utf-8"))

    # When: parameter mappings are inspected.
    values = [
        value
        for mapping in ast.walk(tree)
        if isinstance(mapping, ast.Dict)
        for key, value in zip(mapping.keys, mapping.values)
        if isinstance(key, ast.Constant)
        and key.value == "enable_flight_commands"
    ]

    # Then: the PTY dry-run cannot expose a flight-command action.
    assert any(isinstance(value, ast.Constant) and value.value is False for value in values)
