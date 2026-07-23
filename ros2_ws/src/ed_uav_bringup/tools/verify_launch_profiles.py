#!/usr/bin/env python3
"""Static verification for ED UAV launch profiles — no ROS runtime needed."""

from __future__ import annotations

import argparse
import ast
import sys
from pathlib import Path

# ── per-profile contracts ──────────────────────────────────────────────

PROFILE_CONTRACTS: dict[str, dict] = {
    "offline_replay": {
        "required_args": {
            "bag_path",
            "bag_rate",
            "calibration_file",
            "camera_narrow_serial",
            "camera_wide_serial",
            "lidar_serial",
            "namespace",
            "use_sim_time",
            "authority_token",
        },
        "forbidden_args": set(),
        "profile_value": "offline",
        "lifecycle_stages": ("calibration_gate", "robot_state_publisher", "replay"),
        "must_have": {"validate_for_profile", "ExecuteProcess", "load_calibration"},
    },
    "camera_only": {
        "required_args": {
            "calibration_file",
            "camera_narrow_serial",
            "camera_wide_serial",
            "namespace",
            "use_sim_time",
            "authority_token",
        },
        "forbidden_args": {"lidar_serial"},
        "profile_value": "camera_only",
        "lifecycle_stages": ("calibration_gate", "robot_state_publisher", "camera_drivers"),
        "must_have": {"validate_for_profile", "load_calibration"},
    },
    "lidar": {
        "required_args": {
            "calibration_file",
            "camera_narrow_serial",
            "camera_wide_serial",
            "lidar_serial",
            "namespace",
            "use_sim_time",
            "authority_token",
        },
        "forbidden_args": set(),
        "profile_value": "lidar",
        "lifecycle_stages": ("calibration_gate", "robot_state_publisher", "lidar_driver"),
        "must_have": {"validate_for_profile", "load_calibration"},
    },
    "competition": {
        "required_args": {
            "calibration_file",
            "camera_narrow_serial",
            "camera_wide_serial",
            "lidar_serial",
            "namespace",
            "use_sim_time",
            "authority_token",
        },
        "forbidden_args": set(),
        "profile_value": "competition",
        "lifecycle_stages": ("calibration_gate", "robot_state_publisher", "hardware_owners", "localization"),
        "must_have": {"validate_for_profile", "load_calibration"},
    },
    "fcu_dry_run": {
        "required_args": {
            "pty_device",
            "calibration_file",
            "camera_narrow_serial",
            "camera_wide_serial",
            "lidar_serial",
            "namespace",
            "use_sim_time",
            "authority_token",
        },
        "forbidden_args": set(),
        "profile_value": "offline",
        "lifecycle_stages": ("calibration_gate", "robot_state_publisher", "fcu_dry"),
        "must_have": {"validate_for_profile", "load_calibration"},
    },
    "legacy_rollback": {
        "required_args": {
            "calibration_file",
            "camera_narrow_serial",
            "camera_wide_serial",
            "lidar_serial",
            "namespace",
            "use_sim_time",
            "authority_token",
        },
        "forbidden_args": set(),
        "profile_value": "offline",
        "lifecycle_stages": ("calibration_gate", "robot_state_publisher"),
        "must_have": {"validate_for_profile", "load_calibration"},
    },
}


def _call_name(call: ast.Call) -> str | None:
    if isinstance(call.func, ast.Name):
        return call.func.id
    if isinstance(call.func, ast.Attribute):
        return call.func.attr
    return None


def _string_constant(node: ast.expr) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def verify_launch_file(file_path: Path, profile_name: str) -> int:
    contract = PROFILE_CONTRACTS.get(profile_name)
    if contract is None:
        print(f"PROFILE {profile_name}: RED: unknown profile name", file=sys.stderr)
        return 1

    try:
        source = file_path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(file_path))
    except (OSError, SyntaxError) as exc:
        print(f"PROFILE {profile_name}: RED: {exc}", file=sys.stderr)
        return 2

    arguments_found: set[str] = set()
    profile_literal: str | None = None
    lifecycle_stage_values: set[str] = set()
    identifiers_found: set[str] = set()

    gate_line: int | None = None
    node_line: int | None = None

    for node in ast.walk(tree):
        # Collect all Name identifiers
        if isinstance(node, ast.Name):
            identifiers_found.add(node.id)

        # Collect argument declarations
        if isinstance(node, ast.Call):
            call_name = _call_name(node)
            if call_name == "DeclareLaunchArgument" and node.args and isinstance(node.args[0], ast.Constant):
                arg_name = node.args[0].value
                if isinstance(arg_name, str):
                    arguments_found.add(arg_name)
            if call_name == "validate_for_profile":
                gate_line = node.lineno
            if call_name == "Node":
                node_line = node.lineno
            if call_name in ("ExecuteProcess", "Node", "OpaqueFunction", "DeclareLaunchArgument"):
                identifiers_found.add(call_name)

        # Check PROFILE constant assignment
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "PROFILE":
                    profile_literal = _string_constant(node.value)
            # Check LIFECYCLE_ORDER tuple
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "LIFECYCLE_ORDER":
                    if isinstance(node.value, ast.Tuple):
                        lifecycle_stage_values = {
                            elt.value
                            for elt in node.value.elts
                            if isinstance(elt, ast.Constant) and isinstance(elt.value, str)
                        }

    # Check required arguments
    missing = contract["required_args"] - arguments_found
    if missing:
        print(f"PROFILE {profile_name}: RED: missing arguments: {sorted(missing)}", file=sys.stderr)
        return 1

    # Check forbidden arguments
    forbidden_present = contract["forbidden_args"] & arguments_found
    if forbidden_present:
        print(f"PROFILE {profile_name}: RED: forbidden arguments present: {sorted(forbidden_present)}", file=sys.stderr)
        return 1

    # Check profile value
    if profile_literal != contract["profile_value"]:
        print(
            f"PROFILE {profile_name}: RED: PROFILE={profile_literal}, expected={contract['profile_value']}",
            file=sys.stderr,
        )
        return 1

    # Check lifecycle stages
    expected_stages = set(contract["lifecycle_stages"])
    if lifecycle_stage_values != expected_stages:
        print(
            f"PROFILE {profile_name}: RED: LIFECYCLE_ORDER={sorted(lifecycle_stage_values)}, expected={sorted(expected_stages)}",
            file=sys.stderr,
        )
        return 1

    # Check must-have identifiers
    missing_ids = contract["must_have"] - identifiers_found
    if missing_ids:
        print(f"PROFILE {profile_name}: RED: missing required identifiers: {sorted(missing_ids)}", file=sys.stderr)
        return 1

    # Gate must precede node construction
    if gate_line is None:
        print(f"PROFILE {profile_name}: RED: validate_for_profile gate missing", file=sys.stderr)
        return 1
    if node_line is None:
        print(f"PROFILE {profile_name}: RED: no Node construction found", file=sys.stderr)
        return 1
    if gate_line >= node_line:
        print(f"PROFILE {profile_name}: RED: gate (line {gate_line}) does not precede node (line {node_line})", file=sys.stderr)
        return 1

    # For competition profile, ensure serial defaults are empty (not UNSET)
    if profile_name == "competition":
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and _call_name(node) == "DeclareLaunchArgument":
                if len(node.args) >= 2 and _string_constant(node.args[0]) in {"camera_narrow_serial", "camera_wide_serial", "lidar_serial"}:
                    if isinstance(node.args[1], ast.Constant) and node.args[1].value == "UNSET":
                        print(f"PROFILE {profile_name}: RED: competition serial defaults must be empty, not 'UNSET'", file=sys.stderr)
                        return 1

    # No forbidden TF authority
    if "static_transform_publisher" in source or "map -> odom" in source:
        print(f"PROFILE {profile_name}: RED: launch claims a forbidden TF authority", file=sys.stderr)
        return 1

    print(f"PROFILE {profile_name}: GREEN")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Verify ED UAV launch profile structure.")
    parser.add_argument("--launch", required=True, type=Path, help="Path to the launch profile .py file")
    parser.add_argument("--profile", required=True, choices=sorted(PROFILE_CONTRACTS), help="Profile name")
    args = parser.parse_args(argv)
    return verify_launch_file(args.launch, args.profile)


if __name__ == "__main__":
    raise SystemExit(main())
