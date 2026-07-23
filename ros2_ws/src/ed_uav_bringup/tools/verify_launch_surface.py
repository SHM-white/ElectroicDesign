#!/usr/bin/env python3
"""Statically verify the P06 launch boundary without starting ROS processes."""

from __future__ import annotations

import argparse
import ast
import sys
from pathlib import Path


REQUIRED_ARGUMENTS = {
    "profile",
    "calibration_file",
    "camera_narrow_serial",
    "camera_wide_serial",
    "lidar_serial",
    "namespace",
    "use_sim_time",
}
REQUIRED_PROFILES = {"offline", "camera_only", "lidar", "competition"}


def _call_name(call: ast.Call) -> str | None:
    if isinstance(call.func, ast.Name):
        return call.func.id
    if isinstance(call.func, ast.Attribute):
        return call.func.attr
    return None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check the P06 launch argument and gate surface.")
    parser.add_argument("--launch", required=True, type=Path)
    arguments = parser.parse_args(argv)
    try:
        source = arguments.launch.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(arguments.launch))
    except (OSError, SyntaxError) as exc:
        print(f"BRINGUP: RED: malformed launch file: {exc}", file=sys.stderr)
        return 2
    arguments_found: set[str] = set()
    profile_literals: set[str] = set()
    gate_line: int | None = None
    node_line: int | None = None
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            if any(isinstance(target, ast.Name) and target.id == "P06_PROFILES" for target in node.targets):
                if isinstance(node.value, ast.Tuple):
                    profile_literals = {
                        value.value
                        for value in node.value.elts
                        if isinstance(value, ast.Constant) and isinstance(value.value, str)
                    }
        if not isinstance(node, ast.Call):
            continue
        call_name = _call_name(node)
        if call_name == "DeclareLaunchArgument" and node.args and isinstance(node.args[0], ast.Constant):
            argument_name = node.args[0].value
            if isinstance(argument_name, str):
                arguments_found.add(argument_name)
        if call_name == "validate_for_profile":
            gate_line = node.lineno
        if call_name == "Node":
            node_line = node.lineno
    if arguments_found != REQUIRED_ARGUMENTS:
        print("BRINGUP: RED: launch arguments do not match the P06 surface", file=sys.stderr)
        return 1
    if profile_literals != REQUIRED_PROFILES:
        print("BRINGUP: RED: launch profiles do not match the P06 surface", file=sys.stderr)
        return 1
    if gate_line is None or node_line is None or gate_line >= node_line:
        print("BRINGUP: RED: competition gate does not precede node construction", file=sys.stderr)
        return 1
    if "static_transform_publisher" in source or "map -> odom" in source or "odom -> base_link" in source:
        print("BRINGUP: RED: launch claims a forbidden TF authority", file=sys.stderr)
        return 1
    print("BRINGUP: GREEN")
    return 0


raise SystemExit(main())
