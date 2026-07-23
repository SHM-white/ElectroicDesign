#!/usr/bin/env python3
"""Validate the machine-readable ED UAV ROS contract."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from typing import Any


SCHEMA_VERSION = 1
QOS_PROFILES = {
    "sensor_data_best_effort",
    "state_reliable",
    "latched_reliable",
    "command_reliable",
}
REQUIRED_TOPIC_FIELDS = ("name", "type", "owner", "qos", "units", "frame", "clock", "freshness")
REQUIRED_ENDPOINT_FIELDS = ("name", "type", "owner", "qos", "units", "frame", "clock")
REQUIRED_ROOT_FIELDS = ("schema_version", "interfaces", "topics", "services", "actions", "tf_edges", "static_frames", "lifecycle")
UNBOUNDED_STRING = re.compile(r"(?m)^\s*(?:w?string)(?:\s|$)")
UNBOUNDED_SEQUENCE = re.compile(r"(?m)^\s*[^#\n]+\[\](?:\s|$)")


def error(errors: list[str], message: str) -> None:
    errors.append(message)


def is_nonempty(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def check_unique_authority(items: list[dict[str, Any]], key: str, label: str, errors: list[str]) -> None:
    owners: dict[str, str] = {}
    for item in items:
        name = item.get(key)
        owner = item.get("owner", item.get("publisher"))
        if not is_nonempty(name) or not is_nonempty(owner):
            error(errors, f"{label} missing name or authority")
            continue
        if name in owners:
            error(errors, f"duplicate {label} authority: {name}")
        else:
            owners[name] = owner


def validate(contract: Any) -> list[str]:
    errors: list[str] = []
    if not isinstance(contract, dict):
        return ["contract root must be an object"]
    for field in REQUIRED_ROOT_FIELDS:
        if field not in contract:
            error(errors, f"missing root field: {field}")
    if contract.get("schema_version") != SCHEMA_VERSION:
        error(errors, f"unsupported schema_version: {contract.get('schema_version')}")

    interfaces = contract.get("interfaces", [])
    if not isinstance(interfaces, list):
        error(errors, "interfaces must be a list")
    else:
        for interface in interfaces:
            if not isinstance(interface, dict) or not is_nonempty(interface.get("path")):
                error(errors, "interface missing path")
                continue
            definition = interface.get("definition")
            if not is_nonempty(definition):
                error(errors, f"interface missing definition: {interface['path']}")
                continue
            if UNBOUNDED_STRING.search(definition) or UNBOUNDED_SEQUENCE.search(definition):
                error(errors, f"unbounded dynamic text or array: {interface['path']}")

    topics = contract.get("topics", [])
    if not isinstance(topics, list):
        error(errors, "topics must be a list")
    else:
        for topic in topics:
            if not isinstance(topic, dict):
                error(errors, "topic entry must be an object")
                continue
            for field in REQUIRED_TOPIC_FIELDS:
                if not is_nonempty(topic.get(field)):
                    error(errors, f"topic {topic.get('name', '<unnamed>')} missing {field}")
            if topic.get("qos") not in QOS_PROFILES:
                error(errors, f"unknown QoS: {topic.get('qos')}")
        check_unique_authority(topics, "name", "topic", errors)

    for endpoint_kind in ("services", "actions"):
        endpoints = contract.get(endpoint_kind, [])
        if not isinstance(endpoints, list):
            error(errors, f"{endpoint_kind} must be a list")
            continue
        for endpoint in endpoints:
            if not isinstance(endpoint, dict):
                error(errors, f"{endpoint_kind} entry must be an object")
                continue
            for field in REQUIRED_ENDPOINT_FIELDS:
                if not is_nonempty(endpoint.get(field)):
                    error(errors, f"{endpoint_kind} {endpoint.get('name', '<unnamed>')} missing {field}")
            if endpoint.get("qos") not in QOS_PROFILES:
                error(errors, f"unknown QoS: {endpoint.get('qos')}")
        check_unique_authority(endpoints, "name", endpoint_kind[:-1], errors)

    tf_edges = contract.get("tf_edges", [])
    if not isinstance(tf_edges, list):
        error(errors, "tf_edges must be a list")
    else:
        normalized: list[dict[str, Any]] = []
        for edge in tf_edges:
            if not isinstance(edge, dict):
                error(errors, "TF edge must be an object")
                continue
            parent, child = edge.get("parent"), edge.get("child")
            if not is_nonempty(parent) or not is_nonempty(child) or not is_nonempty(edge.get("publisher")):
                error(errors, "TF edge missing parent, child, or publisher")
                continue
            normalized.append({"edge": f"{parent} -> {child}", "publisher": edge["publisher"]})
        check_unique_authority(normalized, "edge", "TF", errors)

    static_frames = contract.get("static_frames", [])
    if not isinstance(static_frames, list):
        error(errors, "static_frames must be a list")
    else:
        normalized_static: list[dict[str, Any]] = []
        for edge in static_frames:
            if not isinstance(edge, dict):
                error(errors, "static frame must be an object")
                continue
            parent, child, publisher = edge.get("parent"), edge.get("child"), edge.get("publisher")
            if not is_nonempty(parent) or not is_nonempty(child) or not is_nonempty(publisher):
                error(errors, "static frame missing parent, child, or publisher")
                continue
            normalized_static.append({"edge": f"{parent} -> {child}", "publisher": publisher})
        check_unique_authority(normalized_static, "edge", "static TF", errors)

    lifecycle = contract.get("lifecycle", [])
    if not isinstance(lifecycle, list):
        error(errors, "lifecycle must be a list")
    else:
        for entry in lifecycle:
            if not isinstance(entry, dict):
                error(errors, "lifecycle entry must be an object")
                continue
            for field in ("node", "hardware_owner", "activation"):
                if not is_nonempty(entry.get(field)):
                    error(errors, f"lifecycle entry missing {field}")

    return errors


def validate_interface_files(contract_path: Path, contract: Any, errors: list[str]) -> None:
    """Ensure a green manifest cannot conceal an unbounded checked-in interface."""
    package_root = contract_path.parent.parent
    for interface in contract.get("interfaces", []):
        if not isinstance(interface, dict) or not is_nonempty(interface.get("path")):
            continue
        interface_path = package_root / interface["path"]
        try:
            definition = interface_path.read_text(encoding="utf-8")
        except OSError as exc:
            error(errors, f"cannot read interface file {interface['path']}: {exc}")
            continue
        if UNBOUNDED_STRING.search(definition) or UNBOUNDED_SEQUENCE.search(definition):
            error(errors, f"unbounded dynamic text or array: {interface['path']}")


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: check_contract.py CONTRACT.json", file=sys.stderr)
        return 64
    try:
        contract_path = Path(argv[1]).resolve()
        contract = json.loads(contract_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"malformed contract input: {exc}", file=sys.stderr)
        return 2
    errors = validate(contract)
    if contract_path.name == "ros2_contract_manifest.json":
        validate_interface_files(contract_path, contract, errors)
    if errors:
        for message in errors:
            print(f"CONTRACT: RED: {message}", file=sys.stderr)
        return 1
    print("CONTRACT: GREEN")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
