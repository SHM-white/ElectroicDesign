#!/usr/bin/env python3
"""Verify that a robot model owns only its contract-approved static TF edges."""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as element_tree
from pathlib import Path


EXPECTED_EDGES = {
    ("base_link", "fcu_link"),
    ("base_link", "lidar_link"),
    ("base_link", "camera_narrow_optical_frame"),
    ("base_link", "camera_wide_optical_frame"),
    ("base_link", "rangefinder_link"),
}
FORBIDDEN_EDGES = {("map", "odom"), ("odom", "base_link")}


def static_edges(urdf_path: Path) -> list[tuple[str, str]]:
    root = element_tree.parse(urdf_path).getroot()
    edges: list[tuple[str, str]] = []
    for joint in root.findall("joint"):
        if joint.get("type") != "fixed":
            continue
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None or parent.get("link") is None or child.get("link") is None:
            raise ValueError("fixed joint missing parent or child link")
        edges.append((parent.get("link"), child.get("link")))
    return edges


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate contract-owned static TF edges.")
    parser.add_argument("--urdf", required=True, type=Path)
    arguments = parser.parse_args(argv)
    try:
        edges = static_edges(arguments.urdf)
    except (OSError, element_tree.ParseError, ValueError) as exc:
        print(f"DESCRIPTION: RED: malformed robot model: {exc}", file=sys.stderr)
        return 2
    seen: set[tuple[str, str]] = set()
    for edge in edges:
        if edge in seen:
            print(f"DESCRIPTION: RED: duplicate static TF: {edge[0]} -> {edge[1]}", file=sys.stderr)
            return 1
        seen.add(edge)
    for edge in edges:
        if edge in FORBIDDEN_EDGES:
            print(f"DESCRIPTION: RED: forbidden static TF: {edge[0]} -> {edge[1]}", file=sys.stderr)
            return 1
        if edge not in EXPECTED_EDGES:
            print(f"DESCRIPTION: RED: unauthorized static TF: {edge[0]} -> {edge[1]}", file=sys.stderr)
            return 1
    if seen != EXPECTED_EDGES:
        missing = sorted(EXPECTED_EDGES - seen)
        print(f"DESCRIPTION: RED: missing static TF: {missing[0][0]} -> {missing[0][1]}", file=sys.stderr)
        return 1
    print("DESCRIPTION: GREEN")
    return 0


raise SystemExit(main())
