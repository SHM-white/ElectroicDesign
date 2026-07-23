"""Focused acceptance tests for the frozen ROS graph contract."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CHECKER = PACKAGE_ROOT / "tools" / "check_contract.py"
FIXTURE = PACKAGE_ROOT / "test" / "fixtures" / "valid_contract.json"
MANIFEST = PACKAGE_ROOT / "contracts" / "ros2_contract_manifest.json"


def run_checker(tmp_path: Path, mutation: dict | None = None) -> subprocess.CompletedProcess[str]:
    contract = json.loads(FIXTURE.read_text(encoding="utf-8"))
    if mutation:
        contract.update(mutation)
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")
    return subprocess.run(
        [sys.executable, str(CHECKER), str(path)],
        capture_output=True,
        text=True,
        check=False,
    )


def test_accepts_frozen_contract(tmp_path: Path) -> None:
    result = run_checker(tmp_path)
    assert result.returncode == 0, result.stderr
    assert "CONTRACT: GREEN" in result.stdout


def test_rejects_duplicate_odom_to_base_link_owner(tmp_path: Path) -> None:
    contract = json.loads(FIXTURE.read_text(encoding="utf-8"))
    contract["tf_edges"].append(
        {"parent": "odom", "child": "base_link", "publisher": "duplicate"}
    )
    path = tmp_path / "duplicate_tf.json"
    path.write_text(json.dumps(contract), encoding="utf-8")
    result = subprocess.run(
        [sys.executable, str(CHECKER), str(path)], capture_output=True, text=True, check=False
    )
    assert result.returncode != 0
    assert "duplicate TF authority: odom -> base_link" in result.stderr


def test_rejects_duplicate_topic_owner(tmp_path: Path) -> None:
    contract = json.loads(FIXTURE.read_text(encoding="utf-8"))
    contract["topics"].append(contract["topics"][0].copy())
    path = tmp_path / "duplicate_topic.json"
    path.write_text(json.dumps(contract), encoding="utf-8")
    result = subprocess.run(
        [sys.executable, str(CHECKER), str(path)], capture_output=True, text=True, check=False
    )
    assert result.returncode != 0
    assert "duplicate topic authority: /fcu/state" in result.stderr


def test_rejects_missing_unit(tmp_path: Path) -> None:
    contract = json.loads(FIXTURE.read_text(encoding="utf-8"))
    del contract["topics"][0]["units"]
    path = tmp_path / "missing_unit.json"
    path.write_text(json.dumps(contract), encoding="utf-8")
    result = subprocess.run(
        [sys.executable, str(CHECKER), str(path)], capture_output=True, text=True, check=False
    )
    assert result.returncode != 0
    assert "missing units" in result.stderr


def test_rejects_missing_frame(tmp_path: Path) -> None:
    contract = json.loads(FIXTURE.read_text(encoding="utf-8"))
    del contract["topics"][0]["frame"]
    path = tmp_path / "missing_frame.json"
    path.write_text(json.dumps(contract), encoding="utf-8")
    result = subprocess.run(
        [sys.executable, str(CHECKER), str(path)], capture_output=True, text=True, check=False
    )
    assert result.returncode != 0
    assert "missing frame" in result.stderr


def test_rejects_unknown_qos(tmp_path: Path) -> None:
    result = run_checker(tmp_path, {"topics": [{"name": "/bad", "type": "x", "owner": "x", "qos": "turbo", "units": "none", "frame": "base_link", "clock": "acquisition", "freshness": "0.1 s"}]})
    assert result.returncode != 0
    assert "unknown QoS" in result.stderr


def test_rejects_unbounded_dynamic_text(tmp_path: Path) -> None:
    result = run_checker(tmp_path, {"interfaces": [{"path": "msg/Bad.msg", "definition": "string reason"}]})
    assert result.returncode != 0
    assert "unbounded dynamic text" in result.stderr


def test_rejects_stale_schema(tmp_path: Path) -> None:
    result = run_checker(tmp_path, {"schema_version": 0})
    assert result.returncode != 0
    assert "unsupported schema_version" in result.stderr


def test_checked_in_manifest_is_green_and_deterministic() -> None:
    first = subprocess.run([sys.executable, str(CHECKER), str(MANIFEST)], capture_output=True, text=True, check=False)
    second = subprocess.run([sys.executable, str(CHECKER), str(MANIFEST)], capture_output=True, text=True, check=False)
    assert first.returncode == 0, first.stderr
    assert second.returncode == 0, second.stderr
    assert first.stdout == second.stdout == "CONTRACT: GREEN\n"
