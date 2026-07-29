from __future__ import annotations

import ast
import hashlib
import json
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_SOURCE = PACKAGE_ROOT / "ed_uav_fcu_bridge" / "node.py"
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_fcu_bridge import authority
from ed_uav_fcu_bridge.actions import CommandRequest
from ed_uav_fcu_bridge.capability import (
    capability_integrity_hmac,
    parse_capability_report,
)

TEST_INTEGRITY_KEY = bytes(range(32))
TEST_PROVENANCE_AUTHORITY = "independent-prop-off-bench"


def green_report() -> dict[str, str | int | bool | list[dict[str, str | float]] | dict[str, bool]]:
    return {
        "schema": "ed_uav_fcu_bridge.v7_programmable_capability",
        "version": 2,
        "device_identity": "operator-supplied-fcu-001",
        "evidence_kind": "physical_prop_off",
        "command_support": {
            "target_position": True,
            "target_height": True,
            "ascend": True,
            "descend": True,
        },
        "ack_latency_observations": [
            {"command": "target_position", "latency_ms": 12.5},
            {"command": "target_height", "latency_ms": 13.0},
            {"command": "ascend", "latency_ms": 11.0},
            {"command": "descend", "latency_ms": 12.0},
        ],
        "behavior_results": {
            "ordering": True,
            "cancellation": True,
            "zero_motion": True,
            "retry": True,
            "link_loss": True,
            "mutual_exclusion": True,
            "hover": True,
            "land": True,
        },
        "artifact_sha256": "a" * 64,
        "passed": True,
        "reason": "physical prop-off characterization passed all required checks",
        "provenance_authority": TEST_PROVENANCE_AUTHORITY,
        "integrity_hmac_sha256": "0" * 64,
    }


def write_report(path: Path, report: dict[str, str | int | bool | list[dict[str, str | float]] | dict[str, bool]]) -> None:
    artifact = b"retained prop-off transcript\n"
    report["artifact_sha256"] = hashlib.sha256(artifact).hexdigest()
    model = parse_capability_report(report)
    report["integrity_hmac_sha256"] = capability_integrity_hmac(
        model,
        artifact,
        TEST_INTEGRITY_KEY,
    )
    path.with_suffix(path.suffix + ".artifact.jsonl").write_bytes(artifact)
    path.write_text(json.dumps(report), encoding="utf-8")


def capability_trust(
    path: Path,
    device_identity: str = "operator-supplied-fcu-001",
) -> authority.CapabilityTrust:
    return authority.CapabilityTrust(
        path,
        device_identity,
        TEST_PROVENANCE_AUTHORITY,
        TEST_INTEGRITY_KEY,
    )


def test_missing_capability_report_blocks_programmable_field_activation(tmp_path: Path) -> None:
    # Given: field activation references no retained capability report.
    gate = getattr(authority, "require_programmable_capability", None)
    error_type = getattr(authority, "ProgrammableCapabilityError", RuntimeError)
    assert callable(gate), "missing programmable capability gate"

    # When / Then: startup fails closed before a programmable command is exposed.
    with pytest.raises(error_type, match="report"):
        gate(True, capability_trust(tmp_path / "missing.json"))


def test_red_capability_report_blocks_programmable_field_activation(tmp_path: Path) -> None:
    # Given: a complete report explicitly records a failed link-loss result.
    report = green_report()
    report["passed"] = False
    report["reason"] = "link-loss response failed"
    path = tmp_path / "red.json"
    write_report(path, report)
    gate = getattr(authority, "require_programmable_capability", None)
    error_type = getattr(authority, "ProgrammableCapabilityError", RuntimeError)
    assert callable(gate), "missing programmable capability gate"

    # When / Then: the retained red reason rejects field activation.
    with pytest.raises(error_type, match="link-loss response failed"):
        gate(True, capability_trust(path))


def test_signed_green_report_still_fails_closed_on_ack_correlation(tmp_path: Path) -> None:
    # Given: a schema-complete green physical prop-off report.
    path = tmp_path / "green.json"
    write_report(path, green_report())
    gate = getattr(authority, "require_programmable_capability", None)
    error_type = getattr(authority, "ProgrammableCapabilityError", RuntimeError)
    assert callable(gate), "missing programmable capability gate"

    # When / Then: only the externally supplied matching device may activate.
    with pytest.raises(error_type, match="correlation"):
        gate(True, capability_trust(path))
    with pytest.raises(error_type, match="identity"):
        gate(True, capability_trust(path, "different-fcu"))


def test_green_report_rejects_tampered_artifact(tmp_path: Path) -> None:
    # Given: a green report whose retained transcript changed after hashing.
    path = tmp_path / "green.json"
    write_report(path, green_report())
    path.with_suffix(path.suffix + ".artifact.jsonl").write_text("tampered\n", encoding="utf-8")

    # When / Then: field activation rejects the integrity mismatch.
    with pytest.raises(authority.ProgrammableCapabilityError, match="sha256"):
        authority.require_programmable_capability(
            True,
            capability_trust(path),
        )


@pytest.mark.parametrize(
    ("mutation", "reason"),
    (
        ({"schema": "unknown"}, "schema"),
        ({"artifact_sha256": "not-a-hash"}, "sha256"),
        ({"evidence_kind": "fake_pty"}, "physical prop-off"),
        ({"ack_latency_observations": []}, "physical prop-off"),
        (
            {
                "ack_latency_observations": [
                    {"command": "target_position", "latency_ms": 1.0},
                    {"command": "target_height", "latency_ms": 1.0},
                    {"command": "ascend", "latency_ms": 1.0},
                    {"command": "descend", "latency_ms": 1.0},
                    {"command": "descend", "latency_ms": 2.0},
                ]
            },
            "physical prop-off",
        ),
    ),
)
def test_adversarial_reports_fail_closed(
    tmp_path: Path,
    mutation: dict[str, str | list[dict[str, str | float]]],
    reason: str,
) -> None:
    # Given: a nominally green report with one adversarial field mutation.
    report = green_report()
    path = tmp_path / "adversarial.json"
    write_report(path, report)
    report.update(mutation)
    path.write_text(json.dumps(report), encoding="utf-8")

    # When / Then: parsing rejects it with a deterministic schema reason.
    with pytest.raises(authority.ProgrammableCapabilityError, match=reason):
        authority.require_programmable_capability(
            True,
            capability_trust(path),
        )


def test_node_defaults_programmable_commands_off_and_gates_before_serial_open() -> None:
    # Given: the ROS node source without constructing hardware or importing launch code.
    tree = ast.parse(NODE_SOURCE.read_text(encoding="utf-8"))
    constructor = next(
        item for item in ast.walk(tree)
        if isinstance(item, ast.FunctionDef) and item.name == "__init__"
    )

    # When: startup parameter defaults and side-effect ordering are inspected.
    defaults = [
        call.args[1]
        for call in ast.walk(constructor)
        if isinstance(call, ast.Call)
        and getattr(call.func, "attr", "") == "declare_parameter"
        and len(call.args) >= 2
        and isinstance(call.args[0], ast.Constant)
        and call.args[0].value == "enable_programmable_commands"
    ]
    gate_line = next(
        call.lineno for call in ast.walk(constructor)
        if isinstance(call, ast.Call)
        and getattr(call.func, "id", "") == "require_programmable_capability"
    )
    open_line = next(
        call.lineno for call in ast.walk(constructor)
        if isinstance(call, ast.Call) and getattr(call.func, "attr", "") == "open"
    )

    # Then: omission disables the feature and red preflight rejects before serial access.
    assert len(defaults) == 1
    assert isinstance(defaults[0], ast.Constant) and defaults[0].value is False
    assert gate_line < open_line


def test_existing_high_level_commands_remain_byte_identical() -> None:
    # Given: the existing MOVE and HOVER high-level actions.
    # When: their frames are built after introducing the new gated domain.
    actual = (
        CommandRequest.move(100, 30, 90).to_frame().hex().upper(),
        CommandRequest.hover().to_frame().hex().upper(),
    )

    # Then: neither legacy high-level command depends on a capability report.
    assert actual == (
        "AAFFE00B10020364001E005A00000085E7",
        "AAFFE00B1000040000000000000000A8A0",
    )
