"""Characterization reports remain diagnostics, never runtime admission gates."""

from __future__ import annotations

import hashlib

import pytest

from ed_uav_fcu_bridge import authority
from ed_uav_fcu_bridge.actions import CommandRequest
from ed_uav_fcu_bridge.capability import (
    CapabilityReportError,
    capability_integrity_hmac,
    parse_capability_report,
)


TEST_INTEGRITY_KEY = bytes(range(32))


def green_report() -> dict:
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
        "artifact_sha256": hashlib.sha256(b"diagnostic transcript\n").hexdigest(),
        "passed": True,
        "reason": "physical prop-off characterization passed all required checks",
        "provenance_authority": "independent-prop-off-bench",
        "integrity_hmac_sha256": "0" * 64,
    }


def test_characterization_report_parser_remains_available_for_diagnostics() -> None:
    report = parse_capability_report(green_report())

    assert report.passed is True
    assert report.evidence_kind == "physical_prop_off"
    assert len(capability_integrity_hmac(
        report,
        b"diagnostic transcript\n",
        TEST_INTEGRITY_KEY,
    )) == 64


@pytest.mark.parametrize(
    ("mutation", "reason"),
    (
        ({"schema": "unknown"}, "schema"),
        ({"version": True}, "version"),
        ({"artifact_sha256": "not-a-hash"}, "sha256"),
        ({"evidence_kind": "fake_pty"}, "physical prop-off"),
    ),
)
def test_diagnostic_report_parser_rejects_malformed_claims(
    mutation: dict,
    reason: str,
) -> None:
    report = green_report()
    report.update(mutation)

    with pytest.raises(CapabilityReportError, match=reason):
        parse_capability_report(report)


def test_runtime_authority_exports_no_programmable_capability_gate() -> None:
    assert not hasattr(authority, "require_programmable_capability")
    assert not hasattr(authority, "CapabilityTrust")
    assert not hasattr(authority, "ProgrammableCapabilityError")


def test_existing_high_level_commands_remain_byte_identical() -> None:
    actual = (
        CommandRequest.move(100, 30, 90).to_frame().hex().upper(),
        CommandRequest.hover().to_frame().hex().upper(),
    )

    assert actual == (
        "AAFFE00B10020364001E005A00000085E7",
        "AAFFE00B1000040000000000000000A8A0",
    )
