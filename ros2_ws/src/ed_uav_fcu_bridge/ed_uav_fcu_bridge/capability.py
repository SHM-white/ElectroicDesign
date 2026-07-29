"""Strict capability-report boundary for programmable V7 field activation."""

from __future__ import annotations

import hashlib
import hmac
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Final, TypeAlias, TypedDict

JsonValue: TypeAlias = str | int | float | bool | None | list["JsonValue"] | dict[str, "JsonValue"]

SCHEMA: Final = "ed_uav_fcu_bridge.v7_programmable_capability"
VERSION: Final = 2
COMMANDS: Final = ("target_position", "target_height", "ascend", "descend")
BEHAVIORS: Final = (
    "ordering",
    "cancellation",
    "zero_motion",
    "retry",
    "link_loss",
    "mutual_exclusion",
    "hover",
    "land",
)


@dataclass(frozen=True, slots=True)
class CapabilityReportError(ValueError):
    reason: str

    def __str__(self) -> str:
        return self.reason


@dataclass(frozen=True, slots=True)
class AckLatencyObservation:
    command: str
    latency_ms: float


@dataclass(frozen=True, slots=True)
class CapabilityReport:
    schema: str
    version: int
    device_identity: str
    evidence_kind: str
    command_support: tuple[tuple[str, bool], ...]
    ack_latency_observations: tuple[AckLatencyObservation, ...]
    behavior_results: tuple[tuple[str, bool], ...]
    artifact_sha256: str
    passed: bool
    reason: str
    provenance_authority: str | None
    integrity_hmac_sha256: str | None


class AckLatencyDocument(TypedDict):
    command: str
    latency_ms: float


class CapabilityReportDocument(TypedDict):
    schema: str
    version: int
    device_identity: str
    evidence_kind: str
    command_support: dict[str, bool]
    ack_latency_observations: list[AckLatencyDocument]
    behavior_results: dict[str, bool]
    artifact_sha256: str
    passed: bool
    reason: str
    provenance_authority: str | None
    integrity_hmac_sha256: str | None


def _required_boolean_map(raw: JsonValue, names: tuple[str, ...], field: str) -> tuple[tuple[str, bool], ...]:
    if not isinstance(raw, dict) or set(raw) != set(names):
        raise CapabilityReportError(f"{field} must contain exactly the required keys")
    result: list[tuple[str, bool]] = []
    for name in names:
        value = raw[name]
        if type(value) is not bool:
            raise CapabilityReportError(f"{field}.{name} must be boolean")
        result.append((name, value))
    return tuple(result)


def _ack_observations(raw: JsonValue) -> tuple[AckLatencyObservation, ...]:
    if not isinstance(raw, list):
        raise CapabilityReportError("ack_latency_observations must be an array")
    result: list[AckLatencyObservation] = []
    for item in raw:
        if not isinstance(item, dict) or set(item) != {"command", "latency_ms"}:
            raise CapabilityReportError("ACK observation fields are invalid")
        command = item["command"]
        latency = item["latency_ms"]
        if command not in COMMANDS:
            raise CapabilityReportError("ACK observation command or latency is invalid")
        if isinstance(latency, bool) or not isinstance(latency, (int, float)):
            raise CapabilityReportError("ACK observation command or latency is invalid")
        latency_ms = float(latency)
        if not math.isfinite(latency_ms) or latency_ms < 0:
            raise CapabilityReportError("ACK latency must be finite and nonnegative")
        result.append(AckLatencyObservation(command=str(command), latency_ms=latency_ms))
    return tuple(result)


def parse_capability_report(raw: JsonValue) -> CapabilityReport:
    """Parse an untrusted JSON-compatible value into a complete report."""
    required = {
        "schema", "version", "device_identity", "evidence_kind", "command_support",
        "ack_latency_observations", "behavior_results", "artifact_sha256", "passed", "reason",
        "provenance_authority", "integrity_hmac_sha256",
    }
    if not isinstance(raw, dict) or set(raw) != required:
        raise CapabilityReportError("capability report fields do not match schema version 2")
    if raw["schema"] != SCHEMA or type(raw["version"]) is not int or raw["version"] != VERSION:
        raise CapabilityReportError("capability report schema or version is unsupported")
    device_identity = raw["device_identity"]
    evidence_kind = raw["evidence_kind"]
    artifact_sha256 = raw["artifact_sha256"]
    passed = raw["passed"]
    reason = raw["reason"]
    provenance_authority = raw["provenance_authority"]
    integrity_hmac_sha256 = raw["integrity_hmac_sha256"]
    if not isinstance(device_identity, str) or not device_identity.strip():
        raise CapabilityReportError("device identity must be supplied externally")
    if evidence_kind not in ("fake_pty", "physical_prop_off"):
        raise CapabilityReportError("evidence kind must be fake_pty or physical_prop_off")
    if not isinstance(artifact_sha256, str):
        raise CapabilityReportError("artifact sha256 must be 64 lowercase hexadecimal characters")
    if len(artifact_sha256) != 64 or any(
        character not in "0123456789abcdef" for character in artifact_sha256
    ):
        raise CapabilityReportError("artifact sha256 must be 64 lowercase hexadecimal characters")
    if type(passed) is not bool:
        raise CapabilityReportError("capability pass status and explicit reason are required")
    if not isinstance(reason, str) or not reason.strip():
        raise CapabilityReportError("capability pass status and explicit reason are required")
    if provenance_authority is not None and (
        not isinstance(provenance_authority, str) or not provenance_authority.strip()
    ):
        raise CapabilityReportError("provenance authority must be nonempty when supplied")
    if integrity_hmac_sha256 is not None and (
        not isinstance(integrity_hmac_sha256, str)
        or len(integrity_hmac_sha256) != 64
        or any(character not in "0123456789abcdef" for character in integrity_hmac_sha256)
    ):
        raise CapabilityReportError("integrity hmac must be 64 lowercase hexadecimal characters")
    report = CapabilityReport(
        schema=SCHEMA,
        version=VERSION,
        device_identity=device_identity,
        evidence_kind=str(evidence_kind),
        command_support=_required_boolean_map(raw["command_support"], COMMANDS, "command_support"),
        ack_latency_observations=_ack_observations(raw["ack_latency_observations"]),
        behavior_results=_required_boolean_map(raw["behavior_results"], BEHAVIORS, "behavior_results"),
        artifact_sha256=artifact_sha256,
        passed=passed,
        reason=reason,
        provenance_authority=provenance_authority,
        integrity_hmac_sha256=integrity_hmac_sha256,
    )
    observed_commands = tuple(item.command for item in report.ack_latency_observations)
    complete_green = (
        report.evidence_kind == "physical_prop_off"
        and all(value for _, value in report.command_support)
        and all(value for _, value in report.behavior_results)
        and len(observed_commands) == len(COMMANDS)
        and set(observed_commands) == set(COMMANDS)
        and report.provenance_authority is not None
        and report.integrity_hmac_sha256 is not None
    )
    if report.passed and not complete_green:
        raise CapabilityReportError("green capability report lacks required physical prop-off evidence")
    return report


def load_capability_report(path: Path) -> CapabilityReport:
    """Read and parse one capability JSON document."""
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise CapabilityReportError(f"capability report cannot be read: {error}") from error
    return parse_capability_report(raw)


def capability_report_document(report: CapabilityReport) -> CapabilityReportDocument:
    """Convert a validated report model to its stable JSON document shape."""
    return {
        "schema": report.schema,
        "version": report.version,
        "device_identity": report.device_identity,
        "evidence_kind": report.evidence_kind,
        "command_support": dict(report.command_support),
        "ack_latency_observations": [
            {"command": item.command, "latency_ms": item.latency_ms}
            for item in report.ack_latency_observations
        ],
        "behavior_results": dict(report.behavior_results),
        "artifact_sha256": report.artifact_sha256,
        "passed": report.passed,
        "reason": report.reason,
        "provenance_authority": report.provenance_authority,
        "integrity_hmac_sha256": report.integrity_hmac_sha256,
    }


def capability_integrity_hmac(
    report: CapabilityReport,
    artifact: bytes,
    key: bytes,
) -> str:
    """Bind the complete report envelope and artifact bytes to a trusted key."""
    document = capability_report_document(report)
    document["integrity_hmac_sha256"] = None
    canonical = json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
    return hmac.new(key, canonical + b"\n" + artifact, hashlib.sha256).hexdigest()
