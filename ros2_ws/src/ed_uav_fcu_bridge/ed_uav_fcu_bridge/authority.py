"""Startup policy for exposing the flight-command action."""

from __future__ import annotations

import hashlib
import hmac
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from .capability import (
    CapabilityReportError,
    capability_integrity_hmac,
    load_capability_report,
)


@dataclass(frozen=True, slots=True)
class FlightCommandAuthorityError(RuntimeError):
    """Flight commands were requested without enforced SROS2 startup settings."""

    invalid_settings: tuple[str, ...]

    def __str__(self) -> str:
        settings = ", ".join(self.invalid_settings)
        return (
            "flight commands require ROS_SECURITY_ENABLE=true, "
            "ROS_SECURITY_STRATEGY=Enforce, and an existing directory "
            f"ROS_SECURITY_KEYSTORE; invalid settings: {settings}. "
            "SROS2 signed permissions with default DENY remain the caller "
            "authorization layer; these startup checks do not authorize a caller."
        )


@dataclass(frozen=True, slots=True)
class ProgrammableCapabilityError(RuntimeError):
    """Programmable commands were requested without matching green evidence."""

    reason: str

    def __str__(self) -> str:
        return self.reason


@dataclass(frozen=True, slots=True)
class CapabilityTrust:
    report_path: Path
    device_identity: str
    provenance_authority: str
    integrity_key: bytes


def capability_trust_from_environment(
    report_path: Path,
    device_identity: str,
    environment: Mapping[str, str],
) -> CapabilityTrust:
    """Parse deployment-owned provenance trust material."""
    authority = environment.get("ED_UAV_V7_PROVENANCE_AUTHORITY", "").strip()
    key_hex = environment.get("ED_UAV_V7_CAPABILITY_HMAC_KEY_HEX", "").strip()
    try:
        key = bytes.fromhex(key_hex)
    except ValueError as error:
        raise ProgrammableCapabilityError("capability integrity key is not hexadecimal") from error
    if not authority or len(key) < 32:
        raise ProgrammableCapabilityError(
            "programmable capability requires deployment-trusted provenance authority and key"
        )
    return CapabilityTrust(report_path, device_identity, authority, key)


def require_programmable_capability(
    enabled: bool,
    trust: CapabilityTrust,
) -> bool:
    """Allow programmable field commands only for matching retained evidence."""
    if not enabled:
        return False
    try:
        report = load_capability_report(trust.report_path)
    except CapabilityReportError as error:
        raise ProgrammableCapabilityError(str(error)) from error
    artifact_path = trust.report_path.with_suffix(trust.report_path.suffix + ".artifact.jsonl")
    try:
        artifact = artifact_path.read_bytes()
    except OSError as error:
        raise ProgrammableCapabilityError(
            f"capability artifact cannot be read: {error}"
        ) from error
    artifact_sha256 = hashlib.sha256(artifact).hexdigest()
    if artifact_sha256 != report.artifact_sha256:
        raise ProgrammableCapabilityError("capability artifact sha256 does not match report")
    if report.provenance_authority != trust.provenance_authority:
        raise ProgrammableCapabilityError("capability provenance authority is not trusted")
    if report.integrity_hmac_sha256 is None:
        raise ProgrammableCapabilityError("capability report has no verified integrity signature")
    expected_hmac = capability_integrity_hmac(report, artifact, trust.integrity_key)
    if not hmac.compare_digest(expected_hmac, report.integrity_hmac_sha256):
        raise ProgrammableCapabilityError("capability report integrity verification failed")
    if report.device_identity != trust.device_identity:
        raise ProgrammableCapabilityError(
            "capability report device identity does not match configured FCU"
        )
    if not report.passed:
        raise ProgrammableCapabilityError(f"capability report is red: {report.reason}")
    raise ProgrammableCapabilityError(
        "V7 ACK generation correlation is unprovable; programmable field capability remains disabled"
    )


def require_flight_command_authority(
    enabled: bool,
    environment: Mapping[str, str],
) -> bool:
    """Allow action exposure only with enforced SROS2 startup configuration.

    SROS2 signed permissions with default DENY remain responsible for caller
    authorization; this guard only prevents insecure server startup.
    """
    if not enabled:
        return False

    keystore = environment.get("ROS_SECURITY_KEYSTORE", "").strip()
    invalid_settings = tuple(
        name
        for name, valid in (
            ("ROS_SECURITY_ENABLE", environment.get("ROS_SECURITY_ENABLE") == "true"),
            ("ROS_SECURITY_STRATEGY", environment.get("ROS_SECURITY_STRATEGY") == "Enforce"),
            ("ROS_SECURITY_KEYSTORE", bool(keystore) and Path(keystore).is_dir()),
        )
        if not valid
    )
    if invalid_settings:
        raise FlightCommandAuthorityError(invalid_settings=invalid_settings)
    return True
