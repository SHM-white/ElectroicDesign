"""Fail-closed local provisioning for UDP peers and HMAC material."""

from __future__ import annotations

from dataclasses import dataclass
import ipaddress
from pathlib import Path

from .errors import BridgeConfigError
from .models import Endpoint, SenderId


@dataclass(frozen=True, slots=True)
class BridgeProvisioning:
    bind: Endpoint
    car_peer: Endpoint
    hmi_peer: Endpoint
    car_sender_id: SenderId
    hmi_sender_id: SenderId
    bridge_sender_id: SenderId
    hmac_key_file: Path
    mission_timeout_seconds: float
    telemetry_stale_seconds: float


@dataclass(frozen=True, slots=True)
class BridgeConfig:
    provisioning: BridgeProvisioning
    hmac_key: bytes


def load_bridge_config(provisioning: BridgeProvisioning) -> BridgeConfig:
    _validate_endpoint(provisioning.bind, allow_unspecified=True, allow_ephemeral=True)
    _validate_endpoint(provisioning.car_peer, allow_unspecified=False, allow_ephemeral=False)
    _validate_endpoint(provisioning.hmi_peer, allow_unspecified=False, allow_ephemeral=False)
    for field, sender_id in (
        ("car_sender_id", provisioning.car_sender_id),
        ("hmi_sender_id", provisioning.hmi_sender_id),
        ("bridge_sender_id", provisioning.bridge_sender_id),
    ):
        _validate_sender(field, sender_id)
    if provisioning.car_sender_id == provisioning.hmi_sender_id:
        raise BridgeConfigError("sender_id", "car and HMI sender IDs must differ")
    if provisioning.mission_timeout_seconds <= 0.0:
        raise BridgeConfigError("mission_timeout_seconds", "must be positive")
    if not 0.05 <= provisioning.telemetry_stale_seconds <= 10.0:
        raise BridgeConfigError("telemetry_stale_seconds", "must be in range 0.05-10.0")
    key = load_hmac_key_file(provisioning.hmac_key_file)
    return BridgeConfig(provisioning=provisioning, hmac_key=key)


def load_hmac_key_file(path: Path) -> bytes:
    try:
        encoded = path.read_text(encoding="ascii").strip()
    except OSError as error:
        raise BridgeConfigError("hmac_key_file", str(error)) from error
    try:
        key = bytes.fromhex(encoded)
    except ValueError as error:
        raise BridgeConfigError("hmac_key_file", "must contain hexadecimal bytes") from error
    if len(key) < 32:
        raise BridgeConfigError("hmac_key_file", "must contain at least 32 bytes")
    return key


def _validate_endpoint(
    endpoint: Endpoint, *, allow_unspecified: bool, allow_ephemeral: bool
) -> None:
    try:
        address = ipaddress.ip_address(endpoint.host)
    except ValueError as error:
        raise BridgeConfigError("endpoint", f"{endpoint.host!r} is not a numeric IP") from error
    if address.is_unspecified and not allow_unspecified:
        raise BridgeConfigError("endpoint", "peer address cannot be unspecified")
    minimum_port = 0 if allow_ephemeral else 1
    if not minimum_port <= endpoint.port <= 65535:
        raise BridgeConfigError("endpoint", f"port must be in range {minimum_port}-65535")


def _validate_sender(field: str, sender_id: SenderId) -> None:
    if not isinstance(sender_id, int) or not 1 <= sender_id <= 0xFFFFFFFF:
        raise BridgeConfigError(field, "must be a nonzero uint32")
