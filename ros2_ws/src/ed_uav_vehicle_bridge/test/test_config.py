from pathlib import Path

import pytest

from ed_uav_vehicle_bridge.config import BridgeProvisioning, load_bridge_config
from ed_uav_vehicle_bridge.errors import BridgeConfigError
from ed_uav_vehicle_bridge.models import Endpoint, SenderId


def _provisioning(key_file: Path) -> BridgeProvisioning:
    return BridgeProvisioning(
        bind=Endpoint("0.0.0.0", 40100),
        car_peer=Endpoint("127.0.0.1", 40101),
        hmi_peer=Endpoint("127.0.0.1", 40102),
        car_sender_id=SenderId(0x43415231),
        hmi_sender_id=SenderId(0x484D4931),
        bridge_sender_id=SenderId(0x524F5331),
        hmac_key_file=key_file,
        mission_timeout_seconds=90.0,
        telemetry_stale_seconds=0.75,
    )


def test_local_key_and_exact_peer_provisioning_are_required(tmp_path: Path) -> None:
    key_file = tmp_path / "udp.key"
    key_file.write_text(bytes(range(32)).hex(), encoding="ascii")

    config = load_bridge_config(_provisioning(key_file))

    assert config.hmac_key == bytes(range(32))
    assert config.provisioning.car_peer == Endpoint("127.0.0.1", 40101)


def test_missing_short_key_and_unspecified_peer_fail_closed(tmp_path: Path) -> None:
    missing = tmp_path / "missing.key"
    short = tmp_path / "short.key"
    short.write_text(b"short".hex(), encoding="ascii")

    with pytest.raises(BridgeConfigError):
        load_bridge_config(_provisioning(missing))
    with pytest.raises(BridgeConfigError):
        load_bridge_config(_provisioning(short))
    invalid_peer = _provisioning(short)
    invalid_peer = BridgeProvisioning(
        bind=invalid_peer.bind,
        car_peer=Endpoint("0.0.0.0", 40101),
        hmi_peer=invalid_peer.hmi_peer,
        car_sender_id=invalid_peer.car_sender_id,
        hmi_sender_id=invalid_peer.hmi_sender_id,
        bridge_sender_id=invalid_peer.bridge_sender_id,
        hmac_key_file=invalid_peer.hmac_key_file,
        mission_timeout_seconds=invalid_peer.mission_timeout_seconds,
        telemetry_stale_seconds=invalid_peer.telemetry_stale_seconds,
    )
    with pytest.raises(BridgeConfigError):
        load_bridge_config(invalid_peer)
