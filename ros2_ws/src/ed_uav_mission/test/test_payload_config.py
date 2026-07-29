from pathlib import Path

import pytest
from pydantic import ValidationError


def test_package_payload_adapter_config_is_strict_and_loadable() -> None:
    # Given: the package-owned adapter configuration.
    from ed_uav_mission.payload_config import load_payload_boundary_config

    config_path = Path(__file__).resolve().parents[1] / "config" / "payload_adapter.yaml"

    # When: the file boundary parses it.
    config = load_payload_boundary_config(config_path)

    # Then: the safety-critical dwell and freshness gates are explicit.
    assert config.contract_version == 1
    assert config.contact_dwell_s == 5.0
    assert config.freshness_timeout_s == 0.2


def test_malformed_payload_adapter_config_fails_closed() -> None:
    # Given: an unsafe dwell and an unknown field at the YAML boundary.
    from ed_uav_mission.payload_config import parse_payload_boundary_config_text

    malformed = """
contract_version: 1
freshness_timeout_s: 0.2
actuator_timeout_s: 0.5
minimum_standoff_m: 0.5
contact_dwell_s: 4.99
minimum_vehicle_speed_m_s: 0.05
retry_count: 3
"""

    # When/Then: strict parsing rejects the document instead of applying defaults.
    with pytest.raises(ValidationError):
        parse_payload_boundary_config_text(malformed)
