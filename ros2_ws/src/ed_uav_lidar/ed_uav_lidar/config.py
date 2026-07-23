"""Typed lidar launch configuration and field preflight contract."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Final, Mapping


ConfigValue = bool | str
PLACEHOLDER_DRIVER_CONFIG_SUFFIX: Final = "/config/mid360_driver.json"


class Transport(str, Enum):
    DISABLED = "disabled"
    GENERIC = "generic"
    MID360 = "mid360"


class TimeAuthority(str, Enum):
    HOST = "host"
    PTP = "ptp"


@dataclass(frozen=True, slots=True)
class ConfigurationError(Exception):
    field: str
    value: str

    def __str__(self) -> str:
        return f"invalid {self.field}: {self.value}"


@dataclass(frozen=True, slots=True)
class FieldCheck:
    ready: bool
    missing: tuple[str, ...]

    @property
    def code(self) -> str:
        return "LIDAR_FIELD_CONFIGURATION_READY" if self.ready else "LIDAR_FIELD_CONFIGURATION_INCOMPLETE"


@dataclass(frozen=True, slots=True)
class LidarConfig:
    enabled: bool
    transport: Transport
    serial_number: str
    sensor_ip: str
    firmware_version: str
    time_authority: TimeAuthority
    driver_config_path: str
    monitoring_topic: str
    imu_topic: str
    generic_input_topic: str
    fastlio_custom_topic: str
    field_check: FieldCheck

    @property
    def requires_livox(self) -> bool:
        return self.enabled and self.transport is Transport.MID360

    @property
    def time_status(self) -> str:
        match self.time_authority:
            case TimeAuthority.HOST:
                return "HOST_TIME_UNVERIFIED"
            case TimeAuthority.PTP:
                return "PTP_CONFIGURED_UNVERIFIED"


def _read_bool(raw: Mapping[str, ConfigValue], key: str, default: bool) -> bool:
    value = raw.get(key, default)
    if isinstance(value, bool):
        return value
    if value == "true":
        return True
    if value == "false":
        return False
    raise ConfigurationError(field=key, value=str(value))


def _read_string(raw: Mapping[str, ConfigValue], key: str, default: str) -> str:
    value = raw.get(key, default)
    if not isinstance(value, str) or not value:
        raise ConfigurationError(field=key, value=str(value))
    return value


def _parse_transport(value: str) -> Transport:
    match value:
        case "disabled":
            return Transport.DISABLED
        case "generic":
            return Transport.GENERIC
        case "mid360":
            return Transport.MID360
        case _:
            raise ConfigurationError(field="transport", value=value)


def _parse_time_authority(value: str) -> TimeAuthority:
    match value:
        case "host":
            return TimeAuthority.HOST
        case "ptp":
            return TimeAuthority.PTP
        case _:
            raise ConfigurationError(field="time_authority", value=value)


def _field_check(config: LidarConfig) -> FieldCheck:
    if not config.requires_livox:
        return FieldCheck(ready=True, missing=())
    missing = tuple(
        field
        for field, value in (
            ("serial_number", config.serial_number),
            ("sensor_ip", config.sensor_ip),
            ("firmware_version", config.firmware_version),
        )
        if value in {"UNSET", "0.0.0.0"}
    )
    if config.driver_config_path == "config/mid360_driver.json" or config.driver_config_path.endswith(
        PLACEHOLDER_DRIVER_CONFIG_SUFFIX
    ):
        missing = (*missing, "driver_config_path")
    return FieldCheck(ready=not missing, missing=missing)


def normalize_config(raw: Mapping[str, ConfigValue]) -> LidarConfig:
    """Parse launch values once into a typed configuration with a field gate."""
    enabled = _read_bool(raw, "lidar_enabled", False)
    transport = _parse_transport(_read_string(raw, "transport", "disabled"))
    if enabled and transport is Transport.DISABLED:
        raise ConfigurationError(field="transport", value="disabled with lidar_enabled=true")
    if not enabled and transport is not Transport.DISABLED:
        raise ConfigurationError(field="transport", value=f"{transport} with lidar_enabled=false")
    base = LidarConfig(
        enabled=enabled,
        transport=transport,
        serial_number=_read_string(raw, "serial_number", "UNSET"),
        sensor_ip=_read_string(raw, "sensor_ip", "0.0.0.0"),
        firmware_version=_read_string(raw, "firmware_version", "UNSET"),
        time_authority=_parse_time_authority(_read_string(raw, "time_authority", "host")),
        driver_config_path=_read_string(raw, "driver_config_path", "config/mid360_driver.json"),
        monitoring_topic=_read_string(raw, "monitoring_topic", "/lidar/points"),
        imu_topic=_read_string(raw, "imu_topic", "/lidar/imu"),
        generic_input_topic=_read_string(raw, "generic_input_topic", "/lidar/input/points"),
        fastlio_custom_topic=_read_string(raw, "fastlio_custom_topic", "/livox/lidar"),
        field_check=FieldCheck(ready=True, missing=()),
    )
    return LidarConfig(
        enabled=base.enabled,
        transport=base.transport,
        serial_number=base.serial_number,
        sensor_ip=base.sensor_ip,
        firmware_version=base.firmware_version,
        time_authority=base.time_authority,
        driver_config_path=base.driver_config_path,
        monitoring_topic=base.monitoring_topic,
        imu_topic=base.imu_topic,
        generic_input_topic=base.generic_input_topic,
        fastlio_custom_topic=base.fastlio_custom_topic,
        field_check=_field_check(base),
    )
