"""Startup policy for exposing the flight-command action."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path


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
