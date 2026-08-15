"""Compatibility helpers for enabling the flight-command action.

Caller authentication is a deployment/network concern in this project.  The
runtime deliberately does not duplicate it with SROS2 environment checks or a
locally signed capability-report gate.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass


def require_flight_command_authority(
    enabled: bool,
    environment: Mapping[str, str],
) -> bool:
    """Return the explicit operator setting; network admission is external."""
    del environment
    return bool(enabled)
