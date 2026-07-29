"""Mission-side preflight adapter for Todo 2's V7 capability authority."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from ed_uav_fcu_bridge.authority import (
    ProgrammableCapabilityError,
    capability_trust_from_environment,
    require_programmable_capability,
)


@dataclass(frozen=True, slots=True)
class CapabilityDecision:
    ready: bool
    reason: str


def evaluate_d_task_capability(
    *,
    simulation_only: bool,
    report_path: Path,
    device_identity: str,
    environment: Mapping[str, str],
) -> CapabilityDecision:
    """Permit deterministic simulation while applying the field report gate."""
    if simulation_only:
        return CapabilityDecision(
            ready=True,
            reason="simulation/replay uses the deterministic FlightCommand fake",
        )
    try:
        trust = capability_trust_from_environment(
            report_path,
            device_identity,
            environment,
        )
        _ = require_programmable_capability(True, trust)
    except ProgrammableCapabilityError as error:
        return CapabilityDecision(ready=False, reason=str(error))
    return CapabilityDecision(ready=True, reason="verified programmable capability")
