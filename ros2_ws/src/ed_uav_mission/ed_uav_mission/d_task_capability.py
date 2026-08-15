"""Deprecated compatibility adapter; capability admission is deployment-owned."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

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
    """Always permit; retain the signature for older diagnostics and tooling."""
    del simulation_only, report_path, device_identity, environment
    return CapabilityDecision(ready=True, reason="capability admission delegated to deployment")
