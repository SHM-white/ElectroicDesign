"""Atomic deterministic event and portable bag-fixture artifact writers."""

from __future__ import annotations

import json
import shutil
from dataclasses import dataclass
from pathlib import Path

from .model import ScenarioReport


@dataclass(frozen=True, slots=True)
class ArtifactExistsError(Exception):
    """Raised when an existing artifact could hide a stale scenario result."""

    path: Path

    def __str__(self) -> str:
        return f"artifact already exists: {self.path}"


@dataclass(frozen=True, slots=True)
class IncompleteScenarioError(Exception):
    """Raised when an interrupted replay is presented as a final artifact."""

    def __str__(self) -> str:
        return "cannot persist an incomplete scenario as a final artifact"


@dataclass(frozen=True, slots=True)
class FixtureBag:
    """Portable event fixture paths; ROS-specific bag conversion is optional at runtime."""

    root: Path
    event_path: Path
    metadata_path: Path


@dataclass(frozen=True, slots=True)
class EventArtifactWriter:
    """Write one canonical event JSON file without retaining a stale partial file."""

    path: Path

    def write(self, report: ScenarioReport) -> Path:
        """Atomically write a complete report or reject an existing/incomplete output."""
        if not report.completed:
            raise IncompleteScenarioError()
        if self.path.exists():
            raise ArtifactExistsError(self.path)
        partial = self.path.with_name(f"{self.path.name}.partial")
        if partial.exists():
            raise ArtifactExistsError(partial)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        try:
            partial.write_bytes(report.event_json)
            partial.replace(self.path)
        except OSError:
            partial.unlink(missing_ok=True)
            raise
        return self.path


@dataclass(frozen=True, slots=True)
class FixtureBagBuilder:
    """Build an atomic portable replay fixture ready for ROS bag conversion."""

    root: Path

    def write(self, report: ScenarioReport) -> FixtureBag:
        """Persist a complete deterministic replay and minimal provenance metadata."""
        if not report.completed:
            raise IncompleteScenarioError()
        if self.root.exists():
            raise ArtifactExistsError(self.root)
        partial = self.root.with_name(f"{self.root.name}.partial")
        if partial.exists():
            raise ArtifactExistsError(partial)
        partial.mkdir(parents=True)
        event_path = partial / "events.json"
        metadata_path = partial / "metadata.json"
        try:
            event_path.write_bytes(report.event_json)
            metadata_path.write_text(
                json.dumps(
                    {
                        "event_file": "events.json",
                        "format": "ed_uav_verification.portable_event_fixture.v1",
                        "seed": report.config.seed,
                        "simulated_duration_ns": report.simulated_duration_ns,
                    },
                    separators=(",", ":"),
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
            partial.replace(self.root)
        except OSError:
            shutil.rmtree(partial, ignore_errors=True)
            raise
        return FixtureBag(root=self.root, event_path=self.root / "events.json", metadata_path=self.root / "metadata.json")
