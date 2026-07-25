"""Bounded command-line surface for deterministic offline verification replay."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path

from .artifacts import ArtifactExistsError, EventArtifactWriter, FixtureBagBuilder, IncompleteScenarioError
from .faults import FaultKind, FaultWindow
from .model import ScenarioBoundError, ScenarioConfig, ScenarioConfigurationError, Stream
from .rosbag import Rosbag2UnavailableError, RosbagFixtureBuilder
from .scenario import DeterministicScenario


@dataclass(frozen=True, slots=True)
class ScenarioCliError(Exception):
    """Raised when an untrusted CLI field cannot become a typed scenario value."""

    reason: str

    def __str__(self) -> str:
        return self.reason


@dataclass(frozen=True, slots=True)
class CliRequest:
    """Fully parsed deterministic CLI request."""

    config: ScenarioConfig
    event_path: Path
    fixture_root: Path | None
    rosbag_root: Path | None


def parse_fault(raw: str) -> FaultWindow:
    """Parse `kind:stream:start_tick:duration_ticks` at the CLI trust boundary."""
    parts = raw.split(":")
    if len(parts) != 4:
        raise ScenarioCliError("fault must be kind:stream:start_tick:duration_ticks")
    try:
        kind = FaultKind(parts[0])
        stream = Stream(parts[1])
        start_tick = int(parts[2])
        duration_ticks = int(parts[3])
    except ValueError as error:
        raise ScenarioCliError(f"malformed fault: {raw}") from error
    return FaultWindow(kind=kind, stream=stream, start_tick=start_tick, duration_ticks=duration_ticks)


def parse_request(argv: list[str] | None = None) -> CliRequest:
    """Parse all external CLI fields into an immutable scenario configuration."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--duration-seconds", type=int, default=60)
    parser.add_argument("--rate-hz", type=int, default=20)
    parser.add_argument("--max-ticks", type=int, default=120_000)
    parser.add_argument("--fault", action="append", default=[])
    parser.add_argument("--event-json", required=True)
    parser.add_argument("--fixture-dir")
    parser.add_argument("--rosbag-dir")
    parsed = parser.parse_args(argv)
    faults = tuple(parse_fault(raw) for raw in parsed.fault)
    config = ScenarioConfig(
        seed=parsed.seed,
        duration_seconds=parsed.duration_seconds,
        rate_hz=parsed.rate_hz,
        faults=faults,
        max_ticks=parsed.max_ticks,
    )
    fixture_root = Path(parsed.fixture_dir) if parsed.fixture_dir is not None else None
    rosbag_root = Path(parsed.rosbag_dir) if parsed.rosbag_dir is not None else None
    return CliRequest(
        config=config,
        event_path=Path(parsed.event_json),
        fixture_root=fixture_root,
        rosbag_root=rosbag_root,
    )


def _validate_output_paths(request: CliRequest) -> None:
    """Reject stale or overlapping destinations before the first artifact write."""
    destinations = [request.event_path]
    if request.fixture_root is not None:
        destinations.append(request.fixture_root)
    if request.rosbag_root is not None:
        destinations.append(request.rosbag_root)

    resolved_destinations = tuple(destination.resolve(strict=False) for destination in destinations)
    for index, left in enumerate(resolved_destinations):
        for right in resolved_destinations[index + 1 :]:
            if left == right or left in right.parents or right in left.parents:
                raise ScenarioCliError("artifact output paths must not overlap")

    for destination in destinations:
        if destination.exists() or destination.is_symlink():
            raise ArtifactExistsError(destination)
        partial = destination.with_name(f"{destination.name}.partial")
        if partial.exists() or partial.is_symlink():
            raise ArtifactExistsError(partial)


def main(argv: list[str] | None = None) -> int:
    """Run a deterministic scenario and emit success only after complete artifact persistence."""
    try:
        request = parse_request(argv)
        _validate_output_paths(request)
        rosbag_builder = RosbagFixtureBuilder(request.rosbag_root) if request.rosbag_root is not None else None
        if rosbag_builder is not None:
            rosbag_builder.validate_runtime()
        report = DeterministicScenario(request.config).run()
        event_written = False
        fixture_written = False
        artifacts_completed = False
        try:
            EventArtifactWriter(request.event_path).write(report)
            event_written = True
            if request.fixture_root is not None:
                FixtureBagBuilder(request.fixture_root).write(report)
                fixture_written = True
            if rosbag_builder is not None:
                rosbag_builder.write(report)
            artifacts_completed = True
        finally:
            if not artifacts_completed:
                if fixture_written and request.fixture_root is not None:
                    shutil.rmtree(request.fixture_root, ignore_errors=True)
                if event_written:
                    request.event_path.unlink(missing_ok=True)
    except (
        ArtifactExistsError,
        IncompleteScenarioError,
        Rosbag2UnavailableError,
        ScenarioBoundError,
        ScenarioCliError,
        ScenarioConfigurationError,
    ) as error:
        print(f"SCENARIO: RED: {error}", file=sys.stderr)
        return 2
    digest = hashlib.sha256(report.event_json).hexdigest()
    summary = json.dumps(
        {"event_sha256": digest, "simulated_duration_ns": report.simulated_duration_ns, "ticks": report.tick_count},
        separators=(",", ":"),
        sort_keys=True,
    )
    print(f"SCENARIO: GREEN {summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
