"""Prop-off-only programmable V7 characterization CLI."""

from __future__ import annotations

import argparse
import hashlib
import json
import select
import signal
import time
from dataclasses import dataclass
from pathlib import Path
from types import FrameType
from typing import Final

from .actions import (
    CommandRejectedError,
    CommandRequest,
    FlightActionController,
    ResultCode,
)
from .capability import (
    BEHAVIORS,
    COMMANDS,
    SCHEMA,
    VERSION,
    AckLatencyObservation,
    CapabilityReport,
    capability_report_document,
)
from .serial_port import ExclusiveSerialPort, SerialOpenError, SerialOwnershipError
from .session import NativeV7Bridge

EXIT_RED: Final = 2
EXIT_INVALID: Final = 3


@dataclass(frozen=True, slots=True)
class Probe:
    name: str
    request: CommandRequest


@dataclass(frozen=True, slots=True)
class ProbeResult:
    name: str
    supported: bool
    latency_ms: float | None
    reason: str


@dataclass(frozen=True, slots=True)
class ReportOutcome:
    probe_results: tuple[ProbeResult, ...]
    behaviors: tuple[tuple[str, bool], ...]
    reason: str


PROBES: Final = (
    Probe("target_position", CommandRequest.target_position(0, 0)),
    Probe("target_height", CommandRequest.target_height(0)),
    Probe("ascend", CommandRequest.ascend(0, 10)),
    Probe("descend", CommandRequest.descend(0, 10)),
    Probe("hover", CommandRequest.hover()),
    Probe("land", CommandRequest.land()),
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="ed-uav-v7-characterize")
    parser.add_argument("--device", required=True)
    parser.add_argument("--lock-dir", type=Path, default=Path("/tmp"))
    parser.add_argument("--device-identity", required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    return parser


def _probe(bridge: NativeV7Bridge, port: ExclusiveSerialPort, probe: Probe, timeout_s: float) -> ProbeResult:
    started = time.monotonic()
    bridge.start(probe.request, started, timeout_s)
    while True:
        now = time.monotonic()
        readable, _, _ = select.select((port.fileno,), (), (), 0.005)
        if readable:
            results = bridge.feed(port.read(), now)
            if results:
                result = results[0]
                return ProbeResult(
                    probe.name,
                    result.code is ResultCode.SUCCEEDED,
                    round((now - started) * 1000.0, 3),
                    result.reason,
                )
        timeout = bridge.tick(now)
        if timeout is not None:
            return ProbeResult(probe.name, False, None, timeout.reason)


def _discard(data: bytes) -> None:
    """Accept a frame for the isolated mutual-exclusion self-check."""


def _reason(support: tuple[tuple[str, bool], ...], behaviors: tuple[tuple[str, bool], ...]) -> str:
    unsupported = next((name for name, value in support if not value), None)
    failed = next((name for name, value in behaviors if not value), None)
    if unsupported is not None:
        return f"{unsupported} acknowledgement failed"
    if failed is not None:
        return f"{failed} characterization failed"
    return "fake PTY evidence cannot authorize field activation"


@dataclass(frozen=True, slots=True)
class CharacterizationInterruptedError(RuntimeError):
    signum: int

    def __str__(self) -> str:
        return f"characterization interrupted by signal {self.signum}"


def _interrupt(signum: int, frame: FrameType | None) -> None:
    raise CharacterizationInterruptedError(signum)


def _measured_safety_behaviors(timeout_s: float) -> tuple[tuple[str, bool], ...]:
    exclusion = FlightActionController(_discard)
    exclusion.start(CommandRequest.hover(), 0.0, timeout_s)
    try:
        exclusion.start(CommandRequest.land(), 0.0, timeout_s)
    except CommandRejectedError:
        mutual_exclusion = True
    else:
        mutual_exclusion = False

    loss = FlightActionController(_discard)
    request = CommandRequest.target_position(0, 0)
    loss.start(request, 0.0, timeout_s)
    timeout = loss.tick(timeout_s + 0.001)
    link_loss = timeout is not None and timeout.code is ResultCode.TIMEOUT
    try:
        loss.start(request, timeout_s + 0.002, timeout_s)
    except CommandRejectedError:
        retry = True
    else:
        retry = False
    return (
        ("cancellation", False),
        ("zero_motion", False),
        ("retry", retry),
        ("link_loss", link_loss),
        ("mutual_exclusion", mutual_exclusion),
    )


def _write_report(
    arguments: argparse.Namespace,
    outcome: ReportOutcome,
) -> CapabilityReport:
    artifact_path = arguments.report.with_suffix(arguments.report.suffix + ".artifact.jsonl")
    artifact_lines = tuple(
        json.dumps(
            {"command": result.name, "supported": result.supported, "latency_ms": result.latency_ms, "reason": result.reason},
            sort_keys=True,
            separators=(",", ":"),
        )
        for result in outcome.probe_results
    ) + tuple(
        json.dumps(
            {"behavior": name, "measured_pass": value},
            sort_keys=True,
            separators=(",", ":"),
        )
        for name, value in outcome.behaviors
    )
    artifact = ("\n".join(artifact_lines) + "\n").encode()
    artifact_path.write_bytes(artifact)
    programmable = tuple(
        result for result in outcome.probe_results if result.name in COMMANDS
    )
    report = CapabilityReport(
        schema=SCHEMA,
        version=VERSION,
        device_identity=arguments.device_identity,
        evidence_kind="fake_pty",
        command_support=tuple(
            (name, next((result.supported for result in programmable if result.name == name), False))
            for name in COMMANDS
        ),
        ack_latency_observations=tuple(
            AckLatencyObservation(result.name, result.latency_ms)
            for result in programmable
            if result.latency_ms is not None
        ),
        behavior_results=outcome.behaviors,
        artifact_sha256=hashlib.sha256(artifact).hexdigest(),
        passed=False,
        reason=outcome.reason,
        provenance_authority=None,
        integrity_hmac_sha256=None,
    )
    output = json.dumps(capability_report_document(report), sort_keys=True, separators=(",", ":"))
    arguments.report.write_text(output + "\n", encoding="utf-8")
    print(output, flush=True)
    return report


def run(arguments: argparse.Namespace) -> int:
    """Run the fixed non-arming probe sequence and retain one structured report."""
    if arguments.timeout_s <= 0:
        raise ValueError("timeout must be positive")
    probe_results: list[ProbeResult] = []
    with ExclusiveSerialPort(arguments.device, lock_dir=arguments.lock_dir) as port:
        bridge = NativeV7Bridge(port.write)
        for probe in PROBES:
            probe_results.append(_probe(bridge, port, probe, arguments.timeout_s))
    programmable = tuple(result for result in probe_results if result.name in COMMANDS)
    support = tuple((result.name, result.supported) for result in programmable)
    result_by_name = {result.name: result.supported for result in probe_results}
    measured = dict(_measured_safety_behaviors(arguments.timeout_s))
    behaviors = tuple(
        (
            name,
            all(value for _, value in support) if name == "ordering" else
            result_by_name[name] if name in ("hover", "land") else measured[name],
        )
        for name in BEHAVIORS
    )
    _write_report(
        arguments,
        ReportOutcome(tuple(probe_results), behaviors, _reason(support, behaviors)),
    )
    return EXIT_RED


def main() -> int:
    """Execute the bounded prop-off characterization surface."""
    arguments = _parser().parse_args()
    signal.signal(signal.SIGTERM, _interrupt)
    try:
        return run(arguments)
    except (
        CharacterizationInterruptedError,
        SerialOpenError,
        SerialOwnershipError,
        OSError,
        ValueError,
    ) as error:
        behaviors = tuple((name, False) for name in BEHAVIORS)
        _write_report(
            arguments,
            ReportOutcome(
                (),
                behaviors,
                f"link or characterization failure: {error}",
            ),
        )
        return EXIT_INVALID


if __name__ == "__main__":
    raise SystemExit(main())
