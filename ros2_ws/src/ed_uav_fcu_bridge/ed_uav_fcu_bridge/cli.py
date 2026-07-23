"""Manual PTY/serial surface for exercising native V7 bridge actions."""

from __future__ import annotations

import argparse
import select
import signal
import sys
import time
from pathlib import Path
from types import FrameType

from .actions import CommandKind, CommandRequest, ResultCode
from .serial_port import ExclusiveSerialPort, SerialOpenError, SerialOwnershipError
from .session import BridgeConfig, NativeV7Bridge
from .telemetry import FreshnessPolicy


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="ed-uav-fcu-bridge")
    parser.add_argument("--device", required=True)
    parser.add_argument("--lock-dir", type=Path, default=Path("/tmp"))
    parser.add_argument("--command", choices=("unlock", "mode", "takeoff", "move", "hover", "land", "lock"), required=True)
    parser.add_argument("--mode", type=int, default=3)
    parser.add_argument("--height-cm", type=int, default=150)
    parser.add_argument("--distance-cm", type=int, default=100)
    parser.add_argument("--speed-cmps", type=int, default=30)
    parser.add_argument("--direction-deg", type=int, default=90)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--wait-ready", action="store_true")
    parser.add_argument("--position-max-age-s", type=float, default=0.20)
    parser.add_argument("--aux-status-max-age-s", type=float, default=0.50)
    parser.add_argument("--link-max-age-s", type=float, default=0.50)
    return parser


def _request(arguments: argparse.Namespace) -> CommandRequest:
    match arguments.command:
        case "unlock":
            return CommandRequest.unlock()
        case "mode":
            return CommandRequest(CommandKind.SET_MODE, mode=arguments.mode)
        case "takeoff":
            return CommandRequest(CommandKind.TAKEOFF, height_cm=arguments.height_cm)
        case "move":
            return CommandRequest.move(arguments.distance_cm, arguments.speed_cmps, arguments.direction_deg)
        case "hover":
            return CommandRequest.hover()
        case "land":
            return CommandRequest.land()
        case "lock":
            return CommandRequest(CommandKind.LOCK)
        case _:  # pragma: no cover - argparse closes this set before dispatch.
            raise RuntimeError(f"unknown command: {arguments.command}")


def _result_exit_code(code: ResultCode) -> int:
    match code:
        case ResultCode.SUCCEEDED:
            return 0
        case ResultCode.TIMEOUT:
            return 2
        case ResultCode.REJECTED | ResultCode.FCU_ERROR:
            return 3
        case _:
            raise RuntimeError(f"unknown result code: {code}")


def run(arguments: argparse.Namespace) -> int:
    """Run one high-level command over the actual serial endpoint and print its result."""
    policy = FreshnessPolicy(
        arguments.position_max_age_s,
        arguments.aux_status_max_age_s,
        arguments.link_max_age_s,
    )
    request = _request(arguments)
    with ExclusiveSerialPort(arguments.device, lock_dir=arguments.lock_dir) as port:
        bridge = NativeV7Bridge(port.write, BridgeConfig(freshness=policy))
        started = False
        readiness_deadline = time.monotonic() + arguments.timeout_s
        while True:
            now = time.monotonic()
            readable, _, _ = select.select((port.fileno,), (), (), 0.01)
            if readable:
                bridge.feed(port.read(), now)
            if not started and arguments.wait_ready:
                if bridge.mission_ready(now):
                    print("READY mission prerequisites are fresh", flush=True)
                elif now > readiness_deadline:
                    print("RESULT code=REJECTED reason=mission prerequisites are not fresh", flush=True)
                    return 3
                else:
                    continue
            if not started:
                pending = bridge.start(request, now, arguments.timeout_s)
                print(f"SENT command={pending.command.name} frame={pending.raw.hex().upper()}", flush=True)
                started = True
            result = bridge.tick(now)
            if result is not None:
                print(f"RESULT code={result.code.name} reason={result.reason}", flush=True)
                return _result_exit_code(result.code)
            if bridge.actions.last_result is not None:
                result = bridge.actions.last_result
                print(f"RESULT code={result.code.name} reason={result.reason}", flush=True)
                return _result_exit_code(result.code)


def main() -> int:
    """Execute the standalone native-V7 action surface."""
    signal.signal(signal.SIGTERM, _terminate)
    arguments = _parser().parse_args()
    try:
        return run(arguments)
    except (SerialOwnershipError, SerialOpenError) as error:
        print(f"RESULT code=REJECTED reason={error}", file=sys.stderr, flush=True)
        return 3


def _terminate(signum: int, frame: FrameType | None) -> None:
    raise SystemExit(128 + signum)


if __name__ == "__main__":
    raise SystemExit(main())
