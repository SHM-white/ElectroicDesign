"""Bounded Linux PTY fake for the native V7 FCU bridge."""

from __future__ import annotations

import argparse
import os
import random
import select
import signal
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from types import FrameType
from typing import Final

from .fcu import DeterministicPtyFcu, position_v7_frame
from .v7 import V7Frame, decode_v7_frame, encode_v7_frame

DEFAULT_DURATION_SECONDS: Final = 30.0
DEFAULT_RATE_HZ: Final = 20.0
DEFAULT_SEED: Final = 7
MAX_DURATION_SECONDS: Final = 3_600.0
MAX_RATE_HZ: Final = 200.0
MAX_SEED: Final = (1 << 32) - 1
MAX_SELECT_WAIT_SECONDS: Final = 0.05
COMMAND_FRAME_ID: Final = 0xE0
READY_PREFIX: Final = "FAKE FCU READY:"

STATUS_FRAME: Final = encode_v7_frame(V7Frame(address=0xFF, frame_id=0x06, payload=bytes((3, 0))))
_AUX_PAYLOAD = bytearray(20)
struct.pack_into("<h", _AUX_PAYLOAD, 18, 1_800)
AUX_FRAME: Final = encode_v7_frame(V7Frame(address=0xFF, frame_id=0x40, payload=bytes(_AUX_PAYLOAD)))


@dataclass(frozen=True, slots=True)
class FakeFcuConfigurationError(Exception):
    """Raised when an external CLI value cannot describe a safe bounded fake."""

    reason: str

    def __str__(self) -> str:
        return self.reason


@dataclass(frozen=True, slots=True)
class FakeFcuConfig:
    """Validated runtime limits and caller-visible PTY path."""

    pty_device: Path
    duration_seconds: float
    rate_hz: float
    seed: int


class _BridgeFacingPtyFcu(DeterministicPtyFcu):
    """PTY adapter exposing the inherited master only to this bridge-facing fake."""

    def __init__(self) -> None:
        super().__init__()
        os.set_blocking(self._master_fd, False)

    def emit(self, frame: bytes) -> None:
        try:
            os.write(self._master_fd, frame)
        except BlockingIOError:
            return

    def receive(self, timeout_seconds: float) -> bytes:
        readable, _, _ = select.select((self._master_fd,), (), (), timeout_seconds)
        if not readable:
            return b""
        try:
            return os.read(self._master_fd, 4_096)
        except BlockingIOError:
            return b""


class _CommandStream:
    """Mutable accumulator for complete checksum-verified bridge commands."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> tuple[bytes, ...]:
        self._buffer.extend(chunk)
        frames: list[bytes] = []
        while len(self._buffer) >= 4:
            try:
                start = self._buffer.index(0xAA)
            except ValueError:
                self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 4:
                break
            frame_length = self._buffer[3] + 6
            if len(self._buffer) < frame_length:
                break
            raw = bytes(self._buffer[:frame_length])
            if decode_v7_frame(raw) is None:
                del self._buffer[0]
                continue
            del self._buffer[:frame_length]
            frames.append(raw)
        return tuple(frames)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="ed-uav-fake-fcu", description=__doc__)
    parser.add_argument("--pty-device", type=Path, required=True)
    parser.add_argument(
        "--duration-seconds",
        "--duration-s",
        dest="duration_seconds",
        type=float,
        default=DEFAULT_DURATION_SECONDS,
    )
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--ros-args", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("-r", dest="ros_remap", action="append", default=[], help=argparse.SUPPRESS)
    return parser


def parse_config(argv: list[str] | None = None) -> FakeFcuConfig:
    """Parse and validate the CLI trust boundary without touching any device."""
    arguments = _parser().parse_args(argv)
    config = FakeFcuConfig(
        pty_device=arguments.pty_device,
        duration_seconds=arguments.duration_seconds,
        rate_hz=arguments.rate_hz,
        seed=arguments.seed,
    )
    if sys.platform != "linux":
        raise FakeFcuConfigurationError("the fake FCU requires Linux openpty support")
    if not 0.0 < config.duration_seconds <= MAX_DURATION_SECONDS:
        raise FakeFcuConfigurationError(f"duration must be within (0, {MAX_DURATION_SECONDS:g}] seconds")
    if not 0.0 < config.rate_hz <= MAX_RATE_HZ:
        raise FakeFcuConfigurationError(f"rate must be within (0, {MAX_RATE_HZ:g}] Hz")
    if not 0 <= config.seed <= MAX_SEED:
        raise FakeFcuConfigurationError(f"seed must be within [0, {MAX_SEED}]")
    try:
        resolved_parent = config.pty_device.parent.resolve(strict=True)
    except FileNotFoundError as error:
        raise FakeFcuConfigurationError(f"PTY parent does not exist: {config.pty_device.parent}") from error
    if not resolved_parent.is_dir():
        raise FakeFcuConfigurationError(f"PTY parent is not a directory: {resolved_parent}")
    device_root = Path("/dev")
    if resolved_parent == device_root or device_root in resolved_parent.parents:
        raise FakeFcuConfigurationError("the fake FCU cannot create endpoints under /dev")
    return config


def _acknowledgement(command: bytes) -> bytes:
    payload = bytes((command[2], command[-2], command[-1]))
    return encode_v7_frame(V7Frame(address=0xFF, frame_id=0x00, payload=payload))


def _serve(config: FakeFcuConfig, endpoint: _BridgeFacingPtyFcu, stop_requested: threading.Event) -> None:
    generator = random.Random(config.seed)
    forward_origin_cm = generator.randint(-500, 500)
    right_origin_cm = generator.randint(-500, 500)
    decoder = _CommandStream()
    period_seconds = 1.0 / config.rate_hz
    started = time.monotonic()
    deadline = started + config.duration_seconds
    next_sample = started
    sequence = 0

    while not stop_requested.is_set():
        now = time.monotonic()
        if now >= deadline:
            break
        wait_seconds = min(MAX_SELECT_WAIT_SECONDS, max(0.0, next_sample - now), deadline - now)
        for command in decoder.feed(endpoint.receive(wait_seconds)):
            if command[2] == COMMAND_FRAME_ID:
                endpoint.emit(_acknowledgement(command))
        now = time.monotonic()
        if now < next_sample:
            continue
        sequence += 1
        endpoint.emit(position_v7_frame(forward_origin_cm + sequence * 3, right_origin_cm - sequence * 2))
        if sequence >= 3:
            endpoint.emit(STATUS_FRAME)
            endpoint.emit(AUX_FRAME)
        next_sample = max(next_sample + period_seconds, now + period_seconds)


def run(config: FakeFcuConfig) -> None:
    """Own one temporary PTY and remove its caller-visible link on every exit path."""
    if os.path.lexists(config.pty_device):
        raise FakeFcuConfigurationError(f"PTY path already exists: {config.pty_device}")

    stop_requested = threading.Event()

    def request_stop(signum: int, frame: FrameType | None) -> None:
        del signum, frame
        stop_requested.set()

    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    endpoint = _BridgeFacingPtyFcu()
    try:
        with endpoint:
            os.symlink(endpoint.slave_path, config.pty_device)
            try:
                print(f"{READY_PREFIX} {config.pty_device}", flush=True)
                _serve(config, endpoint, stop_requested)
            finally:
                config.pty_device.unlink()
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
        signal.signal(signal.SIGINT, previous_sigint)


def main(argv: list[str] | None = None) -> int:
    """Run the bounded fake and expose configuration or PTY failures on stderr."""
    try:
        run(parse_config(argv))
    except (FakeFcuConfigurationError, OSError) as error:
        print(f"FAKE FCU ERROR: {error}", file=sys.stderr, flush=True)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
