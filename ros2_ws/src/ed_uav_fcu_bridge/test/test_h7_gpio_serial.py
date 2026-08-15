"""Tests for H7GpioTransport frame I/O over a PTY endpoint."""

from __future__ import annotations

import os
import pty
import select
import sys
import threading
import time
from pathlib import Path
from typing import Final

import pytest

PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_fcu_bridge.h7_gpio_protocol import cmd_set_output, parse_response
from ed_uav_fcu_bridge.h7_gpio_serial import H7GpioTransport


def _open_pty_pair() -> tuple[int, str]:
    """Return (master_fd, slave_tty_path) for a PTY pair."""
    master_fd, slave_fd = pty.openpty()
    tty_path = os.ttyname(slave_fd)
    os.close(slave_fd)
    return master_fd, tty_path


def _drain_master(master_fd: int, timeout_s: float = 1.0) -> bytes:
    """Blocking read of all currently available master-side bytes."""
    chunks: list[bytes] = []
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        readable, _, _ = select.select([master_fd], [], [], 0.05)
        if not readable:
            if chunks:
                return b"".join(chunks)
            continue
        try:
            chunk = os.read(master_fd, 4096)
        except BlockingIOError:
            continue
        if not chunk:
            return b"".join(chunks)
        chunks.append(chunk)
    return b"".join(chunks)


def _echo_worker(master_fd: int, stop: threading.Event) -> None:
    """Echo received bytes back with a 0xBB response frame appended."""
    while not stop.is_set():
        readable, _, _ = select.select([master_fd], [], [], 0.05)
        if not readable:
            continue
        try:
            chunk = os.read(master_fd, 4096)
        except (BlockingIOError, OSError):
            continue
        if not chunk:
            continue
        if chunk[0] == 0xAA:
            pin = chunk[1]
            command = chunk[2]
            response = bytes((0xBB, pin, command, 0x00))
            checksum = response[1] ^ response[2] ^ response[3]
            os.write(master_fd, response + bytes((checksum,)))


@pytest.fixture()
def transport_fixture(tmp_path: Path):
    """Yield one transport/PTY pair without a competing reader thread."""
    master_fd, tty_path = _open_pty_pair()
    transport = H7GpioTransport(tty_path, 115200, lock_dir=tmp_path)
    transport.open()
    try:
        yield transport, master_fd
    finally:
        transport.close()
        os.close(master_fd)


@pytest.fixture()
def echo_transport_fixture(transport_fixture):
    """Add a joined echo peer only for tests that need acknowledgements."""
    transport, master_fd = transport_fixture
    stop = threading.Event()
    worker = threading.Thread(target=_echo_worker, args=(master_fd, stop), daemon=True)
    worker.start()
    try:
        yield transport, master_fd
    finally:
        stop.set()
        worker.join(timeout=1.0)
        assert not worker.is_alive()


def test_send_writes_frame_bytes(transport_fixture) -> None:
    transport, master_fd = transport_fixture
    transport.send(cmd_set_output(2, True))
    observed = _drain_master(master_fd)
    assert observed == cmd_set_output(2, True)


def test_read_response_gets_echoed_ack(echo_transport_fixture) -> None:
    transport, master_fd = echo_transport_fixture
    transport.send(cmd_set_output(2, True))
    response = transport.read_response(timeout_s=1.0)
    assert response is not None
    assert response.pin == 2
    assert response.command == 0x01
    assert response.status == 0x00


def test_read_response_timeout_without_peer(transport_fixture) -> None:
    transport, _ = transport_fixture
    # 不发送任何帧, 对端不会回响应; 应超时返回 None。
    start = time.monotonic()
    response = transport.read_response(timeout_s=0.2)
    assert response is None
    assert time.monotonic() - start >= 0.19


def test_transport_rejects_stale_bytes_before_header(echo_transport_fixture) -> None:
    transport, master_fd = echo_transport_fixture
    # 在响应帧前注入一个 0x55 脏字节。
    os.write(master_fd, b"\x55")
    transport.send(cmd_set_output(2, True))
    response = transport.read_response(timeout_s=1.0)
    assert response is not None
    assert response.pin == 2
