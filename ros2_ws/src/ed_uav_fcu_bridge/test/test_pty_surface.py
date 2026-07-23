from __future__ import annotations

import os
import pty
import selectors
import subprocess
import sys
import time
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_fcu_bridge.serial_port import ExclusiveSerialPort, SerialOwnershipError  # noqa: E402
from ed_uav_fcu_bridge.v7_codec import V7StreamDecoder, build_frame, decode_frame  # noqa: E402


def fresh_telemetry() -> bytes:
    position = build_frame(0xFF, 0x08, (100).to_bytes(4, "little", signed=True) + (50).to_bytes(4, "little", signed=True))
    status = build_frame(0xFF, 0x06, bytes((3, 1, 0, 0, 0)))
    channels = b"".join((1500).to_bytes(2, "little", signed=True) for _ in range(9))
    aux = build_frame(0xFF, 0x40, channels + (1800).to_bytes(2, "little", signed=True))
    return position + aux + status


def read_frame(fd: int, timeout_s: float) -> bytes:
    selector = selectors.DefaultSelector()
    selector.register(fd, selectors.EVENT_READ)
    decoder = V7StreamDecoder()
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            events = selector.select(deadline - time.monotonic())
            if not events:
                continue
            for _key, _mask in events:
                try:
                    chunk = os.read(fd, 1024)
                except OSError:
                    continue
                frames = decoder.feed(chunk)
                if frames:
                    return frames[0].raw
    finally:
        selector.close()
    raise AssertionError("PTY did not receive a native V7 frame before its deadline")


def drive_fresh_telemetry_until_command(fd: int, timeout_s: float) -> bytes:
    selector = selectors.DefaultSelector()
    selector.register(fd, selectors.EVENT_READ)
    decoder = V7StreamDecoder()
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            try:
                os.write(fd, fresh_telemetry())
            except OSError:
                pass
            events = selector.select(0.02)
            for _key, _mask in events:
                try:
                    chunk = os.read(fd, 1024)
                except OSError:
                    continue
                frames = decoder.feed(chunk)
                if frames:
                    return frames[0].raw
    finally:
        selector.close()
    raise AssertionError("PTY did not receive a command after fresh telemetry")


def cli_environment() -> dict[str, str]:
    environment = os.environ.copy()
    existing = environment.get("PYTHONPATH", "")
    environment["PYTHONPATH"] = f"{PACKAGE_ROOT}{os.pathsep}{existing}" if existing else str(PACKAGE_ROOT)
    return environment


def test_cli_drives_fresh_pty_frames_and_reports_ack_success(tmp_path: Path) -> None:
    # Given: a real PTY and the bridge CLI waiting for fresh mission prerequisites.
    master_fd, slave_fd = pty.openpty()
    device = os.ttyname(slave_fd)
    os.close(slave_fd)
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "ed_uav_fcu_bridge.cli",
            "--device",
            device,
            "--lock-dir",
            str(tmp_path),
            "--wait-ready",
            "--command",
            "move",
            "--timeout-s",
            "0.5",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=cli_environment(),
    )
    try:
        command = drive_fresh_telemetry_until_command(master_fd, timeout_s=1.0)

        # When: the PTY peer returns the matching V7 acknowledgement.
        sent = decode_frame(command)
        os.write(master_fd, build_frame(0xFF, 0x00, bytes((sent.frame_id, sent.sum_check, sent.add_check))))
        stdout, stderr = process.communicate(timeout=2.0)

        # Then: the executable exits successfully with unambiguous lifecycle output.
        assert process.returncode == 0, stderr
        assert "READY" in stdout
        assert "SENT command=MOVE" in stdout
        assert "RESULT code=SUCCEEDED" in stdout
    finally:
        os.close(master_fd)
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=2.0)


def test_cli_reports_timeout_and_releases_pty_for_restart(tmp_path: Path) -> None:
    # Given: a hover command whose PTY peer deliberately never acknowledges it.
    master_fd, slave_fd = pty.openpty()
    device = os.ttyname(slave_fd)
    os.close(slave_fd)
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "ed_uav_fcu_bridge.cli",
            "--device",
            device,
            "--lock-dir",
            str(tmp_path),
            "--command",
            "hover",
            "--timeout-s",
            "0.05",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=cli_environment(),
    )
    try:
        command = read_frame(master_fd, timeout_s=1.0)

        # When: the ACK deadline expires.
        stdout, stderr = process.communicate(timeout=2.0)

        # Then: failure is explicit, the hover wire command is native, and a restart can own the PTY.
        assert command[4:7] == bytes((0x10, 0x00, 0x04))
        assert process.returncode == 2, stderr
        assert "RESULT code=TIMEOUT" in stdout
        with ExclusiveSerialPort(device, lock_dir=tmp_path):
            pass
    finally:
        os.close(master_fd)
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=2.0)


def test_second_process_cannot_take_the_owned_serial_endpoint(tmp_path: Path) -> None:
    # Given: one process already owns the PTY through the bridge OS lock and exclusive open.
    master_fd, slave_fd = pty.openpty()
    device = os.ttyname(slave_fd)
    os.close(slave_fd)
    owner = ExclusiveSerialPort(device, lock_dir=tmp_path)
    owner.open()
    probe = "\n".join(
        (
            "from pathlib import Path",
            "from ed_uav_fcu_bridge.serial_port import ExclusiveSerialPort, SerialOwnershipError",
            f"port = ExclusiveSerialPort({device!r}, lock_dir=Path({str(tmp_path)!r}))",
            "try:",
            "    port.open()",
            "except SerialOwnershipError:",
            "    raise SystemExit(3)",
            "raise SystemExit(0)",
        )
    )
    try:
        # When: a separate process attempts to acquire the same endpoint.
        contender = subprocess.run(
            [sys.executable, "-c", probe],
            capture_output=True,
            text=True,
            env=cli_environment(),
            check=False,
            timeout=2.0,
        )

        # Then: contention has a binary failure result rather than a shared serial stream.
        assert contender.returncode == 3, contender.stderr
    finally:
        owner.close()
        os.close(master_fd)


def test_interrupt_cleanup_allows_a_new_owner(tmp_path: Path) -> None:
    # Given: a long-running CLI that has acquired the PTY and sent its command.
    master_fd, slave_fd = pty.openpty()
    device = os.ttyname(slave_fd)
    os.close(slave_fd)
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "ed_uav_fcu_bridge.cli",
            "--device",
            device,
            "--lock-dir",
            str(tmp_path),
            "--command",
            "hover",
            "--timeout-s",
            "20.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=cli_environment(),
    )
    try:
        read_frame(master_fd, timeout_s=1.0)

        # When: the owner is interrupted.
        process.terminate()
        process.communicate(timeout=2.0)

        # Then: both the OS lock and exclusive PTY claim have been released.
        with ExclusiveSerialPort(device, lock_dir=tmp_path):
            pass
    finally:
        os.close(master_fd)
        if process.poll() is None:
            process.kill()
            process.wait(timeout=2.0)
