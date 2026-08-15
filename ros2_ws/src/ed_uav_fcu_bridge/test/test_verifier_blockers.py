from __future__ import annotations

import json
import os
import pty
import signal
import subprocess
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_fcu_bridge.actions import (
    CommandRequest,
    FlightActionController,
)
from ed_uav_fcu_bridge.capability import load_capability_report
from ed_uav_fcu_bridge.serial_port import ExclusiveSerialPort
from ed_uav_fcu_bridge.v7_codec import build_frame, decode_frame
from test_pty_surface import cli_environment, read_frame


def characterizer_command(device: str, lock_dir: Path, report: Path) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "ed_uav_fcu_bridge.characterize",
        "--device", device,
        "--lock-dir", str(lock_dir),
        "--device-identity", "fake-pty-fcu-001",
        "--report", str(report),
        "--timeout-s", "0.1",
    ]
    help_result = subprocess.run(
        command[:3] + ["--help"],
        capture_output=True,
        text=True,
        env=cli_environment(),
        check=False,
    )
    if "--observed-cancellation" in help_result.stdout:
        command.extend(
            [
                "--observed-cancellation", "pass",
                "--observed-link-loss", "pass",
                "--observed-zero-motion", "pass",
                "--observed-retry", "pass",
            ]
        )
    return command


def acknowledge_six_probes(master_fd: int) -> tuple[bytes, ...]:
    commands: list[bytes] = []
    for _ in range(6):
        raw = read_frame(master_fd, timeout_s=1.0)
        commands.append(raw)
        sent = decode_frame(raw)
        os.write(
            master_fd,
            build_frame(0xFF, 0x00, bytes((sent.frame_id, sent.sum_check, sent.add_check))),
        )
    return tuple(commands)


def test_fake_pty_cannot_claim_physical_green_capability(tmp_path: Path) -> None:
    # Given: a fake PTY that acknowledges every command and claims physical flags when available.
    master_fd, slave_fd = pty.openpty()
    device = os.ttyname(slave_fd)
    os.close(slave_fd)
    report_path = tmp_path / "fake-physical.json"
    command = characterizer_command(device, tmp_path, report_path)
    help_text = subprocess.run(
        command[:3] + ["--help"], capture_output=True, text=True, env=cli_environment(), check=False
    ).stdout
    if "--evidence-kind" in help_text:
        command.extend(("--evidence-kind", "physical_prop_off", "--propellers-removed"))
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=cli_environment(),
    )
    try:
        acknowledge_six_probes(master_fd)

        # When: the characterization completes against the fake peer.
        stdout, stderr = process.communicate(timeout=2.0)

        # Then: neither flags nor ACKs can create field-green physical evidence.
        assert process.returncode != 0, stderr
        report = load_capability_report(report_path)
        assert report.passed is False
        assert report.evidence_kind == "fake_pty"
        assert json.loads(stdout)["passed"] is False
    finally:
        os.close(master_fd)
        if process.poll() is None:
            process.kill()
            process.wait(timeout=2.0)


def test_identical_command_can_be_issued_again_after_terminal_ack() -> None:
    # Given: one target-position command completed with its matching ACK.
    written: list[bytes] = []
    controller = FlightActionController(written.append)
    request = CommandRequest.target_position(0, 0)
    first = controller.start(request, steady_now=1.0, timeout_s=0.5)
    sent = decode_frame(first.raw)
    ack = decode_frame(build_frame(0xFF, 0x00, bytes((sent.frame_id, sent.sum_check, sent.add_check))))
    assert controller.handle_frame(ack, steady_now=1.1) is not None

    # When: the same command is issued after the first command is terminal.
    second = controller.start(request, steady_now=1.2, timeout_s=0.5)
    completed = controller.handle_frame(ack, steady_now=1.3)

    # Then: ordinary retries are not blocked by a process-lifetime lock.
    assert second.command is request.command
    assert completed is not None
    assert completed.acknowledged is True


def test_cli_has_no_user_asserted_behavior_pass_flags() -> None:
    # Given / When: the characterization CLI help is rendered.
    result = subprocess.run(
        [sys.executable, "-m", "ed_uav_fcu_bridge.characterize", "--help"],
        capture_output=True,
        text=True,
        env=cli_environment(),
        check=False,
    )

    # Then: required behaviors cannot be supplied as operator pass assertions.
    assert result.returncode == 0
    assert "--observed-" not in result.stdout


def test_link_loss_writes_structured_red_report_and_artifact(tmp_path: Path) -> None:
    # Given: a characterizer whose PTY peer disappears after the first command.
    master_fd, slave_fd = pty.openpty()
    device = os.ttyname(slave_fd)
    os.close(slave_fd)
    report_path = tmp_path / "link-loss.json"
    process = subprocess.Popen(
        characterizer_command(device, tmp_path, report_path),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=cli_environment(),
    )
    read_frame(master_fd, timeout_s=1.0)

    # When: the physical link disappears.
    os.close(master_fd)
    stdout, stderr = process.communicate(timeout=2.0)

    # Then: failure still retains parseable red report and artifact evidence.
    assert process.returncode != 0, stderr
    report = load_capability_report(report_path)
    assert report.passed is False
    assert "link" in report.reason.lower() or "input/output" in report.reason.lower()
    assert report_path.with_suffix(report_path.suffix + ".artifact.jsonl").is_file()
    assert json.loads(stdout)["passed"] is False


def test_sigterm_writes_red_report_and_releases_serial_ownership(tmp_path: Path) -> None:
    # Given: an active characterizer that owns a PTY and awaits its first ACK.
    master_fd, slave_fd = pty.openpty()
    device = os.ttyname(slave_fd)
    os.close(slave_fd)
    report_path = tmp_path / "interrupted.json"
    command = characterizer_command(device, tmp_path, report_path)
    timeout_index = command.index("--timeout-s") + 1
    command[timeout_index] = "60"
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=cli_environment(),
    )
    try:
        read_frame(master_fd, timeout_s=1.0)

        # When: SIGTERM interrupts the active serial wait.
        process.send_signal(signal.SIGTERM)
        stdout, stderr = process.communicate(timeout=2.0)

        # Then: interruption evidence is retained and the endpoint can be reacquired immediately.
        assert process.returncode != 0, stderr
        report = load_capability_report(report_path)
        assert report.passed is False
        assert "interrupt" in report.reason.lower()
        assert report_path.with_suffix(report_path.suffix + ".artifact.jsonl").is_file()
        assert json.loads(stdout)["passed"] is False
        with ExclusiveSerialPort(device, lock_dir=tmp_path):
            pass
    finally:
        os.close(master_fd)
        if process.poll() is None:
            process.kill()
            process.wait(timeout=2.0)
