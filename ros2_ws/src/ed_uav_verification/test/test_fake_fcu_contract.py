from __future__ import annotations

import os
import selectors
import subprocess
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
PACKAGE_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_verification"
BRINGUP_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "ed_uav_bringup"
FCU_DRY_RUN_LAUNCH = BRINGUP_ROOT / "launch" / "fcu_dry_run.launch.py"


def _wait_for_ready(process: subprocess.Popen[str]) -> str:
    assert process.stdout is not None
    with selectors.DefaultSelector() as selector:
        _ = selector.register(process.stdout, selectors.EVENT_READ)
        assert selector.select(timeout=2.0), "fake FCU did not announce readiness within two seconds"
    return os.read(process.stdout.fileno(), 4096).decode("utf-8").strip()


def _read_frame(file_descriptor: int) -> bytes:
    with selectors.DefaultSelector() as selector:
        _ = selector.register(file_descriptor, selectors.EVENT_READ)
        assert selector.select(timeout=1.0), "fake FCU did not emit telemetry within one second"
    return os.read(file_descriptor, 512)


def _is_valid_position_frame(frame: bytes) -> bool:
    if len(frame) < 6 or frame[:3] != bytes((0xAA, 0xFF, 0x08)) or len(frame) != frame[3] + 6:
        return False
    sum_check = 0
    add_check = 0
    for value in frame[:-2]:
        sum_check = (sum_check + value) & 0xFF
        add_check = (add_check + sum_check) & 0xFF
    return frame[-2:] == bytes((sum_check, add_check))


def test_fake_fcu_creates_requested_pty_emits_fresh_telemetry_and_releases_it(tmp_path: Path) -> None:
    # Given: a caller-owned path where the fake must expose its PTY slave.
    requested_pty = tmp_path / "fcu"
    environment = os.environ | {"PYTHONPATH": str(PACKAGE_ROOT)}

    # When: the planned fake-FCU process starts at a bounded telemetry rate.
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "ed_uav_verification.fake_fcu",
            "--pty-device",
            str(requested_pty),
            "--seed",
            "31",
            "--rate-hz",
            "5",
        ],
        env=environment,
        stderr=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
    )
    try:
        ready_line = _wait_for_ready(process)
        stderr = process.stderr.read() if process.poll() is not None and process.stderr is not None else ""
        assert ready_line == f"FAKE FCU READY: {requested_pty}", stderr or ready_line
        assert requested_pty.is_symlink(), "fake FCU did not create the requested PTY link"

        file_descriptor = os.open(requested_pty, os.O_RDONLY | os.O_NONBLOCK)
        try:
            first = _read_frame(file_descriptor)
            second = _read_frame(file_descriptor)
        finally:
            os.close(file_descriptor)

        assert _is_valid_position_frame(first), "first fake FCU sample is not a valid V7 position frame"
        assert _is_valid_position_frame(second), "second fake FCU sample is not a valid V7 position frame"
        assert first != second, "fake FCU telemetry is stale across consecutive samples"
    finally:
        if process.poll() is None:
            process.terminate()
        _ = process.wait(timeout=2.0)

    # Then: process shutdown releases the caller-visible PTY path.
    assert not os.path.lexists(requested_pty), "fake FCU left the requested PTY path behind"


def test_fcu_dry_run_launch_starts_fake_before_bridge() -> None:
    # Given: the FCU dry-run launch source.
    source = FCU_DRY_RUN_LAUNCH.read_text(encoding="utf-8")

    # When: process declarations and readiness sequencing are inspected.
    fake_position = source.find('executable="ed-uav-fake-fcu"')
    bridge_position = source.find('executable="ed_uav_fcu_bridge"')

    # Then: bridge startup is gated on the fake process becoming ready.
    assert fake_position >= 0, "fcu_dry_run does not start the planned fake FCU executable"
    assert bridge_position > fake_position, "fcu_dry_run does not declare the bridge after the fake FCU"
    assert "RegisterEventHandler" in source and "OnProcessIO" in source, (
        "fcu_dry_run does not gate bridge startup on the fake FCU readiness message"
    )
    assert "target_action=fake_fcu" in source, "fcu_dry_run does not observe the fake FCU process"
    assert "event.text.startswith(READY_PREFIX)" in source, (
        "fcu_dry_run does not require the fake FCU readiness prefix"
    )
    assert "on_stdout=lambda event: [bridge]" in source, (
        "fcu_dry_run does not start the bridge from the readiness callback"
    )
