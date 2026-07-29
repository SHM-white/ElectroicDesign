from __future__ import annotations

import json
import os
import pty
import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_fcu_bridge.capability import load_capability_report
from ed_uav_fcu_bridge.serial_port import ExclusiveSerialPort
from ed_uav_fcu_bridge.v7_codec import build_frame, decode_frame
from test_pty_surface import cli_environment, read_frame


def test_prop_off_characterizer_uses_only_non_arming_commands_and_prints_report(tmp_path: Path) -> None:
    # Given: a real PTY peer and fake-evidence characterization invocation.
    master_fd, slave_fd = pty.openpty()
    device = os.ttyname(slave_fd)
    os.close(slave_fd)
    report_path = tmp_path / "capability.json"
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "ed_uav_fcu_bridge.characterize",
            "--device", device,
            "--lock-dir", str(tmp_path),
            "--device-identity", "fake-pty-fcu-001",
            "--report", str(report_path),
            "--timeout-s", "0.2",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=cli_environment(),
    )
    commands: list[bytes] = []
    try:
        # When: the fake FCU acknowledges every characterization probe.
        for _ in range(6):
            raw = read_frame(master_fd, timeout_s=1.0)
            commands.append(raw)
            sent = decode_frame(raw)
            ack = build_frame(0xFF, 0x00, bytes((sent.frame_id, sent.sum_check, sent.add_check)))
            os.write(master_fd, ack)
        stdout, stderr = process.communicate(timeout=2.0)

        # Then: the structured fake report is complete but cannot authorize field use.
        assert process.returncode == 2, stderr
        printed = json.loads(stdout)
        report = load_capability_report(report_path)
        assert printed["schema"] == report.schema
        assert printed["passed"] is False
        assert printed["evidence_kind"] == "fake_pty"
        assert printed["behavior_results"]["cancellation"] is False
        assert printed["behavior_results"]["zero_motion"] is False
        assert [raw[4:7] for raw in commands] == [
            bytes((0x10, 0x01, 0x01)),
            bytes((0x10, 0x01, 0x02)),
            bytes((0x10, 0x02, 0x01)),
            bytes((0x10, 0x02, 0x02)),
            bytes((0x10, 0x00, 0x04)),
            bytes((0x10, 0x00, 0x06)),
        ]
        assert all(raw[2] == 0xE0 for raw in commands)
        with ExclusiveSerialPort(device, lock_dir=tmp_path):
            pass
    finally:
        os.close(master_fd)
        if process.poll() is None:
            process.kill()
            process.wait(timeout=2.0)


def test_characterizer_timeout_writes_explicit_red_report(tmp_path: Path) -> None:
    # Given: a PTY peer that withholds every ACK.
    master_fd, slave_fd = pty.openpty()
    device = os.ttyname(slave_fd)
    os.close(slave_fd)
    report_path = tmp_path / "timeout.json"
    process = subprocess.run(
        [
            sys.executable,
            "-m",
            "ed_uav_fcu_bridge.characterize",
            "--device", device,
            "--lock-dir", str(tmp_path),
            "--device-identity", "silent-fcu-001",
            "--report", str(report_path),
            "--timeout-s", "0.02",
        ],
        capture_output=True,
        text=True,
        env=cli_environment(),
        timeout=2.0,
        check=False,
    )
    try:
        # When / Then: timeout is retained as a structured red result.
        assert process.returncode == 2, process.stderr
        report = load_capability_report(report_path)
        assert report.passed is False
        assert "target_position" in report.reason
    finally:
        os.close(master_fd)
