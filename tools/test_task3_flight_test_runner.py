from __future__ import annotations

import hashlib
import json
import os
import shlex
import shutil
import signal
import stat
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import pytest
from typing_extensions import assert_never


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = REPOSITORY_ROOT / "tools/run_task3_flight_test.sh"
InvalidInput = Literal["missing", "uncalibrated", "synthetic", "placeholder"]


@dataclass(frozen=True, slots=True)
class RuntimeFixture:
    root: Path
    runner: Path
    mission: Path
    field_profile: Path
    calibration: Path
    camera_plan: Path
    fcu_serial: Path
    hmac_key: Path
    keystore: Path
    mid360_driver: Path
    fast_lio_launch: Path
    unexpected_runtime: Path


def _runner_path() -> Path:
    assert RUNNER_PATH.is_file(), "missing Task3 flight-test runner: tools/run_task3_flight_test.sh"
    assert RUNNER_PATH.stat().st_mode & stat.S_IXUSR
    return RUNNER_PATH


def _write_executable(path: Path, source: str) -> None:
    path.write_text(source, encoding="utf-8")
    path.chmod(0o755)


def _write_json(path: Path, document: dict[str, object]) -> None:
    path.write_text(json.dumps(document), encoding="utf-8")


def _calibration(status: str) -> dict[str, object]:
    document: dict[str, object] = {
        "schema_version": 1,
        "calibration_id": "field-capture-001",
        "calibration_status": status,
        "calibration_hash": "",
        "sensor_serials": {
            "camera_narrow": "NARROW-001",
            "camera_wide": "WIDE-001",
            "lidar": "MID360-001",
        },
        "transforms": {
            name: {"xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]}
            for name in (
                "fcu_link",
                "lidar_link",
                "camera_narrow_optical_frame",
                "camera_wide_optical_frame",
                "rangefinder_link",
            )
        },
    }
    unsigned = dict(document)
    unsigned.pop("calibration_hash")
    document["calibration_hash"] = hashlib.sha256(
        json.dumps(unsigned, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    return document


def _field_profile(classification: str, activation: str) -> dict[str, object]:
    point = {"x_m": 0.0, "y_m": 0.0}
    return {
        "version": 1,
        "profile_type": "field",
        "profile_id": "task3-field-fixture",
        "units": {"length": "m", "angle": "rad"},
        "frame": {"id": "map", "convention": "ENU"},
        "provenance": {"classification": classification, "activation": activation},
        "takeoff": {"origin": point, "commanded_heading_rad": 0.0},
        "colors": [{"id": "green", "label": "green"}],
        "boundary_segments": [
            {"id": "edge-a", "start": point, "end": {"x_m": 1.0, "y_m": 0.0}, "color_id": "green"},
            {"id": "edge-b", "start": point, "end": {"x_m": 0.0, "y_m": 1.0}, "color_id": "green"},
        ],
        "allowed_zone": {"id": "allowed", "vertices": [point, {"x_m": 2.0, "y_m": 0.0}, {"x_m": 0.0, "y_m": 2.0}]},
        "no_fly_zones": [],
        "altitude": {"minimum_m": 0.0, "takeoff_m": 1.0, "maximum_m": 2.0},
        "landmarks": [],
    }


def _camera_plan(controller_id: str) -> dict[str, object]:
    return {
        "controller_budget_mbit_s": 384.0,
        "cameras": [
            {
                "role": role,
                "serial": serial,
                "observed_serial": serial,
                "by_id": f"/dev/v4l/by-id/{role}-camera",
                "controller_id": controller_id,
                "profile": f"{role}_live",
                "frame_id": f"camera_{role}_optical_frame",
                "mode": {"fourcc": "MJPG", "width": 1280, "height": 720, "frames_per_second": 20, "compression": "mjpeg", "declared_peak_mbit_s": 48.0},
                "calibration": {"serial": serial, "width": 1280, "height": 720, "captured_at_ns": 9_999_999_999_999_999_999, "valid_for_ns": 1, "camera_info_url": "file:///tmp/camera_info.yaml", "capture_provenance": "direct_v4l2", "observed_serial": serial, "observed_by_id": f"/dev/v4l/by-id/{role}-camera"},
            }
            for role, serial in (("narrow", "NARROW-001"), ("wide", "WIDE-001"))
        ],
    }


def _fixture(tmp_path: Path) -> RuntimeFixture:
    source_runner = _runner_path()
    root = tmp_path / "task3-runtime"
    tools = root / "tools"
    runtime = root / "runtime"
    fake_bin = root / "fake-bin"
    tools.mkdir(parents=True)
    runtime.mkdir()
    fake_bin.mkdir()
    runner = tools / source_runner.name
    shutil.copy2(source_runner, runner)
    unexpected_runtime = root / "unexpected-runtime"
    _write_executable(tools / "run_humble.sh", "#!/usr/bin/env bash\nset -euo pipefail\nprintf '%s\\n' humble >\"${TASK3_UNEXPECTED_RUNTIME:?}\"\nexec \"$@\"\n")
    _write_executable(fake_bin / "ros2", "#!/usr/bin/env bash\nset -euo pipefail\nif [[ -z \"${TASK3_FAKE_LAUNCH_READY:-}\" ]]; then\n  printf '%s\\n' ros2 >\"${TASK3_UNEXPECTED_RUNTIME:?}\"\n  exit 90\nfi\nsleep 30 &\nchild=$!\nprintf '%s\\n' \"$child\" >\"${TASK3_FAKE_CHILD_PID:?}\"\ntrap 'kill -TERM \"$child\" 2>/dev/null || true; wait \"$child\" 2>/dev/null || true; exit 130' INT TERM\nprintf '%s\\n' ready >\"${TASK3_FAKE_LAUNCH_READY:?}\"\nwait \"$child\"\n")
    for command in ("nc", "socat"):
        _write_executable(fake_bin / command, "#!/usr/bin/env bash\nprintf '%s\\n' network >\"${TASK3_UNEXPECTED_RUNTIME:?}\"\nexit 91\n")
    mission = runtime / "task3-mission.yaml"
    mission.write_text("task3_identity: task3\n", encoding="utf-8")
    field_profile = runtime / "field-profile.yaml"
    _write_json(field_profile, _field_profile("current_field", "eligible"))
    calibration = runtime / "calibration.yaml"
    _write_json(calibration, _calibration("CALIBRATED"))
    camera_plan = runtime / "camera-plan.json"
    _write_json(camera_plan, _camera_plan("controller-field-a"))
    fcu_serial = runtime / "fcu-serial"
    fcu_serial.touch()
    hmac_key = runtime / "hmac.key.hex"
    hmac_key.write_text("01" * 32, encoding="ascii")
    keystore = runtime / "sros-keystore"
    keystore.mkdir()
    mid360_driver = runtime / "mid360-driver.json"
    _write_json(mid360_driver, {"driver": "field"})
    fast_lio_launch = runtime / "fast_lio.launch.py"
    fast_lio_launch.write_text("def generate_launch_description():\n    return None\n", encoding="utf-8")
    return RuntimeFixture(root, runner, mission, field_profile, calibration, camera_plan, fcu_serial, hmac_key, keystore, mid360_driver, fast_lio_launch, unexpected_runtime)


def _arguments(fixture: RuntimeFixture, dry_run: bool) -> tuple[str, ...]:
    dry_run_flag = ("--dry-run",) if dry_run else ()
    return dry_run_flag + ("--mission-config", str(fixture.mission), "--field-profile", str(fixture.field_profile), "--calibration", str(fixture.calibration), "--camera-runtime-plan", str(fixture.camera_plan), "--fcu-serial", str(fixture.fcu_serial), "--hmac-key-file", str(fixture.hmac_key), "--mid360-driver-config", str(fixture.mid360_driver), "--fast-lio-launch", str(fixture.fast_lio_launch), "--task3-identity", "task3")


def _run(fixture: RuntimeFixture, arguments: tuple[str, ...], extra_environment: dict[str, str]) -> subprocess.CompletedProcess[str]:
    environment = {**os.environ, "PATH": f"{fixture.root / 'fake-bin'}:{os.environ['PATH']}", "TASK3_UNEXPECTED_RUNTIME": str(fixture.unexpected_runtime), "ROS_SECURITY_ENABLE": "true", "ROS_SECURITY_STRATEGY": "Enforce", "ROS_SECURITY_KEYSTORE": str(fixture.keystore), **extra_environment}
    return subprocess.run([str(fixture.runner), *arguments], cwd=fixture.root, env=environment, check=False, capture_output=True, text=True, timeout=5)


def test_runner_dry_run_requires_explicit_runtime_inputs(tmp_path: Path) -> None:
    # Given: an isolated runner with no ROS or network executables.
    fixture = _fixture(tmp_path)

    # When: dry-run omits every required Task3 runtime input.
    result = _run(fixture, ("--dry-run",), {})

    # Then: argument validation stops with the usage exit code before side effects.
    assert result.returncode == 64
    assert not fixture.unexpected_runtime.exists()


def test_runner_dry_run_prints_one_resolved_launch_without_side_effects(tmp_path: Path) -> None:
    # Given: measured temporary runtime inputs and enforced temporary SROS settings.
    fixture = _fixture(tmp_path)

    # When: the shell entry point resolves its Task3 launch in dry-run mode.
    result = _run(fixture, _arguments(fixture, dry_run=True), {})

    # Then: it emits exactly one fully-resolved command without starting Humble, ROS, serial, or UDP.
    assert result.returncode == 0
    commands = [line for line in result.stdout.splitlines() if line.startswith("ros2 launch ")]
    assert len(commands) == 1
    command = shlex.split(commands[0])
    assert command[:4] == ["ros2", "launch", "ed_uav_bringup", "task3_flight_test.launch.py"]
    assert {
        f"mission_config_path:={fixture.mission}", f"field_profile_path:={fixture.field_profile}", f"calibration_file:={fixture.calibration}", f"camera_runtime_plan:={fixture.camera_plan}", f"fcu_serial_port:={fixture.fcu_serial}", f"hmac_key_file:={fixture.hmac_key}", f"mid360_driver_config_path:={fixture.mid360_driver}", f"fast_lio_launch_path:={fixture.fast_lio_launch}", "task3_identity:=task3", "ros_security_enable:=true", "ros_security_strategy:=Enforce", f"ros_security_keystore:={fixture.keystore}", "enable_flight_commands:=true", "enable_realtime_control:=true", "enable_programmable_commands:=false",
    } <= set(command[4:])
    assert not fixture.unexpected_runtime.exists()


@pytest.mark.parametrize("invalid_input", ("missing", "uncalibrated", "synthetic", "placeholder"))
def test_runner_dry_run_rejects_invalid_real_flight_inputs(tmp_path: Path, invalid_input: InvalidInput) -> None:
    # Given: otherwise complete temporary Task3 runtime inputs.
    fixture = _fixture(tmp_path)
    arguments = list(_arguments(fixture, dry_run=True))
    match invalid_input:
        case "missing":
            index = arguments.index("--mission-config")
            del arguments[index : index + 2]
        case "uncalibrated":
            _write_json(fixture.calibration, _calibration("UNCALIBRATED"))
        case "synthetic":
            _write_json(fixture.field_profile, _field_profile("synthetic_simulation", "blocked"))
        case "placeholder":
            _write_json(fixture.camera_plan, _camera_plan("PLACEHOLDER"))
        case unexpected:
            assert_never(unexpected)

    # When: dry-run validates the selected input boundary.
    result = _run(fixture, tuple(arguments), {})

    # Then: invalid, synthetic, and placeholder evidence fails before an external command starts.
    assert result.returncode == 64
    assert not fixture.unexpected_runtime.exists()


def test_runner_interrupt_reaps_the_injected_harmless_launch_process(tmp_path: Path) -> None:
    # Given: a fake launch command that records a child process after startup.
    fixture = _fixture(tmp_path)
    ready = fixture.root / "launch-ready"
    child_pid = fixture.root / "launch-child.pid"
    environment = {"TASK3_FAKE_LAUNCH_READY": str(ready), "TASK3_FAKE_CHILD_PID": str(child_pid)}
    full_environment = {**os.environ, "PATH": f"{fixture.root / 'fake-bin'}:{os.environ['PATH']}", "TASK3_UNEXPECTED_RUNTIME": str(fixture.unexpected_runtime), "ROS_SECURITY_ENABLE": "true", "ROS_SECURITY_STRATEGY": "Enforce", "ROS_SECURITY_KEYSTORE": str(fixture.keystore), **environment}
    process = subprocess.Popen([str(fixture.runner), *_arguments(fixture, dry_run=False)], cwd=fixture.root, env=full_environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, start_new_session=True)
    try:
        for _ in range(100):
            if ready.is_file() and child_pid.is_file():
                break
            time.sleep(0.02)
        assert ready.is_file() and child_pid.is_file()

        # When: the operator interrupts the runner process group.
        os.killpg(process.pid, signal.SIGINT)
        process.communicate(timeout=5)

        # Then: the launch surface returns the interruption exit code with no surviving child.
        assert process.returncode == 130
        with pytest.raises(ProcessLookupError):
            os.kill(int(child_pid.read_text(encoding="utf-8")), 0)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
