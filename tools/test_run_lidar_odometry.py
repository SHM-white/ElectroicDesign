from __future__ import annotations

import json
import os
import shlex
import signal
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import pytest

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = REPOSITORY_ROOT / "tools" / "run_lidar_odometry.sh"


@dataclass(frozen=True, slots=True)
class Workspace:
    root: Path
    launcher: Path
    child_log: Path
    event_log: Path
    ready: Path
    state: Path
    helper: Path
    fake_ros2: Path


def create_workspace(tmp_path: Path) -> Workspace:
    root = tmp_path / "workspace"
    fields = root / "ros2_ws/src/ed_uav_lidar/config/fields"
    tools = root / "tools"
    bin_dir = root / "bin"
    fields.mkdir(parents=True)
    tools.mkdir(parents=True)
    bin_dir.mkdir(parents=True)
    launcher = tools / LAUNCHER.name
    shutil.copy2(LAUNCHER, launcher)
    launcher.chmod(0o755)
    fake_ros2 = bin_dir / "ros2"
    fake_ros2.write_text(
        """#!/usr/bin/env python3
import json, os, sys, time
path = os.environ['FAKE_ROS2_EVENT_LOG']
argv = sys.argv[1:]
topic = next((value for value in argv if value.startswith('/')), '')
required_starts = 1
if os.environ.get('FAKE_FIELD_MODE') == '1':
    required_starts = {'/livox/lidar': 1, '/fast_lio/odometry': 2, '/localization/odom': 3}.get(topic, 1)
deadline = time.monotonic() + 5.0
while True:
    events = []
    if os.path.exists(path):
        with open(path, encoding='utf-8') as stream:
            events = [json.loads(line) for line in stream if line.strip()]
    if sum(event.get('event') == 'start' for event in events) >= required_starts:
        break
    if time.monotonic() >= deadline:
        raise SystemExit(92)
    time.sleep(0.02)
with open(path, 'a', encoding='utf-8') as stream:
    stream.write(json.dumps({'event': 'ros2', 'argv': argv, 'stamp': time.monotonic()}) + '\\n')
""",
        encoding="utf-8",
    )
    fake_ros2.chmod(0o755)
    helper = root / "child.py"
    helper.write_text(
        """import json, os, sys, time
label, mode, log_path, ready_path = sys.argv[1:]
with open(log_path, 'a', encoding='utf-8') as stream:
    stream.write(json.dumps({'label': label, 'pid': os.getpid(), 'pgid': os.getpgid(0)}) + '\\n')
event_path = os.environ.get('FAKE_EVENT_LOG')
if event_path:
    with open(event_path, 'a', encoding='utf-8') as stream:
        stream.write(json.dumps({'event': 'start', 'label': label, 'stamp': time.monotonic()}) + '\\n')
if mode == 'monitor':
    deadline = time.monotonic() + 5.0
    while not os.path.exists(ready_path):
        if time.monotonic() >= deadline:
            raise SystemExit(90)
        time.sleep(0.02)
    print('ODOMETRY_ACCURACY_LIVE dx_m=0.000000 dy_m=0.000000 dz_m=0.000000 xy_m=0.000000 three_d_m=0.000000 frame=odom age_sec=0.000000 health=live', flush=True)
    print('ODOMETRY_ACCURACY_LIVE dx_m=1.000000 dy_m=2.000000 dz_m=2.000000 xy_m=2.236068 three_d_m=3.000000 frame=odom age_sec=0.000000 health=live', flush=True)
    print('ODOMETRY_ACCURACY_RESULT={"schema_version":1,"status":"passed","trial":"stationary","interpretation":"relative","input_topic":"/localization/odom","frame_id":"odom","start_stamp_ns":1,"end_stamp_ns":2,"duration_sec":1e-9,"sample_count":2,"rejected_count":0,"metrics":null}', flush=True)
    raise SystemExit(int(os.environ.get('FAKE_MONITOR_EXIT', '0')))
while True:
    time.sleep(1.0)
""",
        encoding="utf-8",
    )
    return Workspace(
        root=root,
        launcher=launcher,
        child_log=root / "children.jsonl",
        event_log=root / "events.jsonl",
        ready=root / "monitor.ready",
        state=root / "state",
        helper=helper,
        fake_ros2=fake_ros2,
    )


def write_manifest(workspace: Workspace, **overrides: str) -> None:
    fields = workspace.root / "ros2_ws/src/ed_uav_lidar/config/fields"
    for name in ("driver.json", "extrinsics.yaml", "fast_lio.launch.py"):
        (fields / name).write_text("{}\n", encoding="utf-8")
    manifest = {
        "serial_number": "MID360-123",
        "lidar_ip": "192.168.10.10",
        "host_ip": "192.168.10.2",
        "firmware": "v1.0.0",
        "driver_json": "driver.json",
        "extrinsics": "extrinsics.yaml",
        "fast_lio_launch": "fast_lio.launch.py",
    }
    manifest.update(overrides)
    (fields / "mid360_field_manifest.local.json").write_text(
        json.dumps(manifest) + "\n", encoding="utf-8"
    )


def command(workspace: Workspace, label: str, mode: str = "hold") -> str:
    values = (workspace.helper, label, mode, workspace.child_log, workspace.ready)
    return "exec python3 " + " ".join(shlex.quote(str(value)) for value in values)


def environment(workspace: Workspace, *, monitor_mode: str = "monitor") -> dict[str, str]:
    env = {
        "PATH": str(workspace.fake_ros2.parent) + os.pathsep + os.environ["PATH"],
        "XDG_STATE_HOME": str(workspace.state),
        "ED_LIDAR_ODOMETRY_LIDAR_CMD": command(workspace, "lidar"),
        "ED_LIDAR_ODOMETRY_FAST_LIO_CMD": command(workspace, "fast_lio"),
        "ED_LIDAR_ODOMETRY_LOCALIZATION_CMD": command(workspace, "localization"),
        "ED_LIDAR_ODOMETRY_MONITOR_CMD": command(workspace, "monitor", monitor_mode),
        "ED_LIDAR_ODOMETRY_MONITOR_HEALTH_CMD": f"touch {shlex.quote(str(workspace.ready))}",
        "FAKE_ROS2_EVENT_LOG": str(workspace.event_log),
        "FAKE_EVENT_LOG": str(workspace.event_log),
    }
    return env


def run_launcher(
    workspace: Workspace,
    stdin: str,
    *,
    env: dict[str, str] | None = None,
    args: tuple[str, ...] = (),
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["/bin/bash", str(workspace.launcher), *args],
        cwd=workspace.root,
        env=environment(workspace) if env is None else env,
        input=stdin,
        text=True,
        capture_output=True,
        timeout=15,
        check=False,
    )


def wait_for_children(workspace: Workspace, count: int = 4) -> list[dict[str, int | str]]:
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if workspace.child_log.exists():
            rows = [json.loads(line) for line in workspace.child_log.read_text(encoding="utf-8").splitlines()]
            if len(rows) >= count:
                return rows
        time.sleep(0.02)
    raise AssertionError("children did not start")


def assert_dead(pids: list[int]) -> None:
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        alive = []
        for pid in pids:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                continue
            alive.append(pid)
        if not alive:
            return
        time.sleep(0.02)
    raise AssertionError(f"lingering child pids: {alive}")


def test_rejects_arguments_without_spawning(tmp_path: Path) -> None:
    workspace = create_workspace(tmp_path)
    result = run_launcher(workspace, "", args=("simulation",))
    assert result.returncode == 2
    assert not workspace.child_log.exists()


def test_interactive_simulation_reprompts_streams_and_cleans(tmp_path: Path) -> None:
    workspace = create_workspace(tmp_path)
    result = run_launcher(workspace, "bad\nsimulation\n")
    assert result.returncode == 0
    assert "Enter simulation or field-mid360." in result.stderr
    assert "selected_preset=simulation" in result.stdout
    assert result.stdout.count("ODOMETRY_ACCURACY_LIVE ") == 2
    assert result.stdout.count("ODOMETRY_ACCURACY_RESULT=") == 1
    rows = wait_for_children(workspace, count=2)
    assert [row["label"] for row in rows] == ["lidar", "monitor"]
    assert len({row["pgid"] for row in rows}) == 2
    assert_dead([int(row["pid"]) for row in rows])


def test_simulation_starts_integrated_bundle_once_and_keeps_monitor_separate(tmp_path: Path) -> None:
    workspace = create_workspace(tmp_path)
    env = environment(workspace)
    env["ED_LIDAR_ODOMETRY_LIDAR_CMD"] = command(workspace, "integrated_simulation_bundle")
    env["ED_LIDAR_ODOMETRY_FAST_LIO_CMD"] = command(workspace, "duplicate_fast_lio")
    env["ED_LIDAR_ODOMETRY_LOCALIZATION_CMD"] = command(workspace, "duplicate_localization")
    result = run_launcher(workspace, "simulation\n", env=env)
    assert result.returncode == 0, result.stderr + result.stdout
    rows = wait_for_children(workspace, count=2)
    assert [row["label"] for row in rows] == ["integrated_simulation_bundle", "monitor"]
    assert "duplicate_fast_lio" not in {row["label"] for row in rows}
    assert "duplicate_localization" not in {row["label"] for row in rows}
    assert result.stdout.count("ODOMETRY_ACCURACY_LIVE ") == 2
    assert result.stdout.count("ODOMETRY_ACCURACY_RESULT=") == 1
    assert_dead([int(row["pid"]) for row in rows])


def test_field_starts_lidar_before_fast_lio_before_localization_and_gates_monitor_on_health(tmp_path: Path) -> None:
    workspace = create_workspace(tmp_path)
    write_manifest(workspace)
    env = environment(workspace, monitor_mode="hold")
    env["ED_LIDAR_ODOMETRY_LIDAR_CMD"] = command(workspace, "field_lidar")
    env["ED_LIDAR_ODOMETRY_FAST_LIO_CMD"] = command(workspace, "field_fast_lio")
    env["ED_LIDAR_ODOMETRY_LOCALIZATION_CMD"] = command(workspace, "field_localization")
    env["ED_LIDAR_ODOMETRY_MONITOR_CMD"] = command(workspace, "field_monitor", "monitor")
    env["FAKE_ROS2_EVENT_LOG"] = str(workspace.event_log)
    env["FAKE_EVENT_LOG"] = str(workspace.event_log)
    env["FAKE_MONITOR_READY"] = str(workspace.ready)
    env["FAKE_FIELD_MODE"] = "1"
    env["ED_LIDAR_ODOMETRY_MONITOR_HEALTH_CMD"] = f"touch {shlex.quote(str(workspace.ready))}"
    for key in (
        "ED_LIDAR_ODOMETRY_LIDAR_HEALTH_CMD",
        "ED_LIDAR_ODOMETRY_FAST_LIO_HEALTH_CMD",
        "ED_LIDAR_ODOMETRY_LOCALIZATION_HEALTH_CMD",
    ):
        env.pop(key, None)
    result = run_launcher(workspace, "field-mid360\n", env=env)
    assert result.returncode == 0, result.stderr + result.stdout
    rows = wait_for_children(workspace, count=4)
    assert [row["label"] for row in rows] == ["field_lidar", "field_fast_lio", "field_localization", "field_monitor"]
    assert result.stdout.count("ODOMETRY_ACCURACY_LIVE ") == 2
    assert result.stdout.count("ODOMETRY_ACCURACY_RESULT=") == 1
    events = [json.loads(line) for line in workspace.event_log.read_text(encoding='utf-8').splitlines()]
    def first_index(predicate):
        for index, event in enumerate(events):
            if predicate(event):
                return index
        raise AssertionError('matching event not found')

    start_field_lidar = first_index(lambda event: event.get('event') == 'start' and event.get('label') == 'field_lidar')
    health_livox = first_index(lambda event: event.get('event') == 'ros2' and '/livox/lidar' in event.get('argv', []))
    start_field_fast_lio = first_index(lambda event: event.get('event') == 'start' and event.get('label') == 'field_fast_lio')
    health_fast_lio = first_index(lambda event: event.get('event') == 'ros2' and '/fast_lio/odometry' in event.get('argv', []))
    start_field_localization = first_index(lambda event: event.get('event') == 'start' and event.get('label') == 'field_localization')
    health_localization = first_index(lambda event: event.get('event') == 'ros2' and '/localization/odom' in event.get('argv', []))
    start_field_monitor = first_index(lambda event: event.get('event') == 'start' and event.get('label') == 'field_monitor')
    assert start_field_lidar < health_livox < start_field_fast_lio < health_fast_lio < start_field_localization < health_localization < start_field_monitor
    assert health_localization > start_field_localization
    assert_dead([int(row["pid"]) for row in rows])


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        ({"serial_number": "UNSET"}, "placeholder_serial_number"),
        ({"serial_number": "MID360/123"}, "invalid_serial_number"),
        ({"lidar_ip": "not-an-ip"}, "invalid_lidar_ip"),
        ({"host_ip": "192.168.10.10"}, "same_host_and_sensor_ip"),
        ({"driver_json": "missing.json"}, "missing_driver_json"),
    ],
)
def test_invalid_field_manifest_fails_before_spawn(
    tmp_path: Path, overrides: dict[str, str], reason: str
) -> None:
    workspace = create_workspace(tmp_path)
    write_manifest(workspace, **overrides)
    result = run_launcher(workspace, "field-mid360\n")
    assert result.returncode != 0
    assert reason in result.stderr
    assert not workspace.child_log.exists()
    assert not workspace.state.exists()


def test_unapproved_fast_lio_launch_fails_before_spawn(tmp_path: Path) -> None:
    workspace = create_workspace(tmp_path)
    fields = workspace.root / "ros2_ws/src/ed_uav_lidar/config/fields"
    (fields / "rogue.launch.py").write_text("{}\n", encoding="utf-8")
    write_manifest(workspace, fast_lio_launch="rogue.launch.py")

    result = run_launcher(workspace, "field-mid360\n")

    assert result.returncode != 0
    assert "disallowed_fast_lio_launch" in result.stderr
    assert not workspace.child_log.exists()
    assert not workspace.state.exists()


def test_monitor_nonzero_preserves_json_and_exit(tmp_path: Path) -> None:
    workspace = create_workspace(tmp_path)
    env = environment(workspace)
    env["FAKE_MONITOR_EXIT"] = "7"
    result = run_launcher(workspace, "simulation\n", env=env)
    assert result.returncode == 7
    assert result.stdout.count("ODOMETRY_ACCURACY_RESULT=") == 1
    rows = wait_for_children(workspace, count=2)
    assert_dead([int(row["pid"]) for row in rows])


def test_health_failure_reports_label_and_cleans(tmp_path: Path) -> None:
    workspace = create_workspace(tmp_path)
    env = environment(workspace)
    env["ED_LIDAR_ODOMETRY_LIDAR_HEALTH_CMD"] = "false"
    result = run_launcher(workspace, "simulation\n", env=env)
    assert result.returncode != 0
    assert "HEALTH_FAILED:lidar" in result.stderr
    assert not workspace.child_log.exists() or not workspace.child_log.read_text(encoding="utf-8").strip()


def test_upstream_child_death_cleans_every_owned_process(tmp_path: Path) -> None:
    workspace = create_workspace(tmp_path)
    env = environment(workspace, monitor_mode="hold")
    proc = subprocess.Popen(
        ["/bin/bash", str(workspace.launcher)],
        cwd=workspace.root,
        env=env,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    assert proc.stdin is not None
    proc.stdin.write("simulation\n")
    proc.stdin.close()
    proc.stdin = None
    rows = wait_for_children(workspace, count=2)
    lidar_pid = int(next(row["pid"] for row in rows if row["label"] == "lidar"))
    os.kill(lidar_pid, signal.SIGTERM)
    stdout, stderr = proc.communicate(timeout=10)
    assert proc.returncode != 0, stdout + stderr
    assert "CHILD_DIED:lidar" in stderr
    assert_dead([int(row["pid"]) for row in rows])


def test_sigint_cleans_every_owned_process(tmp_path: Path) -> None:
    workspace = create_workspace(tmp_path)
    env = environment(workspace, monitor_mode="hold")
    proc = subprocess.Popen(
        ["/bin/bash", str(workspace.launcher)],
        cwd=workspace.root,
        env=env,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    assert proc.stdin is not None
    proc.stdin.write("simulation\n")
    proc.stdin.close()
    proc.stdin = None
    rows = wait_for_children(workspace, count=2)
    os.kill(proc.pid, signal.SIGINT)
    stdout, stderr = proc.communicate(timeout=10)
    assert proc.returncode == 130, stdout + stderr
    assert_dead([int(row["pid"]) for row in rows])
