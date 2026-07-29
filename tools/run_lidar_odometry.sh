#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
manifest_dir="$repo_root/ros2_ws/src/ed_uav_lidar/config/fields"
manifest_path="$manifest_dir/mid360_field_manifest.local.json"
state_dir="${XDG_STATE_HOME:-$repo_root/.omo/evidence}/lidar-odometry"
run_dir="$state_dir/$(date -u +%Y%m%dT%H%M%SZ)-$$"

declare -A owned_pid=()
declare -A owned_pgid=()
declare -a owned_labels=()
launcher_pgid="$(ps -o pgid= -p $$ | tr -d ' ')"

process_is_alive() {
  local pid="$1"
  local state
  kill -0 "$pid" 2>/dev/null || return 1
  state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d ' ')"
  [[ -n "$state" && "$state" != Z* ]]
}

cleanup() {
  local status=$1
  trap - EXIT INT TERM
  local label pgid pid
  for label in "${owned_labels[@]}"; do
    pgid="${owned_pgid[$label]:-}"
    [[ -n "$pgid" ]] && kill -TERM -- "-$pgid" 2>/dev/null || true
  done
  local deadline=$((SECONDS + 5))
  while ((SECONDS < deadline)); do
    local alive=0
    for label in "${owned_labels[@]}"; do
      pid="${owned_pid[$label]:-}"
      if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        alive=1
      fi
    done
    ((alive == 0)) && break
    sleep 0.1
  done
  for label in "${owned_labels[@]}"; do
    pgid="${owned_pgid[$label]:-}"
    [[ -n "$pgid" ]] && kill -KILL -- "-$pgid" 2>/dev/null || true
  done
  for label in "${owned_labels[@]}"; do
    pid="${owned_pid[$label]:-}"
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
  done
  exit "$status"
}
trap 'cleanup $?' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

choose_preset() {
  local choice=
  while true; do
    printf 'Select LiDAR preset [simulation/field-mid360]: ' >&2
    if ! IFS= read -r choice; then return 130; fi
    case "${choice,,}" in
      simulation|field-mid360) printf '%s
' "${choice,,}"; return 0 ;;
      *) printf 'Enter simulation or field-mid360.
' >&2 ;;
    esac
  done
}

validate_manifest() {
  python3 - "$manifest_path" <<'PY'
import ipaddress, json, re, sys
from pathlib import Path
manifest = Path(sys.argv[1])
required = ("serial_number", "lidar_ip", "host_ip", "firmware", "driver_json", "extrinsics", "fast_lio_launch")
placeholders = {"unset", "placeholder", "tbd", "todo", "example", "sample", "n/a", "na", "localhost"}
identity_pattern = re.compile(r'^[A-Za-z0-9._-]+$')
try:
    data = json.loads(manifest.read_text(encoding='utf-8'))
except Exception as exc:
    print(f'invalid_manifest:{exc}', file=sys.stderr)
    raise SystemExit(1)
if not isinstance(data, dict):
    print('invalid_manifest:not_object', file=sys.stderr)
    raise SystemExit(1)
for key in required:
    if key not in data:
        print(f'missing_key:{key}', file=sys.stderr)
        raise SystemExit(1)
    value = data[key]
    if not isinstance(value, str) or not value.strip():
        print(f'empty_{key}', file=sys.stderr)
        raise SystemExit(1)
    lowered = value.strip().lower()
    if lowered in placeholders or any(token in lowered for token in ('unset', 'placeholder', 'example', 'sample', 'todo', 'tbd')):
        print(f'placeholder_{key}', file=sys.stderr)
        raise SystemExit(1)
for key in ('lidar_ip', 'host_ip'):
    try:
        ipaddress.ip_address(data[key].strip())
    except Exception:
        print(f'invalid_{key}', file=sys.stderr)
        raise SystemExit(1)
if data['lidar_ip'].strip() == data['host_ip'].strip():
    print('same_host_and_sensor_ip', file=sys.stderr)
    raise SystemExit(1)
for key in ('serial_number', 'firmware'):
    value = data[key].strip()
    if not identity_pattern.fullmatch(value):
        print(f'invalid_{key}', file=sys.stderr)
        raise SystemExit(1)
for key in ('driver_json', 'extrinsics'):
    candidate = Path(data[key].strip())
    if not candidate.is_absolute():
        candidate = manifest.parent / candidate
    if not candidate.exists():
        print(f'missing_{key}', file=sys.stderr)
        raise SystemExit(1)
fast_lio_launch = Path(data['fast_lio_launch'].strip())
if not fast_lio_launch.is_absolute():
    fast_lio_launch = manifest.parent / fast_lio_launch
approved_fast_lio_launch = manifest.parent.resolve() / 'fast_lio.launch.py'
if fast_lio_launch.resolve(strict=False) != approved_fast_lio_launch:
    print('disallowed_fast_lio_launch', file=sys.stderr)
    raise SystemExit(1)
if not approved_fast_lio_launch.is_file():
    print('missing_fast_lio_launch', file=sys.stderr)
    raise SystemExit(1)
print('field_manifest=valid', file=sys.stderr)
PY
}

manifest_value() {
  python3 - "$manifest_path" "$1" <<'PY'
import json, sys
from pathlib import Path
manifest = Path(sys.argv[1])
key = sys.argv[2]
data = json.loads(manifest.read_text(encoding='utf-8'))
value = Path(data[key]) if key in {'driver_json', 'extrinsics', 'fast_lio_launch'} else data[key]
if key in {'driver_json', 'extrinsics', 'fast_lio_launch'} and not value.is_absolute():
    value = manifest.parent / value
print(value)
PY
}

shell_quote() {
  printf '%q' "$1"
}

command_for() {
  local label="$1" preset="$2" override="$3"
  if [[ -n "$override" ]]; then printf '%s' "$override"; return; fi
  case "$label:$preset" in
    lidar:simulation) printf 'ros2 launch ed_uav_gazebo lidar_odometry_simulation.launch.py' ;;
    fast_lio:simulation) printf 'ros2 launch ed_uav_gazebo fast_lio_simulation.launch.py' ;;
    localization:simulation) printf 'ros2 launch ed_uav_localization localization_simulation.launch.py' ;;
    monitor:simulation) printf 'ros2 run ed_uav_localization odometry_accuracy_demo --mode stationary --duration-sec 60 --min-samples 100' ;;
    lidar:field-mid360) printf 'if [[ -n "${FIELD_HOST_IP:-}" ]]; then export MID360_HOST_IP="$FIELD_HOST_IP"; fi; ros2 launch ed_uav_lidar lidar.launch.py serial_number:=%q sensor_ip:=%q firmware_version:=%q driver_config_path:=%q time_authority:=host' "$FIELD_SERIAL_NUMBER" "$FIELD_LIDAR_IP" "$FIELD_FIRMWARE" "$FIELD_DRIVER_JSON" ;;
    fast_lio:field-mid360) printf 'ros2 launch %q' "$FIELD_FAST_LIO_LAUNCH" ;;
    localization:field-mid360) printf '/bin/bash -lc "ros2 run ed_uav_localization lio_adapter --ros-args -p calibration_file:=%q & ros2 run ed_uav_localization source_supervisor; wait"' "$FIELD_EXTRINSICS_PATH" ;;
    monitor:field-mid360) printf '/bin/bash tools/run_lidar_odometry_accuracy_demo.sh' ;;
  esac
}

health_command_for() {
  local label="$1" preset="$2" override="$3"
  if [[ -n "$override" ]]; then printf '%s' "$override"; return; fi
  case "$label:$preset" in
    lidar:simulation) printf 'timeout 5s ros2 topic echo /livox/lidar --once >/dev/null' ;;
    lidar:field-mid360) printf 'timeout 5s ros2 topic echo /livox/lidar --once >/dev/null' ;;
    fast_lio:field-mid360) printf 'timeout 5s ros2 topic echo /fast_lio/odometry --once >/dev/null' ;;
    localization:field-mid360) printf 'timeout 5s ros2 topic echo /localization/odom --once >/dev/null' ;;
    monitor:field-mid360) printf 'timeout 5s ros2 topic echo /livox/lidar --once >/dev/null' ;;
  esac
}

spawn() {
  local label="$1" preset="$2" cmd_override="$3" health_override="$4" cmd log pid pgid
  cmd="$(command_for "$label" "$preset" "$cmd_override")"
  log="$run_dir/$label.log"
  : > "$log"
  if [[ "$label" == monitor ]]; then
    setsid /bin/bash -lc "$cmd" > >(tee -a "$log") 2> >(tee -a "$log" >&2) &
  else
    setsid /bin/bash -lc "$cmd" >>"$log" 2>&1 &
  fi
  pid=$!
  pgid="$(ps -o pgid= -p "$pid" | tr -d ' ')"
  if [[ "$pgid" == "$launcher_pgid" ]]; then
    kill -TERM -- "-$pgid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    printf 'launcher_and_child_share_pgid:%s
' "$pgid" >&2
    exit 1
  fi
  owned_pid[$label]="$pid"
  owned_pgid[$label]="$pgid"
  owned_labels+=("$label")
  printf '%s pid=%s pgid=%s cmd=%s
' "$label" "$pid" "$pgid" "$cmd"
  if [[ -n "$health_override" ]]; then
    if ! timeout 10s /bin/bash -lc "$health_override"; then
      printf 'HEALTH_FAILED:%s\n' "$label" >&2
      return 1
    fi
    if [[ "$label" != monitor ]]; then
      kill -0 "$pid" && sleep 0.05 && kill -0 "$pid"
    fi
  else
    if ! timeout 10s /bin/bash -lc "kill -0 $pid"; then
      printf 'HEALTH_FAILED:%s\n' "$label" >&2
      return 1
    fi
    kill -0 "$pid" && sleep 0.05 && kill -0 "$pid"
  fi
}

if (($# > 0)); then printf 'This command takes no arguments.
' >&2; exit 2; fi
selected_preset="$(choose_preset)"
if [[ "$selected_preset" == field-mid360 ]]; then
  validate_manifest
fi
mkdir -p "$state_dir" "$run_dir"
if [[ "$selected_preset" == field-mid360 ]]; then
  FIELD_SERIAL_NUMBER="$(manifest_value serial_number)"
  FIELD_LIDAR_IP="$(manifest_value lidar_ip)"
  FIELD_HOST_IP="$(manifest_value host_ip)"
  FIELD_FIRMWARE="$(manifest_value firmware)"
  FIELD_DRIVER_JSON="$(manifest_value driver_json)"
  FIELD_EXTRINSICS_PATH="$(manifest_value extrinsics)"
  FIELD_FAST_LIO_LAUNCH="$(manifest_value fast_lio_launch)"
fi
printf 'selected_preset=%s
' "$selected_preset"
printf 'evidence_dir=%s
' "$run_dir"
declare -a upstream_labels=()
case "$selected_preset" in
  simulation)
    upstream_labels=(lidar)
    spawn lidar "$selected_preset" "${ED_LIDAR_ODOMETRY_LIDAR_CMD:-}" "$(health_command_for lidar "$selected_preset" "${ED_LIDAR_ODOMETRY_LIDAR_HEALTH_CMD:-}")"
    ;;
  field-mid360)
    upstream_labels=(lidar fast_lio localization)
    spawn lidar "$selected_preset" "${ED_LIDAR_ODOMETRY_LIDAR_CMD:-}" "$(health_command_for lidar "$selected_preset" "${ED_LIDAR_ODOMETRY_LIDAR_HEALTH_CMD:-}")"
    spawn fast_lio "$selected_preset" "${ED_LIDAR_ODOMETRY_FAST_LIO_CMD:-}" "$(health_command_for fast_lio "$selected_preset" "${ED_LIDAR_ODOMETRY_FAST_LIO_HEALTH_CMD:-}")"
    spawn localization "$selected_preset" "${ED_LIDAR_ODOMETRY_LOCALIZATION_CMD:-}" "$(health_command_for localization "$selected_preset" "${ED_LIDAR_ODOMETRY_LOCALIZATION_HEALTH_CMD:-}")"
    ;;
esac
spawn monitor "$selected_preset" "${ED_LIDAR_ODOMETRY_MONITOR_CMD:-}" "$(health_command_for monitor "$selected_preset" "${ED_LIDAR_ODOMETRY_MONITOR_HEALTH_CMD:-}")"
monitor_pid="${owned_pid[monitor]}"
child_status=0
upstream_failed=""
while process_is_alive "$monitor_pid"; do
  for label in "${upstream_labels[@]}"; do
    if ! process_is_alive "${owned_pid[$label]}"; then
      upstream_failed="$label"
      child_status=1
      break 2
    fi
  done
  sleep 0.05
done
if [[ -n "$upstream_failed" ]]; then
  printf 'CHILD_DIED:%s
' "$upstream_failed" >&2
else
  if wait "$monitor_pid"; then
    child_status=0
  else
    child_status=$?
  fi
fi
for label in "${upstream_labels[@]}"; do
  pgid="${owned_pgid[$label]:-}"
  [[ -n "$pgid" ]] && kill -TERM -- "-$pgid" 2>/dev/null || true
  pid="${owned_pid[$label]:-}"
  [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
done
printf 'LIDAR_ODOMETRY_RESULT=%s
' "$child_status"
exit "$child_status"
