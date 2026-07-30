#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

die() {
    printf 'run_landing_marker_recognition: %s\n' "$*" >&2
    exit 64
}

usage() {
    cat <<'EOF'
Usage: ./tools/run_landing_marker_recognition.sh --camera-plan PATH

Launches physical dual-camera landing-marker recognition with RViz through
the ROS 2 Humble runner. PATH is required, must be inside this repository,
and must contain measured narrow/wide bindings with real P25 controller IDs.
The two stable /dev/v4l/by-id devices must be attached and readable.
EOF
}

camera_plan_arg=""
while (($# > 0)); do
    case "$1" in
        --camera-plan)
            (($# >= 2)) || die "--camera-plan requires a path"
            [[ -z "$camera_plan_arg" ]] || die "--camera-plan may be provided only once"
            camera_plan_arg="$2"
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            die "unknown argument '$1'; use --help"
            ;;
    esac
done

[[ -n "$camera_plan_arg" ]] || die "--camera-plan is required"
[[ -r "$camera_plan_arg" ]] || die "camera plan is not readable: '$camera_plan_arg'"
camera_plan_path="$(realpath "$camera_plan_arg")"
case "$camera_plan_path" in
    "$repo_root"/*) ;;
    *) die "camera plan must be inside the repository so it is visible under /workspace" ;;
esac

devices_output="$(python3 - "$camera_plan_path" <<'PY'
import json
import re
import sys
from pathlib import Path

plan_path = Path(sys.argv[1])


def fail(detail):
    print(f"run_landing_marker_recognition: invalid camera plan: {detail}", file=sys.stderr)
    raise SystemExit(64)


try:
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError) as error:
    fail(str(error))
if not isinstance(plan, dict):
    fail("root must be an object")
cameras = plan.get("cameras")
if not isinstance(cameras, list) or len(cameras) != 2:
    fail("plan must contain exactly narrow and wide cameras")

by_role = {}
placeholder_tokens = ("REPLACE_WITH", "PLACEHOLDER", "UNSET", "TODO", "TBD", "EXAMPLE")
for camera in cameras:
    if not isinstance(camera, dict):
        fail("each camera must be an object")
    role = camera.get("role")
    if role not in {"narrow", "wide"} or role in by_role:
        fail("plan must contain exactly narrow and wide cameras")
    controller_id = camera.get("controller_id")
    if not isinstance(controller_id, str) or not controller_id.strip():
        fail(f"{role} controller_id must be non-empty text")
    if any(token in controller_id.upper() for token in placeholder_tokens):
        fail(f"{role} has placeholder controller_id {controller_id!r}")
    by_id = camera.get("by_id")
    if not isinstance(by_id, str) or re.fullmatch(r"/dev/v4l/by-id/[^/\s]+", by_id) is None:
        fail(f"{role} by_id must be one stable /dev/v4l/by-id path")
    by_role[role] = by_id

if set(by_role) != {"narrow", "wide"}:
    fail("plan must contain exactly narrow and wide cameras")
if by_role["narrow"] == by_role["wide"]:
    fail("narrow and wide must use distinct by-id paths")
print(by_role["narrow"])
print(by_role["wide"])
PY
)"
mapfile -t camera_devices <<<"$devices_output"
((${#camera_devices[@]} == 2)) || die "camera plan did not yield exactly two stable devices"

container_plan_path="/workspace/${camera_plan_path#"$repo_root/"}"
printf -v forwarded_devices '%s\n%s' "${camera_devices[0]}" "${camera_devices[1]}"
native_plan_path="$(mktemp "${TMPDIR:-/tmp}/ed-landing-marker-plan.XXXXXX.json")"
cleanup() {
    rm -f "$native_plan_path"
}
trap cleanup EXIT

python3 - "$camera_plan_path" "$native_plan_path" "$repo_root" <<'PY'
import json
import sys
from pathlib import Path

source_path = Path(sys.argv[1])
destination_path = Path(sys.argv[2])
repo_root = Path(sys.argv[3])
calibration_root = (repo_root / "calibration_data").resolve()
workspace_prefix = "file:///workspace/calibration_data/"
plan = json.loads(source_path.read_text(encoding="utf-8"))

for camera in plan["cameras"]:
    calibration = camera.get("calibration")
    if not isinstance(calibration, dict):
        continue
    camera_info_url = calibration.get("camera_info_url")
    if not isinstance(camera_info_url, str) or not camera_info_url.startswith(workspace_prefix):
        continue
    relative_path = camera_info_url.removeprefix(workspace_prefix)
    native_path = (calibration_root / relative_path).resolve()
    try:
        native_path.relative_to(calibration_root)
    except ValueError:
        print(
            "run_landing_marker_recognition: camera_info_url escapes calibration_data",
            file=sys.stderr,
        )
        raise SystemExit(64)
    calibration["camera_info_url"] = native_path.as_uri()

destination_path.write_text(json.dumps(plan, indent=2) + "\n", encoding="utf-8")
PY

cd "$repo_root"
HUMBLE_GUI=1 \
HUMBLE_INTERACTIVE=1 \
HUMBLE_TIMEOUT_SECONDS=0 \
HUMBLE_V4L2_DEVICES="$forwarded_devices" \
bash "$repo_root/tools/run_humble.sh" bash -lc '
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
camera_plan="$2"
if [[ -r "$1" ]]; then
    camera_plan="$1"
fi
exec ros2 launch ed_uav_bringup landing_marker_recognition.launch.py \
    camera_plan:="$camera_plan" use_rviz:=true
' bash "$native_plan_path" "$container_plan_path"
