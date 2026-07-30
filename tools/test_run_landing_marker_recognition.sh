#!/usr/bin/env bash
set -euo pipefail

readonly source_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

fail() {
    printf 'FAIL: %s\n' "$*" >&2
    exit 1
}

expect_success() {
    local description="$1"
    shift
    "$@" || fail "$description"
}

expect_failure() {
    local description="$1"
    shift
    if "$@"; then
        fail "$description unexpectedly succeeded"
    fi
}

assert_contains() {
    local needle="$1"
    local path="$2"
    grep -Fq -- "$needle" "$path" || fail "expected '$needle' in $path"
}

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT
repo_root="$tmpdir/repo"
mkdir -p "$repo_root/tools" "$repo_root/config/cameras"
cp "$source_root/tools/run_landing_marker_recognition.sh" "$repo_root/tools/"

cat >"$repo_root/tools/run_humble.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf 'HUMBLE_GUI=%s\n' "${HUMBLE_GUI:-}" >"${FAKE_HUMBLE_ENV:?}"
printf 'HUMBLE_INTERACTIVE=%s\n' "${HUMBLE_INTERACTIVE:-}" >>"$FAKE_HUMBLE_ENV"
printf 'HUMBLE_TIMEOUT_SECONDS=%s\n' "${HUMBLE_TIMEOUT_SECONDS:-}" >>"$FAKE_HUMBLE_ENV"
printf 'HUMBLE_V4L2_DEVICES=%s\n' "${HUMBLE_V4L2_DEVICES:-}" >>"$FAKE_HUMBLE_ENV"
printf '%s\n' "$@" >"${FAKE_HUMBLE_ARGS:?}"
printf '%s\n' "${5:?}" >"${FAKE_NATIVE_PLAN_PATH:?}"
cp "${5:?}" "${FAKE_NATIVE_PLAN_COPY:?}"
EOF
chmod +x "$repo_root/tools/run_humble.sh"
runner="$repo_root/tools/run_landing_marker_recognition.sh"
example_plan="$source_root/config/cameras/landing_marker_camera_plan.example.json"

[[ -x "$source_root/tools/run_landing_marker_recognition.sh" ]] || fail 'landing-marker runner must be executable'
python3 - "$example_plan" <<'PY'
import json
import sys
from pathlib import Path

plan = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
cameras = {camera["role"]: camera for camera in plan["cameras"]}
expected = {
    "narrow": {
        "serial": "usb-revision:0ac8:3460:0122",
        "by_id": "/dev/v4l/by-id/usb-DHZJ-250122-ZW_W19_HD_Webcam-video-index0",
        "profile": "narrow_live",
        "fps": 20,
        "captured_at_ns": 1785340796844314977,
        "url": "file:///workspace/calibration_data/narrow_1280x720_20260729T155820Z/camera_info.yaml",
    },
    "wide": {
        "serial": "usb-revision:0ac8:3460:0708",
        "by_id": "/dev/v4l/by-id/usb-DHZJ-240708-XH_W19_HD_Webcam-video-index0",
        "profile": "wide_live",
        "fps": 15,
        "captured_at_ns": 1785341018686903931,
        "url": "file:///workspace/calibration_data/wide_1280x720_20260729T160215Z/camera_info.yaml",
    },
}
assert set(cameras) == set(expected)
for role, values in expected.items():
    camera = cameras[role]
    calibration = camera["calibration"]
    mode = camera["mode"]
    assert camera["serial"] == values["serial"]
    assert camera["observed_serial"] == values["serial"]
    assert camera["by_id"] == values["by_id"]
    assert camera["controller_id"] == "REPLACE_WITH_P25_CONTROLLER_ID"
    assert camera["profile"] == values["profile"]
    assert mode == {
        "fourcc": "MJPG",
        "width": 1280,
        "height": 720,
        "frames_per_second": values["fps"],
        "compression": "mjpeg",
        "declared_peak_mbit_s": 64.0 if role == "narrow" else 48.0,
    }
    assert calibration["captured_at_ns"] == values["captured_at_ns"]
    assert calibration["valid_for_ns"] == 31536000000000000
    assert calibration["camera_info_url"] == values["url"]
    assert calibration["capture_provenance"] == "direct_v4l2"
    assert calibration["observed_by_id"] == values["by_id"]
PY

write_plan() {
    local path="$1"
    local narrow_controller="$2"
    local wide_controller="$3"
    cat >"$path" <<EOF
{
  "controller_budget_mbit_s": 384.0,
  "cameras": [
    {
      "role": "narrow",
      "by_id": "/dev/v4l/by-id/narrow-camera-video-index0",
      "controller_id": "$narrow_controller",
      "calibration": {
        "camera_info_url": "file:///workspace/calibration_data/narrow/camera_info.yaml",
        "observed_by_id": "/dev/v4l/by-id/not-forwarded-narrow"
      }
    },
    {
      "role": "wide",
      "by_id": "/dev/v4l/by-id/wide-camera-video-index0",
      "controller_id": "$wide_controller",
      "calibration": {
        "camera_info_url": "file:///workspace/calibration_data/wide/camera_info.yaml",
        "observed_by_id": "/dev/v4l/by-id/not-forwarded-wide"
      }
    }
  ]
}
EOF
}

expect_success '--help must succeed without launching Humble' \
    "$runner" --help >"$tmpdir/help.out"
assert_contains 'Usage:' "$tmpdir/help.out"
assert_contains '--camera-plan' "$tmpdir/help.out"

expect_failure 'camera plan must be required' \
    "$runner" >"$tmpdir/missing-argument.out" 2>&1
assert_contains '--camera-plan is required' "$tmpdir/missing-argument.out"

expect_failure 'missing camera plan must fail clearly' \
    "$runner" --camera-plan "$repo_root/config/cameras/missing.json" \
    >"$tmpdir/missing-plan.out" 2>&1
assert_contains 'camera plan is not readable' "$tmpdir/missing-plan.out"

write_plan \
    "$repo_root/config/cameras/placeholder.json" \
    REPLACE_WITH_P25_CONTROLLER_ID \
    controller-wide
expect_failure 'placeholder controller IDs must be rejected' \
    "$runner" --camera-plan "$repo_root/config/cameras/placeholder.json" \
    >"$tmpdir/placeholder.out" 2>&1
assert_contains 'placeholder controller_id' "$tmpdir/placeholder.out"

write_plan "$repo_root/config/cameras/non-by-id.json" controller-narrow controller-wide
sed -i '0,/\/dev\/v4l\/by-id\/narrow-camera-video-index0/s||/dev/video0|' \
    "$repo_root/config/cameras/non-by-id.json"
expect_failure 'numeric video paths in a plan must be rejected' \
    "$runner" --camera-plan "$repo_root/config/cameras/non-by-id.json" \
    >"$tmpdir/non-by-id.out" 2>&1
assert_contains 'stable /dev/v4l/by-id path' "$tmpdir/non-by-id.out"

cat >"$repo_root/config/cameras/one-camera.json" <<'EOF'
{"cameras":[{"role":"narrow","by_id":"/dev/v4l/by-id/only-one","controller_id":"controller-narrow"}]}
EOF
expect_failure 'plan must contain exactly two cameras' \
    "$runner" --camera-plan "$repo_root/config/cameras/one-camera.json" \
    >"$tmpdir/one-camera.out" 2>&1
assert_contains 'exactly narrow and wide cameras' "$tmpdir/one-camera.out"

write_plan "$tmpdir/outside-plan.json" controller-narrow controller-wide
expect_failure 'plan outside the mounted workspace must be rejected' \
    "$runner" --camera-plan "$tmpdir/outside-plan.json" \
    >"$tmpdir/outside-plan.out" 2>&1
assert_contains 'must be inside the repository' "$tmpdir/outside-plan.out"

valid_plan="$repo_root/config/cameras/valid.json"
write_plan "$valid_plan" controller-narrow controller-wide
source_plan_checksum="$(sha256sum "$valid_plan" | awk '{print $1}')"
export FAKE_HUMBLE_ENV="$tmpdir/humble.env"
export FAKE_HUMBLE_ARGS="$tmpdir/humble.args"
export FAKE_NATIVE_PLAN_PATH="$tmpdir/native-plan.path"
export FAKE_NATIVE_PLAN_COPY="$tmpdir/native-plan.json"
expect_success 'valid measured plan must launch the focused bringup' \
    bash -c 'cd "$1" && exec "$2" --camera-plan config/cameras/valid.json' \
    bash "$repo_root" "$runner"

assert_contains 'HUMBLE_GUI=1' "$FAKE_HUMBLE_ENV"
assert_contains 'HUMBLE_INTERACTIVE=1' "$FAKE_HUMBLE_ENV"
assert_contains 'HUMBLE_TIMEOUT_SECONDS=0' "$FAKE_HUMBLE_ENV"
assert_contains 'HUMBLE_V4L2_DEVICES=/dev/v4l/by-id/narrow-camera-video-index0' "$FAKE_HUMBLE_ENV"
assert_contains '/dev/v4l/by-id/wide-camera-video-index0' "$FAKE_HUMBLE_ENV"
if grep -Fq 'not-forwarded' "$FAKE_HUMBLE_ENV"; then
    fail 'runner forwarded calibration metadata instead of exactly the two camera bindings'
fi
assert_contains 'source /opt/ros/humble/setup.bash' "$FAKE_HUMBLE_ARGS"
assert_contains 'source ros2_ws/install/setup.bash' "$FAKE_HUMBLE_ARGS"
assert_contains 'ros2 launch ed_uav_bringup landing_marker_recognition.launch.py' "$FAKE_HUMBLE_ARGS"
assert_contains 'camera_plan:="$camera_plan"' "$FAKE_HUMBLE_ARGS"
assert_contains 'if [[ -r "$1" ]]' "$FAKE_HUMBLE_ARGS"
assert_contains 'use_rviz:=true' "$FAKE_HUMBLE_ARGS"
assert_contains '/workspace/config/cameras/valid.json' "$FAKE_HUMBLE_ARGS"
[[ "$(sha256sum "$valid_plan" | awk '{print $1}')" == "$source_plan_checksum" ]] || \
    fail 'runner mutated the supplied camera plan'
normalized_plan_path="$(cat "$FAKE_NATIVE_PLAN_PATH")"
[[ "$normalized_plan_path" != "$valid_plan" ]] || fail 'native path reused the supplied camera plan'
[[ ! -e "$normalized_plan_path" ]] || fail 'temporary native camera plan was not removed'
python3 - "$valid_plan" "$FAKE_NATIVE_PLAN_COPY" "$repo_root" <<'PY'
import json
import sys
from pathlib import Path

source = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
normalized = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
repo_root = Path(sys.argv[3])

source_urls = [camera["calibration"]["camera_info_url"] for camera in source["cameras"]]
normalized_urls = [camera["calibration"]["camera_info_url"] for camera in normalized["cameras"]]
assert source_urls == [
    "file:///workspace/calibration_data/narrow/camera_info.yaml",
    "file:///workspace/calibration_data/wide/camera_info.yaml",
]
assert normalized_urls == [
    (repo_root / "calibration_data/narrow/camera_info.yaml").as_uri(),
    (repo_root / "calibration_data/wide/camera_info.yaml").as_uri(),
]
for source_camera, normalized_camera in zip(source["cameras"], normalized["cameras"], strict=True):
    source_calibration = source_camera["calibration"]
    normalized_calibration = normalized_camera["calibration"]
    assert normalized_camera | {"calibration": source_calibration} == source_camera
    assert normalized_calibration | {"camera_info_url": source_calibration["camera_info_url"]} == source_calibration
PY

printf 'GREEN: landing-marker runner plan and launch contracts passed\n'
