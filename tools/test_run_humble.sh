#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly runner="$repo_root/tools/run_humble.sh"

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

assert_not_contains() {
    local needle="$1"
    local path="$2"
    if grep -Fq -- "$needle" "$path"; then
        fail "did not expect '$needle' in $path"
    fi
}

tmpdir="$(mktemp -d)"
cleanup() {
    rm -rf "$tmpdir"
    rm -f "$repo_root/.run-humble-test-dirty"
}
trap cleanup EXIT

mkdir -p "$tmpdir/bin"

cat >"$tmpdir/jammy.os-release" <<'EOF'
ID=ubuntu
VERSION_ID="22.04"
VERSION_CODENAME=jammy
EOF

cat >"$tmpdir/noble.os-release" <<'EOF'
ID=ubuntu
VERSION_ID="24.04"
VERSION_CODENAME=noble
EOF

cat >"$tmpdir/native-setup.bash" <<'EOF'
export HUMBLE_SELECTION=native
EOF

cat >"$tmpdir/native-setup-nounset.bash" <<'EOF'
printf '%s\n' "$AMENT_TRACE_SETUP_FILES" >"${HUMBLE_NATIVE_TRACE:?}"
export HUMBLE_SELECTION=native
EOF

cat >"$tmpdir/bin/docker" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

printf '%s\n' "$*" >>"$FAKE_DOCKER_LOG"

case "${1:-}" in
    info)
        exit "${FAKE_DOCKER_INFO_STATUS:-0}"
        ;;
    image)
        if [[ "${2:-}" == inspect ]]; then
            case "${FAKE_IMAGE_STATE:-missing}" in
                matching)
                    if [[ "$*" == *toolchain-fingerprint* ]]; then
                        printf '%s\n' "$FAKE_EXPECTED_TOOLCHAIN_FINGERPRINT"
                    else
                        printf '%s\n' "$FAKE_EXPECTED_BASE_REF"
                    fi
                    exit 0
                    ;;
                old-toolchain)
                    if [[ "$*" == *toolchain-fingerprint* ]]; then
                        printf '%s\n' '0000000000000000000000000000000000000000000000000000000000000000'
                    else
                        printf '%s\n' "$FAKE_EXPECTED_BASE_REF"
                    fi
                    exit 0
                    ;;
                stale)
                    printf '%s\n' 'ros:humble-ros-base-jammy@sha256:stale'
                    exit 0
                    ;;
                *)
                    exit 1
                    ;;
            esac
        fi
        ;;
    build)
        if [[ -n "${FAKE_BUILD_SLEEP:-}" ]]; then
            sleep "$FAKE_BUILD_SLEEP"
        fi
        if [[ -n "${FAKE_BUILD_MESSAGE:-}" ]]; then
            printf '%s\n' "$FAKE_BUILD_MESSAGE"
        fi
        exit "${FAKE_BUILD_STATUS:-0}"
        ;;
    run)
        if [[ "${FAKE_READ_STDIN:-}" == 1 ]]; then
            cat >"$FAKE_STDIN_CAPTURE"
        fi
        if [[ -n "${FAKE_RUN_SLEEP:-}" ]]; then
            trap 'exit 143' TERM INT
            sleep "$FAKE_RUN_SLEEP" &
            wait "$!"
        fi
        printf 'container-selected\n'
        exit "${FAKE_RUN_STATUS:-0}"
        ;;
esac
EOF
chmod +x "$tmpdir/bin/docker"

expected_base_ref="$(sed -n 's/^ARG ROS_HUMBLE_BASE=//p' "$repo_root/docker/Dockerfile.humble" 2>/dev/null || true)"
expected_toolchain_fingerprint="$(sha256sum "$repo_root/docker/Dockerfile.humble" 2>/dev/null | awk '{print $1}' || true)"

if [[ -z "$expected_base_ref" || -z "$expected_toolchain_fingerprint" ]]; then
    expect_failure 'runner selection tests must be RED before the runner exists' \
        "$runner" bash -lc true
    printf 'RED: runner and pinned Dockerfile are absent as expected\n'
    exit 0
fi

export PATH="$tmpdir/bin:$PATH"
export FAKE_EXPECTED_BASE_REF="$expected_base_ref"
export FAKE_EXPECTED_TOOLCHAIN_FINGERPRINT="$expected_toolchain_fingerprint"
export FAKE_DOCKER_LOG="$tmpdir/docker.log"

expect_success 'Jammy with Humble setup must select native' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/jammy.os-release" \
        HUMBLE_NATIVE_SETUP="$tmpdir/native-setup.bash" \
        "$runner" bash -lc 'test "$HUMBLE_SELECTION" = native'

expect_success 'native setup may dereference optional AMENT trace state' \
    env -u AMENT_TRACE_SETUP_FILES \
        HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/jammy.os-release" \
        HUMBLE_NATIVE_SETUP="$tmpdir/native-setup-nounset.bash" \
        HUMBLE_NATIVE_TRACE="$tmpdir/native-trace.out" \
        "$runner" bash -lc 'test "$HUMBLE_SELECTION" = native'
[[ -f "$tmpdir/native-trace.out" ]] || fail 'native setup did not run with nounset temporarily disabled'
assert_contains $'set +u\n        source "$native_setup"\n        set -u' "$runner"

: >"$FAKE_DOCKER_LOG"
expect_success 'native Jammy behavior remains direct with GUI opt-in set' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/jammy.os-release" \
        HUMBLE_NATIVE_SETUP="$tmpdir/native-setup.bash" \
        HUMBLE_GUI=1 \
        "$runner" bash -lc 'test "$HUMBLE_SELECTION" = native'
assert_not_contains 'docker ' "$FAKE_DOCKER_LOG"

: >"$FAKE_DOCKER_LOG"
native_v4l2_root="$tmpdir/native-v4l2-root"
mkdir -p "$native_v4l2_root/dev/v4l/by-id" "$native_v4l2_root/dev"
ln -s /dev/null "$native_v4l2_root/dev/video0"
ln -s ../../video0 "$native_v4l2_root/dev/v4l/by-id/native-camera-video-index0"
expect_success 'native Jammy dispatch must validate V4L2 without adding container flags' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/jammy.os-release" \
        HUMBLE_NATIVE_SETUP="$tmpdir/native-setup.bash" \
        HUMBLE_V4L2_DEVICE_ROOT="$native_v4l2_root" \
        HUMBLE_V4L2_DEVICES=/dev/v4l/by-id/native-camera-video-index0 \
        NATIVE_ARGS="$tmpdir/native.args" \
        "$runner" bash -lc 'printf "%s\n" "$@" >"$NATIVE_ARGS"' bash native-command-argument
assert_not_contains 'docker ' "$FAKE_DOCKER_LOG"
assert_contains 'native-command-argument' "$tmpdir/native.args"
assert_not_contains '--device' "$tmpdir/native.args"
assert_not_contains '--volume' "$tmpdir/native.args"

native_command_marker="$tmpdir/native-command-ran"
expect_failure 'missing V4L2 request must fail before native Jammy command execution' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/jammy.os-release" \
        HUMBLE_NATIVE_SETUP="$tmpdir/native-setup.bash" \
        HUMBLE_V4L2_DEVICE_ROOT="$native_v4l2_root" \
        HUMBLE_V4L2_DEVICES=/dev/v4l/by-id/missing-camera-video-index0 \
        NATIVE_COMMAND_MARKER="$native_command_marker" \
        "$runner" bash -lc 'touch "$NATIVE_COMMAND_MARKER"' \
        >"$tmpdir/native-v4l2-missing.out" 2>&1
assert_contains 'does not exist' "$tmpdir/native-v4l2-missing.out"
[[ ! -e "$native_command_marker" ]] || fail 'native command ran before missing V4L2 request was rejected'

expect_failure 'unsafe V4L2 request must fail before native Jammy command execution' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/jammy.os-release" \
        HUMBLE_NATIVE_SETUP="$tmpdir/native-setup.bash" \
        HUMBLE_V4L2_DEVICE_ROOT="$native_v4l2_root" \
        HUMBLE_V4L2_DEVICES=/dev/video0 \
        NATIVE_COMMAND_MARKER="$native_command_marker" \
        "$runner" bash -lc 'touch "$NATIVE_COMMAND_MARKER"' \
        >"$tmpdir/native-v4l2-unsafe.out" 2>&1
assert_contains 'must be a stable /dev/v4l/by-id path' "$tmpdir/native-v4l2-unsafe.out"
[[ ! -e "$native_command_marker" ]] || fail 'native command ran before unsafe V4L2 request was rejected'

: >"$FAKE_DOCKER_LOG"
expect_success 'non-Jammy host must select the container' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        FAKE_IMAGE_STATE=missing \
        "$runner" bash -lc true >"$tmpdir/container.out"
assert_contains 'container-selected' "$tmpdir/container.out"
assert_contains 'build ' "$FAKE_DOCKER_LOG"
assert_contains 'run --interactive --rm' "$FAKE_DOCKER_LOG"
assert_not_contains '/dev/v4l/by-id:/dev/v4l/by-id:ro' "$FAKE_DOCKER_LOG"
assert_not_contains '--device /dev/video' "$FAKE_DOCKER_LOG"

v4l2_root="$tmpdir/v4l2-root"
mkdir -p "$v4l2_root/dev/v4l/by-id" "$v4l2_root/dev"
ln -s /dev/null "$v4l2_root/dev/video0"
ln -s /dev/zero "$v4l2_root/dev/video2"
ln -s ../../video0 "$v4l2_root/dev/v4l/by-id/narrow-camera-video-index0"
ln -s ../../video2 "$v4l2_root/dev/v4l/by-id/wide-camera-video-index0"

: >"$FAKE_DOCKER_LOG"
expect_success 'stable V4L2 opt-in must forward only resolved character devices' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        HUMBLE_V4L2_DEVICE_ROOT="$v4l2_root" \
        HUMBLE_V4L2_DEVICES=$'/dev/v4l/by-id/narrow-camera-video-index0\n/dev/v4l/by-id/wide-camera-video-index0' \
        FAKE_IMAGE_STATE=matching \
        "$runner" bash -lc true >"$tmpdir/v4l2.out"
assert_contains "--volume $v4l2_root/dev/v4l/by-id:/dev/v4l/by-id:ro" "$FAKE_DOCKER_LOG"
assert_contains '--device /dev/video0:/dev/video0' "$FAKE_DOCKER_LOG"
assert_contains '--device /dev/video2:/dev/video2' "$FAKE_DOCKER_LOG"
assert_not_contains '--device /dev/v4l/by-id' "$FAKE_DOCKER_LOG"
assert_not_contains '--device /dev/null' "$FAKE_DOCKER_LOG"

absent_v4l2_root="$tmpdir/absent-v4l2-root"
mkdir -p "$absent_v4l2_root/dev"
expect_failure 'V4L2 opt-in must fail clearly when no by-id directory is attached' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        HUMBLE_V4L2_DEVICE_ROOT="$absent_v4l2_root" \
        HUMBLE_V4L2_DEVICES=/dev/v4l/by-id/narrow-camera-video-index0 \
        FAKE_IMAGE_STATE=matching \
        "$runner" bash -lc true >"$tmpdir/v4l2-unattached.out" 2>&1
assert_contains 'V4L2 by-id directory' "$tmpdir/v4l2-unattached.out"
assert_contains 'does not exist' "$tmpdir/v4l2-unattached.out"

expect_failure 'V4L2 opt-in must reject non-by-id paths' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        HUMBLE_V4L2_DEVICE_ROOT="$v4l2_root" \
        HUMBLE_V4L2_DEVICES=/dev/video0 \
        FAKE_IMAGE_STATE=matching \
        "$runner" bash -lc true >"$tmpdir/v4l2-non-by-id.out" 2>&1
assert_contains 'must be a stable /dev/v4l/by-id path' "$tmpdir/v4l2-non-by-id.out"

expect_failure 'V4L2 opt-in must reject missing by-id paths' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        HUMBLE_V4L2_DEVICE_ROOT="$v4l2_root" \
        HUMBLE_V4L2_DEVICES=/dev/v4l/by-id/missing-camera-video-index0 \
        FAKE_IMAGE_STATE=matching \
        "$runner" bash -lc true >"$tmpdir/v4l2-missing.out" 2>&1
assert_contains 'does not exist' "$tmpdir/v4l2-missing.out"

: >"$v4l2_root/dev/video1"
ln -s ../../video1 "$v4l2_root/dev/v4l/by-id/regular-file-video-index0"
expect_failure 'V4L2 opt-in must reject non-character targets' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        HUMBLE_V4L2_DEVICE_ROOT="$v4l2_root" \
        HUMBLE_V4L2_DEVICES=/dev/v4l/by-id/regular-file-video-index0 \
        FAKE_IMAGE_STATE=matching \
        "$runner" bash -lc true >"$tmpdir/v4l2-regular.out" 2>&1
assert_contains 'is not a character device' "$tmpdir/v4l2-regular.out"

ln -s ../../../../outside "$v4l2_root/dev/v4l/by-id/outside-video-index0"
expect_failure 'V4L2 opt-in must reject links resolving outside /dev/videoN' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        HUMBLE_V4L2_DEVICE_ROOT="$v4l2_root" \
        HUMBLE_V4L2_DEVICES=/dev/v4l/by-id/outside-video-index0 \
        FAKE_IMAGE_STATE=matching \
        "$runner" bash -lc true >"$tmpdir/v4l2-outside.out" 2>&1
assert_contains 'must resolve to /dev/videoN' "$tmpdir/v4l2-outside.out"

ln -s ../../video-camera "$v4l2_root/dev/v4l/by-id/non-numbered-video-index0"
expect_failure 'V4L2 opt-in must reject non-numbered video targets' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        HUMBLE_V4L2_DEVICE_ROOT="$v4l2_root" \
        HUMBLE_V4L2_DEVICES=/dev/v4l/by-id/non-numbered-video-index0 \
        FAKE_IMAGE_STATE=matching \
        "$runner" bash -lc true >"$tmpdir/v4l2-non-numbered.out" 2>&1
assert_contains 'must resolve to /dev/videoN' "$tmpdir/v4l2-non-numbered.out"

expect_failure 'V4L2 test root must remain test-only' \
    env HUMBLE_V4L2_DEVICE_ROOT="$v4l2_root" \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        HUMBLE_V4L2_DEVICES=/dev/v4l/by-id/narrow-camera-video-index0 \
        FAKE_IMAGE_STATE=matching \
        "$runner" bash -lc true >"$tmpdir/v4l2-test-root.out" 2>&1
assert_contains 'is test-only' "$tmpdir/v4l2-test-root.out"

: >"$FAKE_DOCKER_LOG"
expect_success 'old image with matching base must rebuild after toolchain changes' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        FAKE_IMAGE_STATE=old-toolchain \
        "$runner" bash -lc true >"$tmpdir/old-toolchain.out"
assert_contains 'build ' "$FAKE_DOCKER_LOG"
assert_contains 'run --interactive --rm' "$FAKE_DOCKER_LOG"

: >"$FAKE_DOCKER_LOG"
expect_failure 'non-Jammy host without a usable runtime must fail' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        FAKE_DOCKER_INFO_STATUS=1 \
        "$runner" bash -lc true >"$tmpdir/runtime.out" 2>&1
assert_contains 'not usable' "$tmpdir/runtime.out"

: >"$FAKE_DOCKER_LOG"
expect_success 'stale cached image must be rebuilt' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        FAKE_IMAGE_STATE=stale \
        "$runner" bash -lc true >"$tmpdir/stale.out"
assert_contains 'build ' "$FAKE_DOCKER_LOG"

: >"$FAKE_DOCKER_LOG"
expect_success 'matching cached image must be reused' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        FAKE_IMAGE_STATE=matching \
        "$runner" bash -lc true >"$tmpdir/matching.out"
assert_not_contains 'build ' "$FAKE_DOCKER_LOG"

: >"$FAKE_DOCKER_LOG"
expect_failure 'hung image build must time out' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        FAKE_IMAGE_STATE=missing \
        FAKE_BUILD_SLEEP=3 \
        HUMBLE_TIMEOUT_SECONDS=1 \
        "$runner" bash -lc true >"$tmpdir/timeout.out" 2>&1

: >"$FAKE_DOCKER_LOG"
expect_failure 'misleading build success output must not mask a failure' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        FAKE_IMAGE_STATE=missing \
        FAKE_BUILD_MESSAGE='build succeeded' \
        FAKE_BUILD_STATUS=17 \
        "$runner" bash -lc true >"$tmpdir/misleading.out" 2>&1
assert_not_contains 'run ' "$FAKE_DOCKER_LOG"

expect_failure 'missing command must fail' "$runner"
expect_failure 'invalid runtime override must fail' \
    env HUMBLE_CONTAINER_RUNTIME='docker --debug' "$runner" bash -lc true
expect_failure 'invalid timeout override must fail' \
    env HUMBLE_TIMEOUT_SECONDS=forever "$runner" bash -lc true
expect_failure 'bounded mode must reject a zero timeout' \
    env HUMBLE_TIMEOUT_SECONDS=0 "$runner" bash -lc true
expect_failure 'invalid interactive override must fail' \
    env HUMBLE_INTERACTIVE=maybe "$runner" bash -lc true

: >"$FAKE_DOCKER_LOG"
expect_success 'interactive mode must omit the outer timeout and attach stdin' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        FAKE_IMAGE_STATE=matching \
        HUMBLE_INTERACTIVE=1 \
        HUMBLE_TIMEOUT_SECONDS=0 \
        "$runner" bash -lc true >"$tmpdir/interactive.out"
assert_contains '--interactive' "$FAKE_DOCKER_LOG"

: >"$FAKE_DOCKER_LOG"
: >"$tmpdir/interactive.stdin"
printf 'interactive-stdin\n' | expect_success 'interactive mode must preserve attached stdin' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        FAKE_IMAGE_STATE=matching \
        FAKE_READ_STDIN=1 \
        FAKE_STDIN_CAPTURE="$tmpdir/interactive.stdin" \
        HUMBLE_INTERACTIVE=1 \
        HUMBLE_TIMEOUT_SECONDS=0 \
        "$runner" bash -s >"$tmpdir/interactive-stdin.out"
assert_contains 'interactive-stdin' "$tmpdir/interactive.stdin"

cat >"$tmpdir/malformed.os-release" <<'EOF'
VERSION_ID="22.04"
EOF
expect_failure 'malformed OS-release test override must fail' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/malformed.os-release" \
        "$runner" bash -lc true

gui_env=(
    DISPLAY=:0
    WAYLAND_DISPLAY=wayland-0
    XDG_RUNTIME_DIR=/run/user/1000
)
expect_success 'GUI container mode must forward the WSLg runtime' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        FAKE_IMAGE_STATE=matching \
        HUMBLE_GUI=1 \
        "${gui_env[@]}" \
        "$runner" bash -lc true >"$tmpdir/gui.out"
assert_contains '--env DISPLAY=:0' "$FAKE_DOCKER_LOG"
assert_contains '--env WAYLAND_DISPLAY=wayland-0' "$FAKE_DOCKER_LOG"
assert_contains '--env XDG_RUNTIME_DIR=/tmp/xdg-runtime' "$FAKE_DOCKER_LOG"
assert_contains '--volume /tmp/.X11-unix:/tmp/.X11-unix:rw' "$FAKE_DOCKER_LOG"
assert_contains '--volume /mnt/wslg:/mnt/wslg:ro' "$FAKE_DOCKER_LOG"
assert_contains '--volume /run/user/1000:/tmp/xdg-runtime:ro' "$FAKE_DOCKER_LOG"
assert_contains '--env QT_X11_NO_MITSHM=1' "$FAKE_DOCKER_LOG"
assert_contains '--env LIBGL_ALWAYS_SOFTWARE=1' "$FAKE_DOCKER_LOG"
assert_contains '--env MESA_LOADER_DRIVER_OVERRIDE=llvmpipe' "$FAKE_DOCKER_LOG"

expect_failure 'GUI mode without DISPLAY must fail clearly' \
    env HUMBLE_TESTING=1 \
        DISPLAY= \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        HUMBLE_GUI=1 \
        WAYLAND_DISPLAY=wayland-0 \
        XDG_RUNTIME_DIR=/run/user/1000 \
        "$runner" bash -lc true >"$tmpdir/gui-missing-display.out" 2>&1
assert_contains 'HUMBLE_GUI requires DISPLAY=:0' "$tmpdir/gui-missing-display.out"

expect_failure 'GUI mode with a missing WSLg mount must fail clearly' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        HUMBLE_GUI=1 \
        DISPLAY=:0 \
        WAYLAND_DISPLAY=wayland-0 \
        XDG_RUNTIME_DIR="$tmpdir/missing-runtime" \
        "$runner" bash -lc true >"$tmpdir/gui-missing-runtime.out" 2>&1
assert_contains 'HUMBLE_GUI requires XDG_RUNTIME_DIR directory' "$tmpdir/gui-missing-runtime.out"

expect_failure 'invalid GUI override must fail' \
    env HUMBLE_GUI=yes "$runner" bash -lc true

touch "$repo_root/.run-humble-test-dirty"
: >"$FAKE_DOCKER_LOG"
expect_success 'dirty worktree must not prevent container use' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        FAKE_IMAGE_STATE=matching \
        "$runner" bash -lc true >"$tmpdir/dirty.out"
rm -f "$repo_root/.run-humble-test-dirty"

: >"$FAKE_DOCKER_LOG"
env HUMBLE_TESTING=1 \
    HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
    HUMBLE_CONTAINER_RUNTIME=docker \
    FAKE_IMAGE_STATE=matching \
    FAKE_RUN_SLEEP=30 \
    "$runner" bash -lc true >"$tmpdir/interrupted.out" 2>&1 &
runner_pid=$!
sleep 1
kill -TERM "$runner_pid"
expect_failure 'interrupted runner must return a failure status' wait "$runner_pid"
if kill -0 "$runner_pid" 2>/dev/null; then
    fail 'interrupted runner is still running'
fi

expect_success 'runner must work after interruption' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        FAKE_IMAGE_STATE=matching \
        "$runner" bash -lc true >"$tmpdir/rerun.out"
assert_contains 'container-selected' "$tmpdir/rerun.out"

printf 'GREEN: runner-selection, stale-image, timeout, interruption, dirty-worktree, and malformed-input checks passed\n'
