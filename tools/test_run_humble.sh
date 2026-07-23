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
                    printf '%s\n' "$FAKE_EXPECTED_BASE_REF"
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

if [[ -z "$expected_base_ref" ]]; then
    expect_failure 'runner selection tests must be RED before the runner exists' \
        "$runner" bash -lc true
    printf 'RED: runner and pinned Dockerfile are absent as expected\n'
    exit 0
fi

export PATH="$tmpdir/bin:$PATH"
export FAKE_EXPECTED_BASE_REF="$expected_base_ref"
export FAKE_DOCKER_LOG="$tmpdir/docker.log"

expect_success 'Jammy with Humble setup must select native' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/jammy.os-release" \
        HUMBLE_NATIVE_SETUP="$tmpdir/native-setup.bash" \
        "$runner" bash -lc 'test "$HUMBLE_SELECTION" = native'

: >"$FAKE_DOCKER_LOG"
expect_success 'non-Jammy host must select the container' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/noble.os-release" \
        HUMBLE_CONTAINER_RUNTIME=docker \
        FAKE_IMAGE_STATE=missing \
        "$runner" bash -lc true >"$tmpdir/container.out"
assert_contains 'container-selected' "$tmpdir/container.out"
assert_contains 'build ' "$FAKE_DOCKER_LOG"
assert_contains 'run --rm' "$FAKE_DOCKER_LOG"

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

cat >"$tmpdir/malformed.os-release" <<'EOF'
VERSION_ID="22.04"
EOF
expect_failure 'malformed OS-release test override must fail' \
    env HUMBLE_TESTING=1 \
        HUMBLE_OS_RELEASE_FILE="$tmpdir/malformed.os-release" \
        "$runner" bash -lc true

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
