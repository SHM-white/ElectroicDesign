#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly dockerfile="$repo_root/docker/Dockerfile.humble"
readonly image_name="ed-humble-toolchain:jammy-humble"
readonly image_label="io.ed.humble.base-ref"
readonly toolchain_image_label="io.ed.humble.toolchain-fingerprint"

die() {
    printf 'run_humble: %s\n' "$*" >&2
    exit 64
}

usage() {
    cat <<'EOF'
Usage: ./tools/run_humble.sh <command> [arguments...]

Runs a command in ROS 2 Humble. Native `/opt/ros/humble` is used only on
Ubuntu 22.04 (Jammy); every other host uses the pinned linux/amd64 container.

Container mode is bounded by HUMBLE_TIMEOUT_SECONDS (default: 900). Set
HUMBLE_INTERACTIVE=1 for an attached, unbounded container session; in that
mode HUMBLE_TIMEOUT_SECONDS may be 0 because no outer timeout is used.

Set HUMBLE_V4L2_DEVICES to newline-separated /dev/v4l/by-id paths to opt in
to camera validation before native/container selection. Container mode mounts
the by-id directory read-only and forwards only the character devices resolved
as /dev/videoN; native commands receive no container arguments.
EOF
}

require_test_hook() {
    if [[ "${HUMBLE_TESTING:-}" != 1 ]]; then
        die "$1 is test-only; remove it for normal use"
    fi
}

source "$repo_root/tools/run_humble_support.sh"

os_value() {
    local key="$1"
    local source_file="$2"
    local value
    value="$(sed -n "s/^${key}=//p" "$source_file" | head -n 1)"
    value="${value#\"}"
    value="${value%\"}"
    printf '%s\n' "$value"
}

is_jammy() {
    local source_file="/etc/os-release"
    local os_id
    local version_id
    if [[ -n "${HUMBLE_OS_RELEASE_FILE:-}" ]]; then
        require_test_hook HUMBLE_OS_RELEASE_FILE
        source_file="$HUMBLE_OS_RELEASE_FILE"
    fi
    [[ -r "$source_file" ]] || die "cannot read OS release data at '$source_file'"
    os_id="$(os_value ID "$source_file")"
    version_id="$(os_value VERSION_ID "$source_file")"
    [[ -n "$os_id" && -n "$version_id" ]] || die "'$source_file' is missing ID or VERSION_ID"
    [[ "$os_id" == ubuntu && "$version_id" == 22.04 ]]
}

base_image_ref() {
    local base_ref
    [[ -r "$dockerfile" ]] || die "missing '$dockerfile'"
    base_ref="$(sed -n 's/^ARG ROS_HUMBLE_BASE=//p' "$dockerfile" | head -n 1)"
    [[ "$base_ref" =~ ^ros:humble-ros-base-jammy@sha256:[0-9a-f]{64}$ ]] || die "Dockerfile does not contain a pinned Humble amd64 base image"
    printf '%s\n' "$base_ref"
}

toolchain_fingerprint() {
    local fingerprint
    command -v sha256sum >/dev/null 2>&1 || die "sha256sum is required to identify the Dockerfile toolchain"
    fingerprint="$(sha256sum "$dockerfile" | awk '{print $1}')"
    [[ "$fingerprint" =~ ^[0-9a-f]{64}$ ]] || die "could not calculate a valid Dockerfile toolchain fingerprint"
    printf '%s\n' "$fingerprint"
}

main() {
    local native_setup="/opt/ros/humble/setup.bash"
    local base_ref
    local runtime
    local fingerprint
    local container_name="ed-humble-run-$$"
    local container_pid=""
    local interactive_mode=0
    local exit_code
    local -a gui_run_args=()
    local -a v4l2_run_args=()
    local -a container_run_args=()

    if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
        usage
        return
    fi
    (($# > 0)) || { usage >&2; exit 64; }
    validate_mode_and_timeout
    [[ "${HUMBLE_INTERACTIVE:-0}" == 1 ]] && interactive_mode=1

    if [[ -n "${HUMBLE_CONTAINER_RUNTIME:-}" ]]; then
        case "$HUMBLE_CONTAINER_RUNTIME" in
            docker|podman) ;;
            *) die "HUMBLE_CONTAINER_RUNTIME must be 'docker' or 'podman'" ;;
        esac
    fi
    if [[ -n "${HUMBLE_NATIVE_SETUP:-}" ]]; then
        require_test_hook HUMBLE_NATIVE_SETUP
        native_setup="$HUMBLE_NATIVE_SETUP"
    fi
    v4l2_args
    if is_jammy && [[ -r "$native_setup" ]]; then
        # shellcheck disable=SC1090
        set +u
        source "$native_setup"
        set -u
        exec "$@"
    fi

    base_ref="$(base_image_ref)"
    fingerprint="$(toolchain_fingerprint)"
    runtime="$(container_runtime)"
    gui_args
    if [[ "${HUMBLE_GUI:-}" == 1 ]]; then
        gui_run_args=(
            --env "DISPLAY=$DISPLAY"
            --env "WAYLAND_DISPLAY=$WAYLAND_DISPLAY"
            --env XDG_RUNTIME_DIR=/tmp/xdg-runtime
            --env QT_X11_NO_MITSHM=1
            --env LIBGL_ALWAYS_SOFTWARE=1
            --env MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
            --env PULSE_SERVER=unix:/mnt/wslg/PulseServer
            --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
            --volume /mnt/wslg:/mnt/wslg:ro
            --volume "$XDG_RUNTIME_DIR:/tmp/xdg-runtime:ro"
        )
    fi
    ensure_image "$runtime" "$base_ref" "$fingerprint"
    container_run_args=(
        run --interactive --rm --init --platform linux/amd64
        --env ROS_HOME=/opt/ed-ros-home
        "${gui_run_args[@]}"
        "${v4l2_run_args[@]}"
        --volume "$repo_root:/workspace"
        --workdir /workspace
        "$image_name"
    )

    if ((interactive_mode)); then
        container_run_args=(run --name "$container_name" "${container_run_args[@]:1}")
        cleanup_interactive() {
            exit_code="$?"
            trap - EXIT INT TERM
            if [[ -n "$container_pid" ]] && kill -0 "$container_pid" 2>/dev/null; then
                "$runtime" rm --force "$container_name" >/dev/null 2>&1 || true
                wait "$container_pid" 2>/dev/null || true
            fi
            exit "$exit_code"
        }
        trap cleanup_interactive EXIT
        trap 'exit 130' INT
        trap 'exit 143' TERM
        "$runtime" "${container_run_args[@]}" "$@" <&0 &
        container_pid="$!"
        wait "$container_pid"
        exit_code="$?"
        exit "$exit_code"
    fi

    exec timeout --foreground "${HUMBLE_TIMEOUT_SECONDS:-900}s" "$runtime" "${container_run_args[@]}" "$@"
}

main "$@"
