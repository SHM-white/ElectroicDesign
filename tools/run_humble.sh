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
EOF
}

require_test_hook() {
    if [[ "${HUMBLE_TESTING:-}" != 1 ]]; then
        die "$1 is test-only; remove it for normal use"
    fi
}

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
    [[ "$base_ref" =~ ^ros:humble-ros-base-jammy@sha256:[0-9a-f]{64}$ ]] \
        || die "Dockerfile does not contain a pinned Humble amd64 base image"
    printf '%s\n' "$base_ref"
}

toolchain_fingerprint() {
    local fingerprint

    command -v sha256sum >/dev/null 2>&1 \
        || die "sha256sum is required to identify the Dockerfile toolchain"
    fingerprint="$(sha256sum "$dockerfile" | awk '{print $1}')"
    [[ "$fingerprint" =~ ^[0-9a-f]{64}$ ]] \
        || die "could not calculate a valid Dockerfile toolchain fingerprint"
    printf '%s\n' "$fingerprint"
}

container_runtime() {
    local requested_runtime="${HUMBLE_CONTAINER_RUNTIME:-}"
    local candidate

    if [[ -n "$requested_runtime" ]]; then
        case "$requested_runtime" in
            docker|podman)
                candidate="$requested_runtime"
                ;;
            *)
                die "HUMBLE_CONTAINER_RUNTIME must be 'docker' or 'podman'"
                ;;
        esac
    else
        for candidate in docker podman; do
            if command -v "$candidate" >/dev/null 2>&1; then
                break
            fi
            candidate=""
        done
    fi

    [[ -n "${candidate:-}" ]] || die "no Docker or Podman runtime found; install one or run this on Jammy with /opt/ros/humble"
    command -v "$candidate" >/dev/null 2>&1 \
        || die "requested container runtime '$candidate' is not installed"
    "$candidate" info >/dev/null 2>&1 \
        || die "container runtime '$candidate' is not usable; start its daemon or select a working runtime"
    printf '%s\n' "$candidate"
}

bounded() {
    command -v timeout >/dev/null 2>&1 \
        || die "GNU timeout is required to bound container pulls and builds"
    timeout --foreground "${HUMBLE_TIMEOUT_SECONDS:-900}s" "$@"
}

image_matches_base() {
    local runtime="$1"
    local base_ref="$2"
    local actual_ref

    actual_ref="$("$runtime" image inspect --format "{{ index .Config.Labels \"$image_label\" }}" "$image_name" 2>/dev/null)" \
        || return 1
    [[ "$actual_ref" == "$base_ref" ]]
}

image_matches_toolchain() {
    local runtime="$1"
    local expected_fingerprint="$2"
    local actual_fingerprint

    actual_fingerprint="$($runtime image inspect --format "{{ index .Config.Labels \"$toolchain_image_label\" }}" "$image_name" 2>/dev/null)" \
        || return 1
    [[ "$actual_fingerprint" == "$expected_fingerprint" ]]
}

ensure_image() {
    local runtime="$1"
    local base_ref="$2"
    local fingerprint="$3"

    if image_matches_base "$runtime" "$base_ref" \
        && image_matches_toolchain "$runtime" "$fingerprint"; then
        printf 'run_humble: using cached %s\n' "$image_name" >&2
        return
    fi

    printf 'run_humble: building %s from %s\n' "$image_name" "$base_ref" >&2
    bounded "$runtime" build \
        --platform linux/amd64 \
        --build-arg "ROS_HUMBLE_BASE=$base_ref" \
        --build-arg "TOOLCHAIN_FINGERPRINT=$fingerprint" \
        --file "$dockerfile" \
        --tag "$image_name" \
        "$repo_root/docker"
}

gui_args() {
    [[ "${HUMBLE_GUI:-}" == 1 ]] || {
        [[ -z "${HUMBLE_GUI:-}" ]] \
            || die "HUMBLE_GUI must be 1 when GUI forwarding is enabled"
        return
    }

    [[ "${DISPLAY:-}" == :0 ]] || die "HUMBLE_GUI requires DISPLAY=:0"
    [[ -n "${WAYLAND_DISPLAY:-}" ]] \
        || die "HUMBLE_GUI requires WAYLAND_DISPLAY"
    [[ -n "${XDG_RUNTIME_DIR:-}" && -d "$XDG_RUNTIME_DIR" ]] \
        || die "HUMBLE_GUI requires XDG_RUNTIME_DIR directory"
    [[ -d /tmp/.X11-unix ]] \
        || die "HUMBLE_GUI requires /tmp/.X11-unix"
    [[ -d /mnt/wslg ]] \
        || die "HUMBLE_GUI requires /mnt/wslg"
}

main() {
    local native_setup="/opt/ros/humble/setup.bash"
    local base_ref
    local runtime
    local fingerprint
    local -a gui_run_args=()

    if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
        usage
        return
    fi
    (($# > 0)) || {
        usage >&2
        exit 64
    }

    [[ "${HUMBLE_TIMEOUT_SECONDS:-900}" =~ ^[1-9][0-9]*$ ]] \
        || die "HUMBLE_TIMEOUT_SECONDS must be a positive number of seconds"

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

    if is_jammy && [[ -r "$native_setup" ]]; then
        # shellcheck disable=SC1090
        source "$native_setup"
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
            --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
            --volume /mnt/wslg:/mnt/wslg:ro
            --volume "$XDG_RUNTIME_DIR:/tmp/xdg-runtime:ro"
        )
    fi
    ensure_image "$runtime" "$base_ref" "$fingerprint"
    exec timeout --foreground "${HUMBLE_TIMEOUT_SECONDS:-900}s" "$runtime" run \
        --rm \
        --init \
        --platform linux/amd64 \
        --env ROS_HOME=/opt/ed-ros-home \
        "${gui_run_args[@]}" \
        --volume "$repo_root:/workspace" \
        --workdir /workspace \
        "$image_name" \
        "$@"
}

main "$@"
