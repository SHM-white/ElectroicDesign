#!/usr/bin/env bash

container_runtime() {
    local requested_runtime="${HUMBLE_CONTAINER_RUNTIME:-}"
    local candidate

    if [[ -n "$requested_runtime" ]]; then
        case "$requested_runtime" in
            docker|podman) candidate="$requested_runtime" ;;
            *) die "HUMBLE_CONTAINER_RUNTIME must be 'docker' or 'podman'" ;;
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
    command -v "$candidate" >/dev/null 2>&1 || die "requested container runtime '$candidate' is not installed"
    "$candidate" info >/dev/null 2>&1 || die "container runtime '$candidate' is not usable; start its daemon or select a working runtime"
    printf '%s\n' "$candidate"
}

bounded() {
    command -v timeout >/dev/null 2>&1 || die "GNU timeout is required to bound container pulls and builds"
    timeout --foreground "${HUMBLE_TIMEOUT_SECONDS:-900}s" "$@"
}

validate_mode_and_timeout() {
    case "${HUMBLE_INTERACTIVE:-0}" in
        ""|0)
            [[ "${HUMBLE_TIMEOUT_SECONDS:-900}" =~ ^[1-9][0-9]*$ ]] || die "HUMBLE_TIMEOUT_SECONDS must be a positive number of seconds in bounded mode"
            ;;
        1)
            [[ "${HUMBLE_TIMEOUT_SECONDS:-900}" =~ ^(0|[1-9][0-9]*)$ ]] || die "HUMBLE_TIMEOUT_SECONDS must be a non-negative number of seconds in interactive mode"
            ;;
        *) die "HUMBLE_INTERACTIVE must be 0 or 1" ;;
    esac
}

image_matches_base() {
    local runtime="$1"
    local base_ref="$2"
    local actual_ref
    actual_ref="$($runtime image inspect --format "{{ index .Config.Labels \"$image_label\" }}" "$image_name" 2>/dev/null)" || return 1
    [[ "$actual_ref" == "$base_ref" ]]
}

image_matches_toolchain() {
    local runtime="$1"
    local expected_fingerprint="$2"
    local actual_fingerprint
    actual_fingerprint="$($runtime image inspect --format "{{ index .Config.Labels \"$toolchain_image_label\" }}" "$image_name" 2>/dev/null)" || return 1
    [[ "$actual_fingerprint" == "$expected_fingerprint" ]]
}

ensure_image() {
    local runtime="$1"
    local base_ref="$2"
    local fingerprint="$3"
    if image_matches_base "$runtime" "$base_ref" && image_matches_toolchain "$runtime" "$fingerprint"; then
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
        [[ -z "${HUMBLE_GUI:-}" ]] || die "HUMBLE_GUI must be 1 when GUI forwarding is enabled"
        return
    }
    [[ "${DISPLAY:-}" == :0 ]] || die "HUMBLE_GUI requires DISPLAY=:0"
    [[ -n "${WAYLAND_DISPLAY:-}" ]] || die "HUMBLE_GUI requires WAYLAND_DISPLAY"
    [[ -n "${XDG_RUNTIME_DIR:-}" && -d "$XDG_RUNTIME_DIR" ]] || die "HUMBLE_GUI requires XDG_RUNTIME_DIR directory"
    [[ -d /tmp/.X11-unix ]] || die "HUMBLE_GUI requires /tmp/.X11-unix"
    [[ -d /mnt/wslg ]] || die "HUMBLE_GUI requires /mnt/wslg"
}
