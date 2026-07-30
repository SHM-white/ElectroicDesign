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

v4l2_args() {
    local requested_devices="${HUMBLE_V4L2_DEVICES:-}"
    local device_root="${HUMBLE_V4L2_DEVICE_ROOT:-}"
    local stable_path
    local host_stable_path
    local link_target
    local host_resolved_path
    local resolved_path
    local by_id_source="/dev/v4l/by-id"
    local -a stable_paths=()
    local -A seen_stable_paths=()
    local -A seen_resolved_paths=()

    v4l2_run_args=()
    [[ -n "$requested_devices" ]] || {
        [[ -z "$device_root" ]] || require_test_hook HUMBLE_V4L2_DEVICE_ROOT
        return
    }
    command -v realpath >/dev/null 2>&1 || die "realpath is required for V4L2 device forwarding"
    if [[ -n "$device_root" ]]; then
        require_test_hook HUMBLE_V4L2_DEVICE_ROOT
        [[ "$device_root" == /* && -d "$device_root/dev" ]] || die "HUMBLE_V4L2_DEVICE_ROOT must contain a dev directory"
        device_root="${device_root%/}"
        by_id_source="$device_root/dev/v4l/by-id"
    fi
    [[ -d "$by_id_source" ]] || die "V4L2 by-id directory '$by_id_source' does not exist"

    mapfile -t stable_paths <<<"$requested_devices"
    ((${#stable_paths[@]} > 0)) || die "HUMBLE_V4L2_DEVICES must contain at least one path"
    v4l2_run_args=(--volume "$by_id_source:/dev/v4l/by-id:ro")
    for stable_path in "${stable_paths[@]}"; do
        [[ "$stable_path" =~ ^/dev/v4l/by-id/[^/[:space:]]+$ ]] || die "V4L2 device '$stable_path' must be a stable /dev/v4l/by-id path"
        [[ -z "${seen_stable_paths[$stable_path]:-}" ]] || die "duplicate V4L2 by-id path '$stable_path'"
        seen_stable_paths[$stable_path]=1
        host_stable_path="$device_root$stable_path"
        [[ -L "$host_stable_path" ]] || die "V4L2 by-id path '$stable_path' does not exist or is not a symbolic link"
        link_target="$(readlink "$host_stable_path")"
        if [[ "$link_target" == /* ]]; then
            resolved_path="$(realpath --canonicalize-missing --no-symlinks "$link_target")"
            host_resolved_path="$device_root$resolved_path"
        else
            host_resolved_path="$(realpath --canonicalize-missing --no-symlinks "$(dirname "$host_stable_path")/$link_target")"
            resolved_path="${host_resolved_path#"$device_root"}"
        fi
        [[ "$resolved_path" =~ ^/dev/video[0-9]+$ ]] || die "V4L2 by-id path '$stable_path' must resolve to /dev/videoN, got '$resolved_path'"
        [[ -c "$host_resolved_path" ]] || die "resolved V4L2 path '$resolved_path' is not a character device"
        [[ -z "${seen_resolved_paths[$resolved_path]:-}" ]] || die "V4L2 by-id paths resolve to duplicate device '$resolved_path'"
        seen_resolved_paths[$resolved_path]=1
        v4l2_run_args+=(--device "$resolved_path:$resolved_path")
    done
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
