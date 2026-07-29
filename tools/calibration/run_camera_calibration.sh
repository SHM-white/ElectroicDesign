#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
readonly width="${CAMERA_CALIBRATION_WIDTH:-1280}"
readonly height="${CAMERA_CALIBRATION_HEIGHT:-720}"
readonly square_mm="${CAMERA_CALIBRATION_SQUARE_MM:-15.0}"

role_choice="${1:-}"
if (($# > 1)); then
    printf 'Usage: %s [1|2]\n' "$0" >&2
    exit 2
fi
if [[ -z "$role_choice" ]]; then
    printf 'Camera [1=normal view, 2=wide angle]: ' >&2
    if ! read -r role_choice; then
        printf '\nStandard input is unavailable. Run: run_camera_calibration.sh [1|2]\n' >&2
        exit 2
    fi
fi
case "$role_choice" in
    1) readonly role="narrow" ;;
    2) readonly role="wide" ;;
    *)
        printf 'Camera choice must be 1 (normal view) or 2 (wide angle).\n' >&2
        exit 2
        ;;
esac

printf 'Starting %s camera calibration at %sx%s.\n' "$role" "$width" "$height"

if [[ ! -d /dev/v4l/by-id ]] || ! compgen -G '/dev/v4l/by-id/*-video-index0' >/dev/null; then
    printf 'No stable V4L2 camera found under /dev/v4l/by-id.\n' >&2
    exit 1
fi

readonly run_id="$(date -u +%Y%m%dT%H%M%SZ)"
readonly default_output="$repo_root/calibration_data/${role}_${width}x${height}_$run_id"
readonly output_dir="$default_output"

export PYTHONPATH="$repo_root/ros2_ws/src/ed_uav_camera${PYTHONPATH:+:$PYTHONPATH}"

printf 'Output directory: %s\n' "$output_dir"

exec python3 "$repo_root/tools/calibration/calibrate_chessboard.py" \
    --role "$role" \
    --width "$width" \
    --height "$height" \
    --confirm-square-mm "$square_mm" \
    --output-dir "$output_dir"
