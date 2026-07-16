#!/usr/bin/env bash
# 正式启动：真实飞控、海康 MVS 相机和 H7 激光；等待 AUX6 启动任务。
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_DIR/.venv/bin/python3}"
MCU_PORT="${MCU_PORT:-}"
H7_PORT="${H7_PORT:-/dev/serial/by-path/pci-0000:00:14.0-usb-0:7:1.0-port0}"
CAMERA_ID="${CAMERA_ID:-0}"
CAMERA_EXPOSURE_MS="${CAMERA_EXPOSURE_MS:-50}"
CAMERA_GAIN="${CAMERA_GAIN:-8}"
PROFILE="${PROFILE:-competition}"
VISION_PREVIEW="${VISION_PREVIEW:-0}"

fail() { printf '\033[0;31m[ERROR]\033[0m %s\n' "$*" >&2; exit 1; }
info() { printf '\033[0;32m[INFO]\033[0m  %s\n' "$*"; }

[[ -x "$PYTHON" ]] || fail "Python 不可执行: $PYTHON"
[[ -n "$MCU_PORT" ]] || fail "必须显式指定飞控串口，例如 MCU_PORT=/dev/serial/by-id/... ./drone/start.sh"
[[ -e "$MCU_PORT" ]] || fail "飞控串口不存在: $MCU_PORT"
[[ -e "$H7_PORT" ]] || fail "H7 桥接板串口不存在: $H7_PORT"
[[ "$(readlink -f "$MCU_PORT")" != "$(readlink -f "$H7_PORT")" ]] || fail "飞控与H7不能使用同一串口"

ARGS=(
    --profile "$PROFILE"
    --vision-backend mvs
    --camera-id "$CAMERA_ID"
    --camera-exposure-ms "$CAMERA_EXPOSURE_MS"
    --camera-gain "$CAMERA_GAIN"
    --serial-port "$MCU_PORT"
    --h7-serial "$H7_PORT"
    --verbose
)
[[ "$VISION_PREVIEW" == "1" ]] && ARGS+=(--vision-preview)

info "正式模式：飞控、MVS相机、H7激光均为真实硬件"
info "飞控=$MCU_PORT H7=$H7_PORT MVS索引=$CAMERA_ID"
info "初始化完成后等待 AUX6 > 1700us 启动，不会自动开始任务"
printf '\033[1;33m[WARN]\033[0m  请确认桨叶、人员、遥控器和紧急停机方案均已就绪。\n'
read -r -p "输入 START 确认启动程序: " confirm
[[ "$confirm" == "START" ]] || { info "已取消"; exit 0; }

cd "$PROJECT_DIR"
exec "$PYTHON" -m drone.main "${ARGS[@]}"
