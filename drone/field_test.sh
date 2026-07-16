#!/usr/bin/env bash
# 场地混合硬件测试：飞控链路模拟，海康 MVS 相机和 H7 激光使用真实硬件。
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_DIR/.venv/bin/python3}"
H7_PORT="${H7_PORT:-/dev/serial/by-path/pci-0000:00:14.0-usb-0:7:1.0-port0}"
CAMERA_ID="${CAMERA_ID:-0}"
CAMERA_EXPOSURE_MS="${CAMERA_EXPOSURE_MS:-20}"
CAMERA_GAIN="${CAMERA_GAIN:-8}"
PREVIEW_WIDTH="${PREVIEW_WIDTH:-720}"
PROFILE="${PROFILE:-debug}"
IMAGE_REPLAY="${IMAGE_REPLAY:-0}"
IMAGE_DIR="${IMAGE_DIR:-$PROJECT_DIR}"
SWITCH_SECONDS="${SWITCH_SECONDS:-0.1}"

fail() { printf '\033[0;31m[ERROR]\033[0m %s\n' "$*" >&2; exit 1; }
info() { printf '\033[0;32m[INFO]\033[0m  %s\n' "$*"; }

[[ -x "$PYTHON" ]] || fail "Python 不可执行: $PYTHON"

if [[ "$IMAGE_REPLAY" == "1" ]]; then
    [[ -d "$IMAGE_DIR" ]] || fail "图片目录不存在: $IMAGE_DIR"
    info "场地图片回放模式：目录=$IMAGE_DIR，每 ${SWITCH_SECONDS} 秒切换一张"
    info "逐张执行现有 OCR 并使用原任务 UI 显示；按 q/ESC 可提前退出"
    cd "$PROJECT_DIR"
    exec "$PYTHON" -m drone.test_vision \
        --image-dir "$IMAGE_DIR" \
        --switch-seconds "$SWITCH_SECONDS"
fi

[[ -e "$H7_PORT" ]] || fail "H7 桥接板串口不存在: $H7_PORT"

info "场地测试模式：飞控=模拟，MVS相机=真实，H7激光=真实"
info "MVS索引=$CAMERA_ID 曝光=${CAMERA_EXPOSURE_MS}ms 增益=${CAMERA_GAIN}dB"
info "X11预览宽度=${PREVIEW_WIDTH}px（识别仍使用相机原始分辨率）"
info "H7串口=$H7_PORT；状态机启动后立即开始，人工移动并由目标数字OCR确认到达"
printf '\033[1;33m[WARN]\033[0m  激光会由任务状态机自动控制；请勿直视并确保光路安全。\n'

cd "$PROJECT_DIR"
exec "$PYTHON" -m drone.main \
    --simulate-mcu \
    --auto-start \
    --manual-navigation \
    --profile "$PROFILE" \
    --vision-backend mvs \
    --camera-id "$CAMERA_ID" \
    --camera-exposure-ms "$CAMERA_EXPOSURE_MS" \
    --camera-gain "$CAMERA_GAIN" \
    --vision-preview \
    --preview-width "$PREVIEW_WIDTH" \
    --h7-serial "$H7_PORT" \
    --verbose
