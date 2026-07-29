#!/usr/bin/env bash
# ==============================================================================
# debug_start.sh — 无人机调试启动脚本
# G_植保飞行器 项目 | x86 迷你主机 + 凌霄飞控 + 海康工业相机 + H7 GPIO
#
# 用法:
#   chmod +x debug_start.sh
#   ./debug_start.sh                  # 默认: dry-run 模拟, 控制台日志
#   ./debug_start.sh --real           # 真实硬件飞行 (⚠ 拆桨测试!)
#   ./debug_start.sh --no-camera      # 不启用相机, 仅测试飞控链路
#   ./debug_start.sh --tuning         # tuning 速度档位
#   ./debug_start.sh --competition    # competition 速度档位
#   ./debug_start.sh --test           # 仅运行单元测试
#   ./debug_start.sh --check          # 硬件检测 (不飞行)
#   ./debug_start.sh --help           # 显示帮助
#
# 硬件连接:
#   /dev/ttyUSB0 → STM32F4 MCU (凌霄飞控, 115200bps)
#   /dev/ttyUSB1 → STM32H7/F4 GPIO (激光头 01脚, 115200bps)
#   USB直连       → 海康工业相机 (UVC, OpenCV VideoCapture)
# ==============================================================================

set -e

# ── 颜色输出 ──────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }
step()  { echo -e "${BLUE}[STEP]${NC} $*"; }

# ── 配置变量 ──────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
PYTHON="python3"

# 硬件串口 (根据实际插入顺序调整)
MCU_PORT="${MCU_PORT:-/dev/ttyUSB0}"       # 飞控MCU
H7_PORT="${H7_PORT:-/dev/ttyUSB1}"         # H7 GPIO 激光板
OPENMV_PORT="${OPENMV_PORT:-/dev/ttyUSB2}"  # OpenMV (本方案不用)
CAMERA_ID="${CAMERA_ID:-0}"                 # 海康工业相机设备ID

# ── 默认参数 ──────────────────────────────────────────
DRY_RUN="--dry-run"
PROFILE="--profile debug"
VISION_BACKEND="--vision-backend industrial"
NO_CAMERA=""
AUTO_START=""
VERBOSE="--verbose"
NO_SAVE_LOGS=""

# ── 帮助 ──────────────────────────────────────────────
show_help() {
    echo ""
    echo "══════════════════════════════════════════════════════════════"
    echo "  G_植保飞行器 — 调试启动脚本"
    echo "══════════════════════════════════════════════════════════════"
    echo ""
    echo "用法:  ./debug_start.sh [选项]"
    echo ""
    echo "选项:"
    echo "  --real          真实硬件飞行模式 (⚠ 拆桨测试!)"
    echo "                  关闭 dry-run, 启用自动启动"
    echo ""
    echo "  --no-camera     不启用视觉识别"
    echo "                  仅测试飞控通信链路与状态机"
    echo ""
    echo "  --no-laser      不初始化激光输出"
    echo "                  仅测试飞控+相机, 不控制 H7 GPIO"
    echo ""
    echo "  --openmv        使用 OpenMV 识别方案 (非默认)"
    echo "                  通过串口接收 OpenMV 板端识别结果"
    echo ""
    echo "  --tuning        使用 tuning 速度档位 (30cm/s)"
    echo "  --competition   使用 competition 速度档位 (45cm/s)"
    echo ""
    echo "  --test          仅运行单元测试 (不连接硬件)"
    echo "  --check         硬件检测 (扫描串口/相机/依赖)"
    echo "  --help          显示此帮助"
    echo ""
    echo "串口环境变量 (可临时覆盖):"
    echo "  MCU_PORT=${MCU_PORT}"
    echo "  H7_PORT=${H7_PORT}"
    echo "  CAMERA_ID=${CAMERA_ID}"
    echo ""
    echo "示例:"
    echo "  ./debug_start.sh                          # 默认 dry-run 调试"
    echo "  ./debug_start.sh --real --tuning           # 真实飞行 tuning 档"
    echo "  ./debug_start.sh --check                   # 仅硬件检测"
    echo "  MCU_PORT=/dev/ttyUSB2 ./debug_start.sh     # 临时改MCU串口"
    echo ""
    echo "⚠  无人机涉及安全风险, 请务必:"
    echo "   1. 先用 --check 确认硬件"
    echo "   2. 先用默认 dry-run 模式验证"
    echo "   3. 拆桨测试确认传感器和执行器"
    echo "   4. 在空旷场地, 确保紧急停机方案就绪"
    echo ""
}

# ── 参数解析 ──────────────────────────────────────────
REAL_MODE=false
CHECK_ONLY=false
TEST_ONLY=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --real)
            REAL_MODE=true
            DRY_RUN=""
            AUTO_START="--auto-start"
            shift
            ;;
        --no-camera)
            NO_CAMERA="--no-camera"
            shift
            ;;
        --no-laser)
            NO_LASER="--no-laser"
            shift
            ;;
        --openmv)
            VISION_BACKEND="--vision-backend openmv"
            shift
            ;;
        --tuning)
            PROFILE="--profile tuning"
            shift
            ;;
        --competition)
            PROFILE="--profile competition"
            shift
            ;;
        --test)
            TEST_ONLY=true
            shift
            ;;
        --check)
            CHECK_ONLY=true
            shift
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            error "未知选项: $1"
            echo "使用 --help 查看帮助"
            exit 1
            ;;
    esac
done

# ── 打印启动横幅 ──────────────────────────────────────
echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  G_植保飞行器 — 调试启动"
echo "══════════════════════════════════════════════════════════════"
echo "  脚本目录:   $SCRIPT_DIR"
echo "  Python:     $($PYTHON --version 2>&1)"
echo "  系统:       $(lsb_release -ds 2>/dev/null || uname -o)"
echo "  架构:       $(uname -m)"
echo ""
echo "  运行模式:   $([ "$REAL_MODE" = true ] && echo -e "${RED}真实飞行⚠${NC}" || echo -e "${GREEN}dry-run 模拟${NC}")"
echo "  速度档位:   ${PROFILE#--profile }"
echo "  视觉后端:   ${VISION_BACKEND#--vision-backend }"
echo "  相机:       $([ -n "$NO_CAMERA" ] && echo '禁用' || echo '启用')"
echo "  激光:       $([ -n "$NO_LASER" ] && echo '禁用' || echo '启用 (H7 GPIO 01脚)')"
echo "══════════════════════════════════════════════════════════════"
echo ""

# ── 单元测试模式 ──────────────────────────────────────
if [ "$TEST_ONLY" = true ]; then
    step "运行单元测试..."
    cd "$PROJECT_DIR/test"
    $PYTHON test_all.py
    exit $?
fi

# ── 硬件检测 ──────────────────────────────────────────
do_hardware_check() {
    local ok=0
    local fail=0

    echo ""
    step "=== 硬件检测 ==="

    # 1. Python 依赖
    echo ""
    info "--- Python 依赖 ---"
    for mod in serial cv2 numpy pytesseract; do
        if $PYTHON -c "import ${mod}" 2>/dev/null; then
            info "  ✅ ${mod}"
            ok=$((ok + 1))
        else
            error "  ❌ ${mod} 未安装"
            fail=$((fail + 1))
        fi
    done

    # 2. 串口设备
    echo ""
    info "--- 串口设备 ---"
    for port in "$MCU_PORT" "$H7_PORT"; do
        if [ -e "$port" ]; then
            info "  ✅ $port 存在"
            ok=$((ok + 1))
        else
            warn "  ⚠  $port 未检测到 (可能是没插设备或设备名不同)"
            fail=$((fail + 1))
        fi
    done
    # 列出所有 ttyUSB
    if ls /dev/ttyUSB* 2>/dev/null >/dev/null; then
        echo ""
        info "当前插入的 USB 串口:"
        ls -la /dev/ttyUSB* 2>/dev/null | while read line; do
            echo "    $line"
        done
    fi

    # 3. 视频设备
    echo ""
    info "--- 视频设备 ---"
    if command -v v4l2-ctl &> /dev/null; then
        v4l2-ctl --list-devices 2>/dev/null || warn "  未检测到视频设备"
    fi
    if $PYTHON -c "
import cv2
cap = cv2.VideoCapture($CAMERA_ID)
if cap.isOpened():
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f'  ✅ 相机 $CAMERA_ID 可用: {int(w)}x{int(h)}')
    cap.release()
else:
    print(f'  ⚠  相机 $CAMERA_ID 不可用')
" 2>/dev/null; then
        :
    fi

    echo ""
    info "--- 汇总 ---"
    echo "  通过: $ok 项"
    if [ $fail -gt 0 ]; then
        warn "  未通过: $fail 项"
    fi
    echo ""
}

# 总是先执行快速检测
do_hardware_check

# 纯检测模式就退出
if [ "$CHECK_ONLY" = true ]; then
    info "硬件检测完成, 退出。"
    exit 0
fi

# ── 安全确认 (真实飞行) ──────────────────────────────
if [ "$REAL_MODE" = true ]; then
    echo ""
    warn "══════════════════════════════════════════════════"
    warn "  ⚠⚠⚠  你将进入真实飞行模式!  ⚠⚠⚠"
    warn ""
    warn "  请确认以下事项:"
    warn "    ✅ 已拆桨完成传感器/执行器测试"
    warn "    ✅ 紧急停机方案已就绪 (遥控器/拔电池)"
    warn "    ✅ 飞行区域空旷无人员"
    warn "    ✅ 电池电压正常"
    warn "    ✅ 串口连接正确 (MCU:$MCU_PORT, H7:$H7_PORT)"
    warn "══════════════════════════════════════════════════"
    echo ""
    read -r -p "  输入 YES 确认继续: " confirm
    if [ "$confirm" != "YES" ]; then
        warn "已取消。"
        exit 0
    fi
    echo ""
    info "安全确认通过, 启动飞行..."
fi

# ── 构建启动参数 ──────────────────────────────────────
ARGS=(
    $PROFILE
    $DRY_RUN
    $VERBOSE
    $VISION_BACKEND
    $NO_CAMERA
    $NO_LASER
    $AUTO_START
    --serial-port "$MCU_PORT"
    --h7-serial "$H7_PORT"
)

# dry-run 下不挂载真实激光串口
if [ -n "$DRY_RUN" ]; then
    # dry-run 模式下不传 --h7-serial, 使用 dummy 激光
    ARGS=("${ARGS[@]/--h7-serial/}")
    ARGS=("${ARGS[@]/$H7_PORT/}")
    # 清理空参数
    ARGS=($(printf '%s\n' "${ARGS[@]}" | grep -v '^$' || true))
fi

echo ""
step "启动参数:"
echo "  $PYTHON $PROJECT_DIR/main.py ${ARGS[*]}"
echo ""

# ── 启动 ──────────────────────────────────────────────
cd "$PROJECT_DIR"
exec $PYTHON main.py "${ARGS[@]}"
