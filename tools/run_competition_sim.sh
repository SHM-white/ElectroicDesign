#!/usr/bin/env bash
# ==============================================================================
# run_competition_sim.sh — 完整比赛流程模拟
#
# 启动热点 + 模拟桥接器 + 可选无人机任务，连接真实小车和地面站。
#
# 用法:
#   sudo ./tools/run_competition_sim.sh                    # 默认: 等待地面站选择
#   sudo ./tools/run_competition_sim.sh --auto-task 1      # 自动选择任务1
#   sudo ./tools/run_competition_sim.sh --no-drone         # 不启动无人机
#   sudo ./tools/run_competition_sim.sh --dry-run          # 无人机模拟模式
#
# 环境变量:
#   ED_HOTSPOT_SSID      热点名称 (默认: ED-UAV)
#   ED_HOTSPOT_PASSWORD   WPA2 密码 (留空=开放)
#   ED_CAR_MAC            小车 MAC 地址 (DHCP 固定 IP)
#   ED_HMI_MAC            地面站 MAC 地址 (DHCP 固定 IP)
#   ED_AUTH_KEY_FILE      HMAC 密钥文件路径
# ==============================================================================

set -Eeuo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

# ── 配置 ──────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-$PROJECT_DIR/.venv/bin/python3}"

NUC_IP="192.168.20.1"
CAR_IP="192.168.20.2"
HMI_IP="192.168.20.3"
NUC_PORT=42000

# 默认参数
AUTO_TASK=0
NO_DRONE=false
DRY_RUN=false
DURATION=300
VERBOSE=false
KEY_FILE="${ED_AUTH_KEY_FILE:-}"
HOTSPOT_SCRIPT="$SCRIPT_DIR/ed_comm.sh"
BRIDGE_SCRIPT="$SCRIPT_DIR/competition_sim_bridge.py"

# ── 颜色 ──────────────────────────────────────────────────
R='\033[0;31m'; G='\033[0;32m'; Y='\033[0;33m'; C='\033[0;36m'; B='\033[1m'; N='\033[0m'
ok()   { echo -e "${G}[OK]${N}  $*"; }
warn() { echo -e "${Y}[!!]${N}  $*"; }
fail() { echo -e "${R}[ERR]{N} $*" >&2; }
info() { echo -e "${C}[>>]${N}  $*"; }
die()  { fail "$*"; exit 1; }

# ── 参数解析 ──────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --auto-task)
            AUTO_TASK="$2"; shift 2
            [[ "$AUTO_TASK" =~ ^[012]$ ]] || die "--auto-task 必须是 0, 1 或 2"
            ;;
        --no-drone)
            NO_DRONE=true; shift
            ;;
        --dry-run)
            DRY_RUN=true; shift
            ;;
        --duration)
            DURATION="$2"; shift 2
            ;;
        --key-file)
            KEY_FILE="$2"; shift 2
            ;;
        --verbose|-v)
            VERBOSE=true; shift
            ;;
        --help|-h)
            echo "用法: sudo $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --auto-task N    自动选择任务 (0=等待地面站, 1=任务1, 2=任务2)"
            echo "  --no-drone       不启动无人机任务"
            echo "  --dry-run        无人机使用模拟模式 (--dry-run)"
            echo "  --duration SEC   最大运行时间 (默认: 300秒)"
            echo "  --key-file FILE  HMAC 密钥文件"
            echo "  --verbose        详细输出"
            echo "  --help           显示帮助"
            echo ""
            echo "环境变量:"
            echo "  ED_HOTSPOT_SSID     热点名称 (默认: ED-UAV)"
            echo "  ED_HOTSPOT_PASSWORD  WPA2 密码"
            echo "  ED_CAR_MAC           小车 MAC"
            echo "  ED_HMI_MAC           地面站 MAC"
            echo "  ED_AUTH_KEY_FILE     HMAC 密钥文件"
            exit 0
            ;;
        *)
            die "未知参数: $1 (使用 --help 查看帮助)"
            ;;
    esac
done

# ── 前置检查 ──────────────────────────────────────────────
check_prerequisites() {
    info "检查前置条件..."

    # Root 权限 (热点需要)
    if [[ $EUID -ne 0 ]]; then
        die "需要 root 权限来创建热点。请使用: sudo $0 $*"
    fi

    # Python
    [[ -x "$PYTHON" ]] || die "Python 不可执行: $PYTHON"

    # 模拟桥接器
    [[ -f "$BRIDGE_SCRIPT" ]] || die "模拟桥接器不存在: $BRIDGE_SCRIPT"

    # 热点脚本
    [[ -f "$HOTSPOT_SCRIPT" ]] || die "热点脚本不存在: $HOTSPOT_SCRIPT"

    # nmcli
    command -v nmcli >/dev/null || die "需要 NetworkManager (nmcli)"

    # ping
    command -v ping >/dev/null || die "需要 ping (iputils-ping)"

    ok "前置条件检查通过"
}

# ── 热点管理 ──────────────────────────────────────────────
ensure_hotspot() {
    info "确保热点已启动..."

    # 检查热点是否已在运行
    if nmcli -t -f NAME,TYPE connection show --active 2>/dev/null | grep -q "^ed-hotspot:"; then
        ok "热点已在运行"
    else
        # 启动热点
        bash "$HOTSPOT_SCRIPT" setup
        bash "$HOTSPOT_SCRIPT" run &
        HOTSPOT_PID=$!
        sleep 2

        if nmcli -t -f NAME,TYPE connection show --active 2>/dev/null | grep -q "^ed-hotspot:"; then
            ok "热点已启动"
        else
            die "热点启动失败"
        fi
    fi

    # 验证本机 IP
    if ip addr show 2>/dev/null | grep -q "$NUC_IP"; then
        ok "本机 IP: $NUC_IP"
    else
        warn "本机未找到 $NUC_IP，热点可能未正确配置"
    fi
}

# ── 连通性检查 ────────────────────────────────────────────
check_connectivity() {
    info "检查设备连通性..."

    # 等待设备连接
    local wait_count=0
    local max_wait=10

    while [[ $wait_count -lt $max_wait ]]; do
        local car_ok=false
        local hmi_ok=false

        if ping -c 1 -W 1 "$CAR_IP" >/dev/null 2>&1; then
            car_ok=true
        fi
        if ping -c 1 -W 1 "$HMI_IP" >/dev/null 2>&1; then
            hmi_ok=true
        fi

        if $car_ok && $hmi_ok; then
            ok "CAR ($CAR_IP) 可达"
            ok "HMI ($HMI_IP) 可达"
            return 0
        fi

        wait_count=$((wait_count + 1))
        if [[ $wait_count -eq 1 ]]; then
            info "等待设备连接..."
        fi
        sleep 2
    done

    # 部分设备不可达也继续
    if ping -c 1 -W 1 "$CAR_IP" >/dev/null 2>&1; then
        ok "CAR ($CAR_IP) 可达"
    else
        warn "CAR ($CAR_IP) 不可达 (小车可能未上电或未连接热点)"
    fi

    if ping -c 1 -W 1 "$HMI_IP" >/dev/null 2>&1; then
        ok "HMI ($HMI_IP) 可达"
    else
        warn "HMI ($HMI_IP) 不可达 (地面站可能未上电或未连接热点)"
    fi
}

# ── 端口检查 ──────────────────────────────────────────────
check_port() {
    if ss -uln 2>/dev/null | grep -q ":$NUC_PORT "; then
        warn "端口 $NUC_PORT 已被占用"
        local pid
        pid=$(ss -ulnp 2>/dev/null | grep ":$NUC_PORT " | grep -oP 'pid=\K[0-9]+' || true)
        if [[ -n "$pid" ]]; then
            warn "占用进程: PID=$pid"
            read -r -p "是否终止该进程? [y/N] " answer
            if [[ "$answer" =~ ^[Yy] ]]; then
                kill "$pid" 2>/dev/null || true
                sleep 1
                ok "进程已终止"
            else
                die "端口被占用，无法继续"
            fi
        fi
    else
        ok "端口 $NUC_PORT 可用"
    fi
}

# ── 构建模拟桥接器命令 ────────────────────────────────────
build_bridge_cmd() {
    local cmd=("$PYTHON" "$BRIDGE_SCRIPT")

    if [[ -n "$KEY_FILE" ]]; then
        cmd+=(--key-file "$KEY_FILE")
    fi

    if [[ $AUTO_TASK -gt 0 ]]; then
        cmd+=(--auto-task "$AUTO_TASK")
    fi

    cmd+=(--duration "$DURATION")

    if $VERBOSE; then
        cmd+=(--verbose)
    fi

    echo "${cmd[@]}"
}

# ── 构建无人机命令 ────────────────────────────────────────
build_drone_cmd() {
    if $NO_DRONE; then
        echo "none"
        return
    fi

    local cmd="$PYTHON -m drone.main --auto-start --verbose"

    if $DRY_RUN; then
        cmd="$cmd --dry-run --simulate-mcu"
    fi

    echo "$cmd"
}

# ── 主流程 ────────────────────────────────────────────────
main() {
    echo ""
    echo -e "${C}${B}╔══════════════════════════════════════════════════════════════════════════════╗${N}"
    echo -e "${C}${B}║                    ED UAV 完整比赛流程模拟                                   ║${N}"
    echo -e "${C}${B}╚══════════════════════════════════════════════════════════════════════════════╝${N}"
    echo ""

    # 前置检查
    check_prerequisites

    # 热点
    ensure_hotspot

    # 连通性
    check_connectivity

    # 端口
    check_port

    # 构建命令
    local bridge_cmd
    bridge_cmd=$(build_bridge_cmd)

    local drone_cmd
    drone_cmd=$(build_drone_cmd)

    echo ""
    info "启动配置:"
    echo "  热点:     ED-UAV ($NUC_IP)"
    echo "  CAR:      $CAR_IP"
    echo "  HMI:      $HMI_IP"
    echo "  自动任务: $AUTO_TASK (0=等待地面站)"
    echo "  无人机:   $drone_cmd"
    echo "  最大时间: ${DURATION}秒"
    echo ""

    # 启动模拟桥接器
    info "启动比赛流程模拟..."
    echo ""

    # 设置清理
    cleanup() {
        echo ""
        warn "正在清理..."
        if [[ -n "${HOTSPOT_PID:-}" ]]; then
            kill "$HOTSPOT_PID" 2>/dev/null || true
        fi
        ok "清理完成"
    }
    trap cleanup EXIT INT TERM

    # 运行桥接器
    if [[ "$drone_cmd" == "none" ]]; then
        eval "$bridge_cmd --no-drone"
    else
        eval "$bridge_cmd --drone-cmd '$drone_cmd'"
    fi

    local exit_code=$?

    echo ""
    if [[ $exit_code -eq 0 ]]; then
        ok "比赛流程模拟完成"
    else
        fail "比赛流程模拟异常退出 (code=$exit_code)"
    fi

    return $exit_code
}

# ── 入口 ──────────────────────────────────────────────────
main "$@"
