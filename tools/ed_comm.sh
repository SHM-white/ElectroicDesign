#!/usr/bin/env bash
# ==============================================================================
# ED UAV 一键通信工具
# 功能：启动热点 + 实时诊断三端通信（CAR/HMI/ROS）
#
# 用法：
#   sudo ./tools/ed_comm.sh              # 启动热点 + 运行诊断（默认）
#   sudo ./tools/ed_comm.sh setup        # 仅创建/更新热点配置
#   sudo ./tools/ed_comm.sh stop         # 关闭热点
#   sudo ./tools/ed_comm.sh status       # 查看热点状态
#   sudo ./tools/ed_comm.sh test         # 测试网络连通性
# ==============================================================================

set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

# ─── 配置 ───────────────────────────────────────────────────────────────────
CON_NAME="ed-hotspot"
SSID="${ED_HOTSPOT_SSID:-ED-UAV}"
PASSWORD="${ED_HOTSPOT_PASSWORD:-}"
CHANNEL="${ED_HOTSPOT_CHANNEL:-6}"
BAND="${ED_HOTSPOT_BAND:-bg}"
IFACE="${ED_HOTSPOT_IFACE:-}"
STA_IFACE=""

NUC_IP="192.168.20.1"
CAR_IP="192.168.20.2"
HMI_IP="192.168.20.3"
SUBNET="192.168.20.0/24"

CAR_MAC="${ED_CAR_MAC:-}"
HMI_MAC="${ED_HMI_MAC:-}"

DNSMASQ_CONF="/etc/NetworkManager/dnsmasq-shared.d/ed-hotspot.conf"

# ─── 颜色 ───────────────────────────────────────────────────────────────────
R='\033[0;31m'; G='\033[0;32m'; Y='\033[0;33m'; C='\033[0;36m'; B='\033[1m'; N='\033[0m'
ok()   { echo -e "${G}[OK]${N}  $*"; }
warn() { echo -e "${Y}[!!]${N}  $*"; }
fail() { echo -e "${R}[ERR]${N} $*" >&2; }
die()  { fail "$*"; exit 1; }

# ─── 前置检查 ───────────────────────────────────────────────────────────────
check_root()  { [[ $EUID -eq 0 ]] || die "需要 root: sudo $0 $*"; }
check_tools() {
    command -v nmcli   >/dev/null || die "需要 NetworkManager (nmcli)"
    command -v python3 >/dev/null || die "需要 python3"
    command -v ping    >/dev/null || die "需要 ping (iputils-ping)"
}

detect_iface() {
    [[ -n "$IFACE" ]] && return
    local -a wifi_ifaces
    mapfile -t wifi_ifaces < <(nmcli -t -f DEVICE,TYPE device status 2>/dev/null | grep ':wifi$' | cut -d: -f1)

    if [[ ${#wifi_ifaces[@]} -eq 0 ]]; then
        die "未检测到无线网卡。请插入 USB 无线网卡或设置 ED_HOTSPOT_IFACE 手动指定。"
    fi

    if [[ ${#wifi_ifaces[@]} -ge 2 ]]; then
        # 多个无线接口：已连接的做 STA（互联网），另一个做 AP（热点）
        ok "检测到 ${#wifi_ifaces[@]} 个无线接口: ${wifi_ifaces[*]}"
        local default_route_dev
        default_route_dev=$(ip route 2>/dev/null | awk '/^default/{print $5; exit}')
        for iface in "${wifi_ifaces[@]}"; do
            local state
            state=$(nmcli -t -f DEVICE,STATE device status 2>/dev/null | grep "^${iface}:" | cut -d: -f2)
            if [[ "$state" == "已连接" || "$state" == "connected" ]]; then
                # 有默认路由的接口优先做 STA（互联网出口）
                if [[ "$iface" == "$default_route_dev" ]]; then
                    STA_IFACE="$iface"
                    ok "  $iface → STA (互联网，默认路由)"
                elif [[ -z "$STA_IFACE" ]]; then
                    STA_IFACE="$iface"
                    ok "  $iface → STA (互联网)"
                else
                    IFACE="$iface"
                    ok "  $iface → AP (热点)"
                fi
            else
                IFACE="$iface"
                ok "  $iface → AP (热点)"
            fi
        done
        # 兜底：所有接口都已连接且仍未确定 AP，取最后一个非 STA 接口
        if [[ -z "$IFACE" && -n "$STA_IFACE" ]]; then
            for iface in "${wifi_ifaces[@]}"; do
                if [[ "$iface" != "$STA_IFACE" ]]; then
                    IFACE="$iface"
                    ok "  $iface → AP (热点，兜底选择)"
                    break
                fi
            done
        fi
        [[ -n "$IFACE" ]] || die "无法确定 AP 接口"
    else
        IFACE="${wifi_ifaces[0]}"
        warn "仅检测到 1 个无线接口 ($IFACE)，热点会断开当前 Wi-Fi"
    fi
}

# ─── 热点管理 ───────────────────────────────────────────────────────────────
hotspot_is_active() {
    nmcli -t -f NAME,TYPE connection show --active 2>/dev/null | grep -q "^${CON_NAME}:"
}

hotspot_ensure() {
    # 如果热点已在运行，直接返回
    if hotspot_is_active; then
        ok "热点已在运行"
        return 0
    fi

    # 如果连接配置不存在，先创建
    if ! nmcli connection show "$CON_NAME" &>/dev/null; then
        hotspot_create
    fi

    # 仅在单网卡模式下断开当前无线连接
    if [[ -z "$STA_IFACE" ]]; then
        local active
        active=$(nmcli -t -f NAME,DEVICE,TYPE connection show --active 2>/dev/null \
            | grep ":${IFACE}:802-11-wireless" | head -1 | cut -d: -f1 || true)
        if [[ -n "$active" && "$active" != "$CON_NAME" ]]; then
            warn "断开当前无线: $active"
            nmcli connection down "$active" 2>/dev/null || true
            sleep 1
        fi
    else
        ok "双网卡模式: $STA_IFACE 保持互联网连接"
    fi

    # 启动热点
    nmcli connection up "$CON_NAME" >/dev/null 2>&1 || die "启动热点失败"
    sleep 1

    if ip addr show "$IFACE" 2>/dev/null | grep -q "$NUC_IP"; then
        ok "热点已启动: $IFACE = $NUC_IP"
    else
        warn "热点已启动，但 $IFACE 上未找到 $NUC_IP"
    fi
}

hotspot_create() {
    echo -e "${C}创建热点配置 ...${N}"

    nmcli connection delete "$CON_NAME" 2>/dev/null || true

    local -a args=(
        connection add type wifi ifname "$IFACE" con-name "$CON_NAME"
        ssid "$SSID"
        802-11-wireless.mode ap
        802-11-wireless.band "$BAND"
        802-11-wireless.channel "$CHANNEL"
        ipv4.method shared
        ipv4.addresses "${NUC_IP}/24"
        ipv4.never-default yes
        ipv6.method disabled
        connection.autoconnect yes
        connection.zone trusted
    )

    if [[ -n "$PASSWORD" ]]; then
        [[ ${#PASSWORD} -ge 8 ]] || die "WPA2 密码至少 8 字符"
        args+=(
            802-11-wireless-security.key-mgmt wpa-psk
            802-11-wireless-security.psk "$PASSWORD"
        )
    fi

    nmcli "${args[@]}" || die "创建连接失败"
    ok "连接配置已创建"

    # 设置 WPA2 明确参数（ESP32 兼容）
    nmcli connection modify "$CON_NAME" 802-11-wireless-security.proto rsn 2>/dev/null || true
    nmcli connection modify "$CON_NAME" 802-11-wireless-security.pairwise ccmp 2>/dev/null || true
    nmcli connection modify "$CON_NAME" 802-11-wireless-security.group ccmp 2>/dev/null || true

    # IP 转发
    echo "net.ipv4.ip_forward = 1" > /etc/sysctl.d/99-ed-hotspot.conf
    sysctl -p /etc/sysctl.d/99-ed-hotspot.conf >/dev/null 2>&1

    # NAT（如果有线出口）
    local wan
    wan=$(ip route | awk '/^default/{print $5; exit}')
    if [[ -n "$wan" && "$wan" != "$IFACE" ]]; then
        iptables -t nat -C POSTROUTING -s "$SUBNET" -o "$wan" -j MASQUERADE 2>/dev/null \
            || iptables -t nat -A POSTROUTING -s "$SUBNET" -o "$wan" -j MASQUERADE
    fi

    # 客户端互通
    iptables -C FORWARD -i "$IFACE" -o "$IFACE" -j ACCEPT 2>/dev/null \
        || iptables -A FORWARD -i "$IFACE" -o "$IFACE" -j ACCEPT

    # 允许热点子网的 UDP/TCP 入站（ESP32 通讯需要）
    iptables -C INPUT -i "$IFACE" -p udp -s "$SUBNET" -j ACCEPT 2>/dev/null \
        || iptables -I INPUT 1 -i "$IFACE" -p udp -s "$SUBNET" -j ACCEPT
    iptables -C INPUT -i "$IFACE" -p tcp -s "$SUBNET" -j ACCEPT 2>/dev/null \
        || iptables -I INPUT 2 -i "$IFACE" -p tcp -s "$SUBNET" -j ACCEPT

    # DHCP 静态绑定
    mkdir -p "$(dirname "$DNSMASQ_CONF")"
    cat > "$DNSMASQ_CONF" <<EOF
interface=${IFACE}
bind-interfaces
dhcp-range=192.168.20.10,192.168.20.50,255.255.255.0,12h
dhcp-option=option:router,${NUC_IP}
dhcp-option=option:dns-server,${NUC_IP}
EOF
    [[ -n "$CAR_MAC" ]] && echo "dhcp-host=${CAR_MAC},${CAR_IP},ed-car,infinite" >> "$DNSMASQ_CONF"
    [[ -n "$HMI_MAC" ]] && echo "dhcp-host=${HMI_MAC},${HMI_IP},ed-hmi,infinite" >> "$DNSMASQ_CONF"

    # /etc/hosts
    sed -i '/# ed-comm$/d' /etc/hosts 2>/dev/null || true
    cat >> /etc/hosts <<EOF
${NUC_IP}  ed-nuc  # ed-comm
${CAR_IP}  ed-car  # ed-comm
${HMI_IP}  ed-hmi  # ed-comm
EOF

    systemctl reload NetworkManager 2>/dev/null || true
    ok "热点配置完成"
}

hotspot_stop() {
    nmcli connection down "$CON_NAME" 2>/dev/null && ok "热点已关闭" || warn "热点未在运行"
}

hotspot_status() {
    echo ""
    echo -e "${C}┌── ED UAV 热点状态 ──────────────────────────┐${N}"

    if hotspot_is_active; then
        echo -e "  状态:    ${G}● 运行中${N}"
    else
        echo -e "  状态:    ${R}○ 未启动${N}"
    fi

    if ip addr show "$IFACE" 2>/dev/null | grep -q "$NUC_IP"; then
        echo -e "  接口:    ${G}${IFACE}${N} = ${NUC_IP}"
    else
        echo -e "  接口:    ${Y}${IFACE}${N} (无 IP)"
    fi

    echo -e "  SSID:    ${SSID}"
    echo -e "  转发:    $([ "$(cat /proc/sys/net/ipv4/ip_forward)" = "1" ] && echo -e "${G}已启用${N}" || echo -e "${R}未启用${N}")"
    echo -e "  自启:    $(nmcli -t -f AUTOCONNECT connection show "$CON_NAME" 2>/dev/null | cut -d: -f1 | grep -q yes && echo -e "${G}已启用${N}" || echo -e "${Y}未启用${N}")"

    # DHCP
    if [[ -f "$DNSMASQ_CONF" ]]; then
        echo -e "  DHCP:    ${G}已配置${N}"
        grep 'dhcp-host=' "$DNSMASQ_CONF" 2>/dev/null | sed 's/^/           /' || true
    fi

    # 客户端
    echo ""
    echo -e "  ${C}已连接客户端:${N}"
    ip neigh show dev "$IFACE" 2>/dev/null | grep -v FAILED | while read -r line; do
        local ip_addr mac
        ip_addr=$(echo "$line" | awk '{print $1}')
        mac=$(echo "$line" | awk '{print $5}')
        local tag=""
        [[ "$ip_addr" == "$CAR_IP" ]] && tag=" ${B}← CAR${N}"
        [[ "$ip_addr" == "$HMI_IP" ]] && tag=" ${B}← HMI${N}"
        echo "    ${ip_addr}  ${mac}${tag}"
    done || echo "    (无)"

    echo -e "${C}└──────────────────────────────────────────────┘${N}"
    echo ""
}

hotspot_test() {
    echo ""
    echo -e "${C}┌── 网络连通性测试 ─────────────────────────────┐${N}"

    ip addr show "$IFACE" 2>/dev/null | grep -q "$NUC_IP" || die "热点未启动"
    echo -e "  [OK] 接口就绪 ($NUC_IP)"

    [[ "$(cat /proc/sys/net/ipv4/ip_forward)" == "1" ]] \
        && echo -e "  [OK] IP 转发" \
        || echo -e "  [${R}FAIL${N}] IP 转发未启用"

    echo ""
    echo -e "  ${C}探测客户端 ...${N}"
    ping -c 1 -b -W 1 "$SUBNET" >/dev/null 2>&1 || true
    sleep 1

    if ping -c 1 -W 2 "$CAR_IP" >/dev/null 2>&1; then
        echo -e "  [OK] CAR ($CAR_IP) ${G}可达${N}"
    else
        echo -e "  [--] CAR ($CAR_IP) ${Y}未响应${N}"
    fi

    if ping -c 1 -W 2 "$HMI_IP" >/dev/null 2>&1; then
        echo -e "  [OK] HMI ($HMI_IP) ${G}可达${N}"
    else
        echo -e "  [--] HMI ($HMI_IP) ${Y}未响应${N}"
    fi

    if ss -uln 2>/dev/null | grep -q ":42000 "; then
        echo -e "  [--] 端口 42000 ${Y}已占用${N}（ROS bridge？）"
    else
        echo -e "  [OK] 端口 42000 ${G}可用${N}"
    fi

    echo -e "${C}└───────────────────────────────────────────────┘${N}"
    echo ""
}

# ─── 诊断工具 ───────────────────────────────────────────────────────────────
run_diagnostic() {
    local diag_script="tools/diagnostics/vehicle_comm_diagnostic.py"
    [[ -f "$diag_script" ]] || die "诊断工具不存在: $diag_script"

    echo ""
    echo -e "${G}启动通信诊断 ...${N}  (Ctrl+C 退出)"
    echo ""

    # 优雅退出：关闭热点可选
    cleanup() {
        echo ""
        echo -e "${Y}诊断已停止${N}"
    }
    trap cleanup EXIT

    python3 "$diag_script" "$@"
}

# ─── 主入口 ─────────────────────────────────────────────────────────────────
main() {
    local cmd="${1:-run}"

    # help 不需要任何检查
    case "$cmd" in
        help|-h|--help)
            echo "用法: sudo $0 [命令]"
            echo ""
            echo "命令:"
            echo "  run    启动热点 + 运行诊断（默认）"
            echo "  setup  仅创建/更新热点配置"
            echo "  stop   关闭热点"
            echo "  status 查看热点状态"
            echo "  test   测试网络连通性"
            echo ""
            echo "环境变量:"
            echo "  ED_HOTSPOT_SSID      热点名称 (默认: ED-UAV)"
            echo "  ED_HOTSPOT_PASSWORD   WPA2 密码 (留空=开放)"
            echo "  ED_HOTSPOT_CHANNEL    信道 (默认: 6)"
            echo "  ED_HOTSPOT_IFACE      无线接口 (自动检测)"
            echo "  ED_CAR_MAC            小车 MAC (DHCP 固定 .2)"
            echo "  ED_HMI_MAC            地面站 MAC (DHCP 固定 .3)"
            exit 0
            ;;
    esac

    check_tools
    detect_iface

    case "$cmd" in
        run)
            check_root
            hotspot_ensure
            run_diagnostic "${@:2}"
            ;;
        setup)
            check_root
            hotspot_create
            hotspot_ensure
            hotspot_status
            ;;
        stop)
            check_root
            hotspot_stop
            ;;
        status)
            hotspot_status
            ;;
        test)
            check_root
            hotspot_test
            ;;
        *)
            die "未知命令: $cmd (运行 $0 help 查看帮助)"
            ;;
    esac
}

main "$@"
