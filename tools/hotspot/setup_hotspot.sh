#!/usr/bin/env bash
# ==============================================================================
# ED UAV 热点一键管理工具
# 用途：在机载 x86 NUC (Ubuntu 22.04+) 上创建、管理离线 Wi-Fi 热点
# 拓扑：NUC (AP) 192.168.20.1 ← Wi-Fi → CAR (STA) .2 / HMI (STA) .3
#
# 用法：
#   sudo ./setup_hotspot.sh create   # 创建热点 + 防火墙 + DHCP 静态绑定 + 开机自启
#   sudo ./setup_hotspot.sh remove   # 删除热点配置
#   sudo ./setup_hotspot.sh start    # 启动热点
#   sudo ./setup_hotspot.sh stop     # 关闭热点
#   sudo ./setup_hotspot.sh status   # 查看热点状态
#   sudo ./setup_hotspot.sh enable   # 设为开机自动启动
#   sudo ./setup_hotspot.sh disable  # 取消开机自启
#   sudo ./setup_hotspot.sh test     # 测试网络连通性
# ==============================================================================

set -euo pipefail

# ---- 可配置参数 ----
CON_NAME="ed-hotspot"
IFACE="${ED_HOTSPOT_IFACE:-}"           # 留空则自动检测第一个 wlan 接口
STA_IFACE=""                            # 保存原始 STA 接口名（AP+STA 共存时使用）
SSID="${ED_HOTSPOT_SSID:-ED-UAV}"
PASSWORD="${ED_HOTSPOT_PASSWORD:-5RQqDVzbg5GxZpLz}"     # 留空则开放网络（推荐测试时使用）
CHANNEL="${ED_HOTSPOT_CHANNEL:-6}"
BAND="${ED_HOTSPOT_BAND:-bg}"           # bg = 2.4GHz, a = 5GHz

# 三端固定地址（与协议一致）
NUC_IP="192.168.20.1"
NUC_PREFIX="24"
CAR_IP="192.168.20.2"
HMI_IP="192.168.20.3"
SUBNET="192.168.20.0/24"

# ESP32 MAC 地址（烧录前从设备背面或串口日志读取，格式 AA:BB:CC:DD:EE:FF）
CAR_MAC="${ED_CAR_MAC:-}"
HMI_MAC="${ED_HMI_MAC:-}"

DNSMASQ_CONF="/etc/NetworkManager/dnsmasq-shared.d/ed-hotspot.conf"
HOSTS_FILE="/etc/NetworkManager/dnsmasq-shared.d/ed-hotspot.hosts"

# ---- 颜色输出 ----
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[0;33m'; CYAN='\033[0;36m'; NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }
fatal() { error "$*"; exit 1; }

# ---- 前置检查 ----
check_root() {
    [[ $EUID -eq 0 ]] || fatal "必须使用 sudo 运行此脚本"
}

check_nm() {
    command -v nmcli >/dev/null 2>&1 || fatal "需要 NetworkManager (nmcli)。Ubuntu 桌面版默认已安装。"
    # 使用 -t 避免 locale 问题，检查 NM 是否活跃
    local state
    state=$(nmcli -t general status 2>/dev/null | head -1 | cut -d: -f1)
    [[ -n "$state" ]] || fatal "NetworkManager 未运行"
}

detect_iface() {
    # 检测所有无线接口
    local -a wifi_ifaces
    mapfile -t wifi_ifaces < <(nmcli -t -f DEVICE,TYPE device status | grep ':wifi$' | cut -d: -f1)

    if [[ ${#wifi_ifaces[@]} -eq 0 ]]; then
        fatal "未检测到无线接口。请插入 USB 无线网卡或用 ED_HOTSPOT_IFACE 指定。"
    fi

    if [[ ${#wifi_ifaces[@]} -ge 2 ]]; then
        # 多个无线接口：找到已连接的那个（STA），另一个做 AP
        info "检测到 ${#wifi_ifaces[@]} 个无线接口: ${wifi_ifaces[*]}"
        local default_route_dev
        default_route_dev=$(ip route 2>/dev/null | awk '/^default/{print $5; exit}')
        for iface in "${wifi_ifaces[@]}"; do
            local state
            state=$(nmcli -t -f DEVICE,STATE device status | grep "^${iface}:" | cut -d: -f2)
            if [[ "$state" == "已连接" || "$state" == "connected" ]]; then
                # 有默认路由的接口优先做 STA（互联网出口）
                if [[ "$iface" == "$default_route_dev" ]]; then
                    STA_IFACE="$iface"
                    info "  $iface → STA (已连接互联网，默认路由)"
                elif [[ -z "$STA_IFACE" ]]; then
                    STA_IFACE="$iface"
                    info "  $iface → STA (已连接互联网)"
                else
                    IFACE="$iface"
                    info "  $iface → AP (将用于热点)"
                fi
            else
                IFACE="$iface"
                info "  $iface → AP (将用于热点)"
            fi
        done
        # 兜底：所有接口都已连接且仍未确定 AP，取最后一个非 STA 接口
        if [[ -z "$IFACE" && -n "$STA_IFACE" ]]; then
            for iface in "${wifi_ifaces[@]}"; do
                if [[ "$iface" != "$STA_IFACE" ]]; then
                    IFACE="$iface"
                    info "  $iface → AP (兜底选择)"
                    break
                fi
            done
        fi
        if [[ -z "$STA_IFACE" ]]; then
            # 没有已连接的接口，用第一个做 AP
            IFACE="${wifi_ifaces[0]}"
            warn "没有已连接的无线接口，使用 $IFACE 做 AP"
        fi
        if [[ -z "$IFACE" ]]; then
            fatal "无法确定 AP 接口。请用 ED_HOTSPOT_IFACE 指定。"
        fi
    else
        # 单个无线接口
        IFACE="${wifi_ifaces[0]}"
        STA_IFACE=""
        info "检测到单个无线接口: $IFACE"
        local state
        state=$(nmcli -t -f DEVICE,STATE device status | grep "^${IFACE}:" | cut -d: -f2)
        if [[ "$state" == "已连接" || "$state" == "connected" ]]; then
            warn "$IFACE 当前已连接互联网。热点创建将断开此连接。"
            warn "建议插入 USB 无线网卡以保持互联网连接。"
        fi
    fi
}

get_active_wifi() {
    # 返回指定接口上当前活跃的无线连接名
    local iface="${1:-$IFACE}"
    nmcli -t -f NAME,DEVICE,TYPE connection show --active 2>/dev/null \
        | grep ":${iface}:802-11-wireless" | head -1 | cut -d: -f1 || true
}

# ---- 核心功能 ----

do_create() {
    info "正在创建热点 '$SSID' ..."

    # 1) 若同名连接已存在，先删除
    if nmcli connection show "$CON_NAME" &>/dev/null; then
        warn "连接 '$CON_NAME' 已存在，将覆盖"
        nmcli connection delete "$CON_NAME" 2>/dev/null || true
    fi

    # 2) 如果 AP 接口上有活跃连接，断开它
    local active_con
    active_con=$(get_active_wifi "$IFACE")
    if [[ -n "$active_con" ]]; then
        info "断开 $IFACE 上的连接: $active_con"
        nmcli connection down "$active_con" 2>/dev/null || true
        sleep 1
    fi

    # 3) 创建 AP 连接
    local -a ap_args=(
        connection add
        type wifi
        ifname "$IFACE"
        con-name "$CON_NAME"
        ssid "$SSID"
        802-11-wireless.mode ap
        802-11-wireless.band "$BAND"
        802-11-wireless.channel "$CHANNEL"
        ipv4.method shared
        ipv4.addresses "${NUC_IP}/${NUC_PREFIX}"
        ipv4.never-default yes
        ipv6.method disabled
        connection.autoconnect no
        connection.zone trusted
    )

    # WPA2 密码（8~63 字符）
    if [[ -n "$PASSWORD" ]]; then
        if [[ ${#PASSWORD} -lt 8 || ${#PASSWORD} -gt 63 ]]; then
            fatal "WPA2 密码长度必须为 8~63 字符"
        fi
        ap_args+=(
            802-11-wireless-security.key-mgmt wpa-psk
            802-11-wireless-security.psk "$PASSWORD"
        )
    fi

    nmcli "${ap_args[@]}" || fatal "创建连接失败"
    info "连接配置已创建 (接口: $IFACE)"

    # 4) IP 转发 + NAT
    setup_forwarding

    # 5) DHCP 静态绑定
    setup_dhcp

    # 6) 创建 /etc/hosts 便于调试
    setup_hosts

    # 7) 启动热点
    do_start

    info "热点创建完成"
    if [[ -n "$STA_IFACE" ]]; then
        echo ""
        info "══════════════════════════════════════════"
        info "  双网卡模式：互联网 + 热点同时可用"
        info "  STA 接口: $STA_IFACE (连接互联网)"
        info "  AP  接口: $IFACE  (ED-UAV 热点)"
        info "══════════════════════════════════════════"
    fi
    echo ""
    print_summary
}

setup_forwarding() {
    info "配置 IP 转发和 NAT ..."

    # 内核 IP 转发
    local sysctl_conf="/etc/sysctl.d/99-ed-hotspot.conf"
    cat > "$sysctl_conf" <<'EOF'
# ED UAV hotspot: enable IPv4 forwarding
net.ipv4.ip_forward = 1
EOF
    sysctl -p "$sysctl_conf" >/dev/null 2>&1

    # iptables NAT（如果有线接口访问外网）
    local wan_iface
    wan_iface=$(ip route | grep '^default' | head -1 | awk '{print $5}')
    if [[ -n "$wan_iface" && "$wan_iface" != "$IFACE" ]]; then
        # 清除旧规则
        iptables -t nat -D POSTROUTING -s "$SUBNET" -o "$wan_iface" -j MASQUERADE 2>/dev/null || true
        iptables -t nat -A POSTROUTING -s "$SUBNET" -o "$wan_iface" -j MASQUERADE
        info "NAT 规则已添加 (出口: $wan_iface)"
    fi

    # 允许热点子网内转发（客户端互通关键）
    iptables -D FORWARD -i "$IFACE" -o "$IFACE" -j ACCEPT 2>/dev/null || true
    iptables -A FORWARD -i "$IFACE" -o "$IFACE" -j ACCEPT

    # 持久化 iptables
    if command -v iptables-save >/dev/null 2>&1; then
        mkdir -p /etc/iptables
        iptables-save > /etc/iptables/rules.v4 2>/dev/null || true
    fi
}

setup_dhcp() {
    info "配置 DHCP 静态绑定 ..."

    mkdir -p "$(dirname "$DNSMASQ_CONF")"

    # dnsmasq 共享模式配置
    cat > "$DNSMASQ_CONF" <<EOF
# ED UAV hotspot dnsmasq 配置（由 setup_hotspot.sh 管理）
interface=${IFACE}
bind-interfaces

# DHCP 范围（排除 NUC/网关地址）
dhcp-range=192.168.20.10,192.168.20.50,255.255.255.0,12h

# 网关和 DNS
dhcp-option=option:router,${NUC_IP}
dhcp-option=option:dns-server,${NUC_IP}

# 静态绑定（格式：MAC,IP,hostname,lease-time）
EOF

    if [[ -n "$CAR_MAC" ]]; then
        echo "dhcp-host=${CAR_MAC},${CAR_IP},ed-car,infinite" >> "$DNSMASQ_CONF"
        info "CAR 绑定: ${CAR_MAC} → ${CAR_IP}"
    else
        warn "未设置 CAR MAC 地址 (ED_CAR_MAC)，CAR 将获得动态 IP"
    fi

    if [[ -n "$HMI_MAC" ]]; then
        echo "dhcp-host=${HMI_MAC},${HMI_IP},ed-hmi,infinite" >> "$DNSMASQ_CONF"
        info "HMI 绑定: ${HMI_MAC} → ${HMI_IP}"
    else
        warn "未设置 HMI MAC 地址 (ED_HMI_MAC)，HMI 将获得动态 IP"
    fi

    # 重启 NM 使 dnsmasq 配置生效
    systemctl reload NetworkManager 2>/dev/null || true
}

setup_hosts() {
    info "添加 /etc/hosts 条目 ..."
    # 先清除旧条目
    sed -i '/# ed-hotspot$/d' /etc/hosts 2>/dev/null || true
    cat >> /etc/hosts <<EOF
${NUC_IP}  ed-nuc  # ed-hotspot
${CAR_IP}  ed-car  # ed-hotspot
${HMI_IP}  ed-hmi  # ed-hotspot
EOF
}

do_remove() {
    info "正在删除热点配置 ..."
    nmcli connection delete "$CON_NAME" 2>/dev/null || warn "连接 '$CON_NAME' 不存在"
    rm -f "$DNSMASQ_CONF" "$HOSTS_FILE"
    sed -i '/# ed-hotspot$/d' /etc/hosts 2>/dev/null || true
    rm -f /etc/sysctl.d/99-ed-hotspot.conf
    # 清理 iptables
    local wan_iface
    wan_iface=$(ip route | grep '^default' | head -1 | awk '{print $5}')
    if [[ -n "$wan_iface" ]]; then
        iptables -t nat -D POSTROUTING -s "$SUBNET" -o "$wan_iface" -j MASQUERADE 2>/dev/null || true
    fi
    iptables -D FORWARD -i "$IFACE" -o "$IFACE" -j ACCEPT 2>/dev/null || true
    systemctl reload NetworkManager 2>/dev/null || true
    info "热点配置已删除"
}

do_start() {
    info "正在启动热点 ..."
    nmcli connection up "$CON_NAME" || fatal "启动热点失败"
    sleep 2
    # 确认接口有 IP
    if ip addr show "$IFACE" | grep -q "$NUC_IP"; then
        info "热点已启动，${IFACE} = ${NUC_IP}"
    else
        warn "热点已启动但 $IFACE 上未找到 $NUC_IP，请检查 NetworkManager 日志"
    fi
}

do_stop() {
    info "正在关闭热点 ..."
    nmcli connection down "$CON_NAME" 2>/dev/null && info "热点已关闭" || warn "热点未在运行"
}

do_enable() {
    info "设置开机自动启动 ..."
    nmcli connection modify "$CON_NAME" connection.autoconnect yes
    info "已启用开机自启"
}

do_disable() {
    info "取消开机自动启动 ..."
    nmcli connection modify "$CON_NAME" connection.autoconnect no
    info "已取消开机自启"
}

do_status() {
    echo ""
    echo -e "${CYAN}==== ED UAV 热点状态 ====${NC}"
    echo ""

    # 连接状态
    local con_state
    con_state=$(nmcli -t -f NAME,DEVICE,TYPE connection show --active 2>/dev/null \
        | grep "${CON_NAME}" | head -1 || true)
    if [[ -n "$con_state" ]]; then
        echo -e "  连接:    ${GREEN}已激活${NC}"
    else
        echo -e "  连接:    ${RED}未激活${NC}"
    fi

    # IP 地址
    local display_iface="${IFACE}"
    if [[ -n "$STA_IFACE" ]] && [[ "$IFACE" != "$STA_IFACE" ]]; then
        # AP+STA 共存模式，显示两个接口
        local sta_ip
        sta_ip=$(ip -4 addr show "$STA_IFACE" 2>/dev/null | grep 'inet ' | awk '{print $2}' | head -1)
        if [[ -n "$sta_ip" ]]; then
            echo -e "  STA接口:  ${GREEN}${STA_IFACE}${NC} = ${sta_ip} (互联网)"
        else
            echo -e "  STA接口:  ${YELLOW}${STA_IFACE}${NC} (未获取IP)"
        fi
    fi
    if ip addr show "$display_iface" 2>/dev/null | grep -q "$NUC_IP"; then
        echo -e "  AP接口:   ${GREEN}${display_iface}${NC} = ${NUC_IP}"
    else
        echo -e "  AP接口:   ${YELLOW}${display_iface}${NC} (未分配 IP)"
    fi

    # IP 转发
    local fwd
    fwd=$(cat /proc/sys/net/ipv4/ip_forward)
    if [[ "$fwd" == "1" ]]; then
        echo -e "  转发:    ${GREEN}已启用${NC}"
    else
        echo -e "  转发:    ${RED}未启用${NC}"
    fi

    # DHCP
    if [[ -f "$DNSMASQ_CONF" ]]; then
        echo -e "  DHCP:    ${GREEN}已配置${NC}"
        grep 'dhcp-host=' "$DNSMASQ_CONF" 2>/dev/null | while read -r line; do
            echo "           $line"
        done
    else
        echo -e "  DHCP:    ${YELLOW}未配置静态绑定${NC}"
    fi

    # 自动启动
    local auto
    auto=$(nmcli -t -f NAME,AUTOCONNECT connection show "$CON_NAME" 2>/dev/null \
        | cut -d: -f2 || true)
    if [[ "$auto" == "yes" ]]; then
        echo -e "  自启动:  ${GREEN}已启用${NC}"
    else
        echo -e "  自启动:  ${YELLOW}未启用${NC}"
    fi

    # 已连接客户端
    echo ""
    echo -e "  ${CYAN}已连接客户端:${NC}"
    local clients
    clients=$(nmcli -t -f DEVICE,TYPE,STATE device status 2>/dev/null \
        | grep "${IFACE}:wifi:connected" || true)
    if [[ -n "$clients" ]]; then
        # 查看 ARP 表
        ip neigh show dev "$IFACE" 2>/dev/null | grep -v FAILED | while read -r line; do
            local ip_addr mac
            ip_addr=$(echo "$line" | awk '{print $1}')
            mac=$(echo "$line" | awk '{print $5}')
            local label=""
            [[ "$ip_addr" == "$CAR_IP" ]] && label=" (CAR)"
            [[ "$ip_addr" == "$HMI_IP" ]] && label=" (HMI)"
            echo "           ${ip_addr} ${mac}${label}"
        done
    else
        echo "           (无)"
    fi

    echo ""
}

do_test() {
    echo ""
    echo -e "${CYAN}==== 网络连通性测试 ====${NC}"
    echo ""

    # 1) 接口检查
    if ! ip addr show "$IFACE" 2>/dev/null | grep -q "$NUC_IP"; then
        fatal "热点未启动 ($IFACE 上没有 $NUC_IP)"
    fi
    echo -e "  [OK] 接口 $IFACE 已就绪 ($NUC_IP)"

    # 2) IP 转发检查
    if [[ "$(cat /proc/sys/net/ipv4/ip_forward)" != "1" ]]; then
        echo -e "  [FAIL] IP 转发未启用"
    else
        echo -e "  [OK] IP 转发已启用"
    fi

    # 3) ARP 探测（ping 广播地址触发 ARP）
    echo ""
    info "正在探测客户端 ..."
    ping -c 1 -b -W 1 "$SUBNET" >/dev/null 2>&1 || true
    sleep 1

    # 4) CAR 连通性
    if ping -c 1 -W 2 "$CAR_IP" >/dev/null 2>&1; then
        echo -e "  [OK] CAR ($CAR_IP) ${GREEN}可达${NC}"
    else
        echo -e "  [--] CAR ($CAR_IP) ${YELLOW}未响应${NC}（ESP32 可能未开机）"
    fi

    # 5) HMI 连通性
    if ping -c 1 -W 2 "$HMI_IP" >/dev/null 2>&1; then
        echo -e "  [OK] HMI ($HMI_IP) ${GREEN}可达${NC}"
    else
        echo -e "  [--] HMI ($HMI_IP) ${YELLOW}未响应${NC}（ESP32 可能未开机）"
    fi

    # 6) UDP 端口检查
    echo ""
    info "检查 UDP 端口 ..."
    if ss -uln | grep -q ":42000 "; then
        echo -e "  [--] 端口 42000 ${YELLOW}已被占用${NC}（可能是 ROS bridge）"
    else
        echo -e "  [OK] 端口 42000 ${GREEN}可用${NC}"
    fi

    echo ""
    echo "如果 CAR/HMI 不可达，请确认："
    echo "  1. ESP32 已上电并烧录固件"
    echo "  2. config_local.h 中 SSID 与本热点一致"
    echo "  3. 热点未启用客户端隔离"
    echo ""
}

print_summary() {
    echo ""
    echo -e "${CYAN}==== 网络拓扑 ====${NC}"
    echo ""
    echo "    ┌───────────────────────────────────┐"
    echo "    │  NUC (AP)  ${NUC_IP}:42000          │"
    echo "    │            SSID: ${SSID}           │"
    echo "    └──────┬────────────────┬────────────┘"
    echo "           │ Wi-Fi          │ Wi-Fi"
    echo "    ┌──────┴──────┐  ┌──────┴──────┐"
    echo "    │ CAR (STA)   │  │ HMI (STA)   │"
    echo "    │ ${CAR_IP}    │  │ ${HMI_IP}    │"
    echo "    │ :42001      │  │ :42002      │"
    echo "    └─────────────┘  └─────────────┘"
    echo ""
    echo "  运行诊断工具:  python3 tools/diagnostics/vehicle_comm_diagnostic.py"
    echo "  查看热点状态:  sudo ./tools/hotspot/setup_hotspot.sh status"
    echo ""
}

# ---- 主入口 ----

main() {
    local cmd="${1:-}"
    [[ -n "$cmd" ]] || { echo "用法: $0 <create|remove|start|stop|status|enable|disable|test>"; exit 1; }

    check_root
    check_nm

    # 非 create/remove 操作也需要知道接口
    if [[ "$cmd" != "create" && "$cmd" != "remove" ]]; then
        detect_iface
    fi

    case "$cmd" in
        create)
            detect_iface
            do_create
            ;;
        remove)
            detect_iface 2>/dev/null || true
            do_remove
            ;;
        start)   do_start ;;
        stop)    do_stop ;;
        status)  do_status ;;
        enable)  do_enable ;;
        disable) do_disable ;;
        test)    do_test ;;
        *)       fatal "未知命令: $cmd (支持 create|remove|start|stop|status|enable|disable|test)" ;;
    esac
}

main "$@"
