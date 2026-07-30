#!/usr/bin/env bash
# ==============================================================================
# ED UAV 开机自启一键安装
#
# 功能：
#   1. 配置 Wi-Fi 热点（NetworkManager AP + DHCP 静态绑定 + 开机自启）
#   2. 安装 ROS vehicle bridge / mission executor 为 systemd 服务
#   3. 安装通信诊断日志服务
#   4. 一键 enable/disable/status
#
# 用法：
#   sudo ./tools/install_boot.sh install   # 交互式安装（首次）
#   sudo ./tools/install_boot.sh uninstall # 卸载所有服务
#   sudo ./tools/install_boot.sh status    # 查看全部服务状态
#   sudo ./tools/install_boot.sh enable    # 全部启用
#   sudo ./tools/install_boot.sh disable   # 全部禁用
# ==============================================================================

set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
REPO_ROOT="$(pwd)"

# ─── 常量 ───────────────────────────────────────────────────────────────────
CON_NAME="ed-hotspot"
NUC_IP="192.168.20.1"
CAR_IP="192.168.20.2"
HMI_IP="192.168.20.3"
SUBNET="192.168.20.0/24"
DNSMASQ_CONF="/etc/NetworkManager/dnsmasq-shared.d/ed-hotspot.conf"
STATE_DIR="/var/lib/ed-uav"
CONFIG_FILE="${STATE_DIR}/boot.conf"

# systemd 服务名
SVC_HOTSPOT_WAIT="ed-hotspot-wait.service"
SVC_VEHICLE_BRIDGE="ed-vehicle-bridge.service"
SVC_MISSION_EXECUTOR="ed-mission-executor.service"
SVC_DIAGNOSTIC="ed-comm-diagnostic.service"

# ─── 颜色 ───────────────────────────────────────────────────────────────────
R='\033[0;31m'; G='\033[0;32m'; Y='\033[0;33m'; C='\033[0;36m'; B='\033[1m'; N='\033[0m'
ok()   { echo -e "  ${G}✓${N} $*"; }
warn() { echo -e "  ${Y}!${N}  $*"; }
fail() { echo -e "  ${R}✗${N} $*" >&2; }
die()  { fail "$*"; exit 1; }
info() { echo -e "${C}── $* ──${N}"; }

# ─── 前置检查 ───────────────────────────────────────────────────────────────
check_root() { [[ $EUID -eq 0 ]] || die "需要 root 权限: sudo $0 $*"; }

detect_iface() {
    IFACE=$(nmcli -t -f DEVICE,TYPE device status 2>/dev/null | grep ':wifi$' | head -1 | cut -d: -f1)
    [[ -n "$IFACE" ]] || die "未检测到无线网卡"
    ok "无线接口: $IFACE"
}

detect_ros() {
    ROS_SETUP=""
    if [[ -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]]; then
        ROS_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"
        ok "ROS 工作空间: $ROS_SETUP"
    else
        warn "ROS 工作空间未构建 (ros2_ws/install/ 不存在)"
    fi
}

# ─── 配置持久化 ─────────────────────────────────────────────────────────────
save_config() {
    mkdir -p "$STATE_DIR"
    cat > "$CONFIG_FILE" <<EOF
# ED UAV 开机自启配置（由 install_boot.sh 生成）
# $(date '+%Y-%m-%d %H:%M:%S')
IFACE=${IFACE}
SSID=${SSID:-ED-UAV}
PASSWORD=${PASSWORD:-}
CHANNEL=${CHANNEL:-6}
BAND=${BAND:-bg}
CAR_MAC=${CAR_MAC:-}
HMI_MAC=${HMI_MAC:-}
HMAC_KEY_FILE=${HMAC_KEY_FILE:-}
MISSION_CONFIG=${MISSION_CONFIG:-}
PROFILE_PATH=${PROFILE_PATH:-}
SIMULATION_ONLY=${SIMULATION_ONLY:-false}
EOF
    ok "配置已保存: $CONFIG_FILE"
}

load_config() {
    if [[ -f "$CONFIG_FILE" ]]; then
        # shellcheck source=/dev/null
        source "$CONFIG_FILE"
        ok "已加载配置: $CONFIG_FILE"
    fi
}

# ─── 热点配置 ───────────────────────────────────────────────────────────────
setup_hotspot() {
    info "配置 Wi-Fi 热点"

    SSID="${SSID:-ED-UAV}"
    PASSWORD="${PASSWORD:-}"
    CHANNEL="${CHANNEL:-6}"
    BAND="${BAND:-bg}"

    # 询问参数（仅首次或交互模式）
    if [[ "${INTERACTIVE:-0}" == "1" ]]; then
        read -rp "  热点名称 [${SSID}]: " input; SSID="${input:-$SSID}"
        read -rp "  WPA2 密码 (留空=开放网络): " input; PASSWORD="${input:-$PASSWORD}"
        read -rp "  信道 [${CHANNEL}]: " input; CHANNEL="${input:-$CHANNEL}"
        read -rp "  频段 (bg=2.4G/a=5G) [${BAND}]: " input; BAND="${input:-BAND}"
        read -rp "  小车 MAC (留空跳过): " input; CAR_MAC="${input:-}"
        read -rp "  地面站 MAC (留空跳过): " input; HMI_MAC="${input:-}"
    fi

    # 清理旧配置
    nmcli connection delete "$CON_NAME" 2>/dev/null || true

    # 创建 AP 连接
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
        connection.autoconnect-priority 100
    )

    if [[ -n "${PASSWORD:-}" ]]; then
        [[ ${#PASSWORD} -ge 8 ]] || die "WPA2 密码至少 8 字符"
        args+=(
            802-11-wireless-security.key-mgmt wpa-psk
            802-11-wireless-security.psk "$PASSWORD"
        )
    fi

    nmcli "${args[@]}" >/dev/null || die "创建热点连接失败"
    ok "热点连接已创建: $SSID"

    # IP 转发
    cat > /etc/sysctl.d/99-ed-hotspot.conf <<< "net.ipv4.ip_forward = 1"
    sysctl -p /etc/sysctl.d/99-ed-hotspot.conf >/dev/null 2>&1
    ok "IP 转发已启用"

    # iptables
    local wan
    wan=$(ip route 2>/dev/null | awk '/^default/{print $5; exit}')
    if [[ -n "$wan" && "$wan" != "$IFACE" ]]; then
        iptables -t nat -C POSTROUTING -s "$SUBNET" -o "$wan" -j MASQUERADE 2>/dev/null \
            || iptables -t nat -A POSTROUTING -s "$SUBNET" -o "$wan" -j MASQUERADE
    fi
    iptables -C FORWARD -i "$IFACE" -o "$IFACE" -j ACCEPT 2>/dev/null \
        || iptables -A FORWARD -i "$IFACE" -o "$IFACE" -j ACCEPT
    # 持久化
    if command -v iptables-save >/dev/null; then
        mkdir -p /etc/iptables
        iptables-save > /etc/iptables/rules.v4 2>/dev/null || true
    fi
    ok "防火墙规则已配置"

    # DHCP 静态绑定
    mkdir -p "$(dirname "$DNSMASQ_CONF")"
    cat > "$DNSMASQ_CONF" <<EOF
interface=${IFACE}
bind-interfaces
dhcp-range=192.168.20.10,192.168.20.50,255.255.255.0,12h
dhcp-option=option:router,${NUC_IP}
dhcp-option=option:dns-server,${NUC_IP}
EOF
    [[ -n "${CAR_MAC:-}" ]] && echo "dhcp-host=${CAR_MAC},${CAR_IP},ed-car,infinite" >> "$DNSMASQ_CONF"
    [[ -n "${HMI_MAC:-}" ]] && echo "dhcp-host=${HMI_MAC},${HMI_IP},ed-hmi,infinite" >> "$DNSMASQ_CONF"
    ok "DHCP 已配置"

    # /etc/hosts
    sed -i '/# ed-boot$/d' /etc/hosts 2>/dev/null || true
    cat >> /etc/hosts <<EOF
${NUC_IP}  ed-nuc  # ed-boot
${CAR_IP}  ed-car  # ed-boot
${HMI_IP}  ed-hmi  # ed-boot
EOF

    systemctl reload NetworkManager 2>/dev/null || true
    ok "热点配置完成"
}

# ─── systemd 服务安装 ───────────────────────────────────────────────────────
install_wait_service() {
    cat > "/etc/systemd/system/${SVC_HOTSPOT_WAIT}" <<EOF
[Unit]
Description=ED UAV: 等待热点接口就绪
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c '\
  IFACE=\$(nmcli -t -f DEVICE,TYPE device status 2>/dev/null | grep ":wifi\$" | head -1 | cut -d: -f1); \
  for i in \$(seq 1 30); do \
    ip addr show "\$IFACE" 2>/dev/null | grep -q "${NUC_IP}" && exit 0; \
    sleep 1; \
  done; \
  echo "热点接口超时"; exit 1'

[Install]
WantedBy=multi-user.target
EOF
    ok "热点等待服务: ${SVC_HOTSPOT_WAIT}"
}

install_vehicle_bridge() {
    [[ -n "${ROS_SETUP:-}" ]] || { warn "跳过 vehicle bridge（ROS 未构建）"; return; }

    local key_arg=""
    [[ -n "${HMAC_KEY_FILE:-}" ]] && key_arg="--key-file ${HMAC_KEY_FILE}"

    cat > "/etc/systemd/system/${SVC_VEHICLE_BRIDGE}" <<EOF
[Unit]
Description=ED UAV: ROS vehicle bridge (UDP ↔ ROS)
After=${SVC_HOTSPOT_WAIT}
Requires=${SVC_HOTSPOT_WAIT}

[Service]
Type=simple
Environment=HOME=/root
ExecStartPre=/bin/sleep 2
ExecStart=/bin/bash -c '\
  source ${ROS_SETUP}; \
  ros2 run ed_uav_vehicle_bridge ed_uav_vehicle_bridge \
    --ros-args -p bind_host:=${NUC_IP} -p bind_port:=42000 \
    -p car_peer_host:=${CAR_IP} -p car_peer_port:=42001 \
    -p hmi_peer_host:=${HMI_IP} -p hmi_peer_port:=42002 \
    -p car_sender_id:=1128419121 -p hmi_sender_id:=1212563761 \
    -p bridge_sender_id:=1381122353 \
    ${key_arg:+-p hmac_key_file:=${HMAC_KEY_FILE}} \
    -p telemetry_stale_seconds:=0.75 \
    -p mission_timeout_seconds:=90.0'
Restart=on-failure
RestartSec=3
StandardOutput=journal
StandardError=journal
SyslogIdentifier=ed-bridge

[Install]
WantedBy=multi-user.target
EOF
    ok "Vehicle bridge 服务: ${SVC_VEHICLE_BRIDGE}"
}

install_mission_executor() {
    [[ -n "${ROS_SETUP:-}" ]] || { warn "跳过 mission executor（ROS 未构建）"; return; }
    [[ -n "${MISSION_CONFIG:-}" ]] || { warn "跳过 mission executor（未配置任务文件）"; return; }

    local profile="${PROFILE_PATH:-}"
    local sim="${SIMULATION_ONLY:-false}"

    cat > "/etc/systemd/system/${SVC_MISSION_EXECUTOR}" <<EOF
[Unit]
Description=ED UAV: ROS mission executor
After=${SVC_VEHICLE_BRIDGE}
Requires=${SVC_VEHICLE_BRIDGE}

[Service]
Type=simple
Environment=HOME=/root
ExecStartPre=/bin/sleep 3
ExecStart=/bin/bash -c '\
  source ${ROS_SETUP}; \
  ros2 run ed_uav_mission mission_executor \
    --ros-args --enclave /ed_uav_mission_executor \
    -p mission_config_path:=${MISSION_CONFIG} \
    ${profile:+-p profile_path:=${profile}} \
    -p simulation_only:=${sim}'
Restart=on-failure
RestartSec=3
StandardOutput=journal
StandardError=journal
SyslogIdentifier=ed-mission

[Install]
WantedBy=multi-user.target
EOF
    ok "Mission executor 服务: ${SVC_MISSION_EXECUTOR}"
}

install_diagnostic() {
    local diag_script="${REPO_ROOT}/tools/diagnostics/vehicle_comm_diagnostic.py"
    [[ -f "$diag_script" ]] || { warn "跳过诊断服务（脚本不存在）"; return; }

    local log_dir="/var/log/ed-uav"
    mkdir -p "$log_dir"

    local key_arg=""
    [[ -n "${HMAC_KEY_FILE:-}" ]] && key_arg="--key-file ${HMAC_KEY_FILE}"

    cat > "/etc/systemd/system/${SVC_DIAGNOSTIC}" <<EOF
[Unit]
Description=ED UAV: 通信诊断日志
After=${SVC_HOTSPOT_WAIT}
Requires=${SVC_HOTSPOT_WAIT}

[Service]
Type=simple
ExecStartPre=/bin/sleep 2
ExecStart=/usr/bin/python3 ${diag_script} ${key_arg} --log-file ${log_dir}/diag_\$(date +%Y%m%d_%H%M%S).log
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=ed-diag

[Install]
WantedBy=multi-user.target
EOF
    ok "诊断日志服务: ${SVC_DIAGNOSTIC}（日志目录: ${log_dir}）"
}

# ─── 安装 ───────────────────────────────────────────────────────────────────
do_install() {
    echo ""
    echo -e "${B}${C}╔══════════════════════════════════════════╗${N}"
    echo -e "${B}${C}║   ED UAV 开机自启配置                    ║${N}"
    echo -e "${B}${C}╚══════════════════════════════════════════╝${N}"
    echo ""

    check_root
    detect_iface
    detect_ros

    # 交互式输入
    echo ""
    info "热点配置"
    load_config
    INTERACTIVE=1

    read -rp "  热点名称 [${SSID:-ED-UAV}]: " input; SSID="${input:-${SSID:-ED-UAV}}"
    read -rp "  WPA2 密码 (留空=开放): " input; PASSWORD="${input:-${PASSWORD:-}}"
    read -rp "  信道 [${CHANNEL:-6}]: " input; CHANNEL="${input:-${CHANNEL:-6}}"
    read -rp "  小车 MAC (可选): " input; CAR_MAC="${input:-${CAR_MAC:-}}"
    read -rp "  地面站 MAC (可选): " input; HMI_MAC="${input:-${HMI_MAC:-}}"

    echo ""
    info "ROS 配置"
    read -rp "  HMAC 密钥文件 (十六进制, 可选): " input; HMAC_KEY_FILE="${input:-${HMAC_KEY_FILE:-}}"
    read -rp "  任务配置文件 (可选, 留空跳过 mission): " input; MISSION_CONFIG="${input:-${MISSION_CONFIG:-}}"
    read -rp "  场地配置文件 (可选): " input; PROFILE_PATH="${input:-${PROFILE_PATH:-}}"
    read -rp "  仅仿真模式? (true/false) [${SIMULATION_ONLY:-false}]: " input; SIMULATION_ONLY="${input:-${SIMULATION_ONLY:-false}}"

    echo ""
    info "安装服务"

    # 热点
    setup_hotspot

    # systemd
    install_wait_service
    install_vehicle_bridge
    install_mission_executor
    install_diagnostic

    # 保存配置
    save_config

    # 重载并启用
    systemctl daemon-reload
    systemctl enable "$SVC_HOTSPOT_WAIT" >/dev/null 2>&1
    [[ -f "/etc/systemd/system/${SVC_VEHICLE_BRIDGE}" ]] && systemctl enable "$SVC_VEHICLE_BRIDGE" >/dev/null 2>&1
    [[ -f "/etc/systemd/system/${SVC_MISSION_EXECUTOR}" ]] && systemctl enable "$SVC_MISSION_EXECUTOR" >/dev/null 2>&1
    [[ -f "/etc/systemd/system/${SVC_DIAGNOSTIC}" ]] && systemctl enable "$SVC_DIAGNOSTIC" >/dev/null 2>&1
    ok "所有服务已启用"

    # 启动热点
    nmcli connection up "$CON_NAME" >/dev/null 2>&1 || warn "热点启动失败（可能已在运行）"

    echo ""
    echo -e "${G}${B}安装完成！下次开机将自动启动全部服务。${N}"
    echo ""
    echo "  管理命令:"
    echo "    sudo systemctl status ed-*            # 查看全部服务"
    echo "    sudo journalctl -u ed-vehicle-bridge -f  # 实时日志"
    echo "    sudo ./tools/install_boot.sh status   # 一键状态"
    echo ""
}

# ─── 卸载 ───────────────────────────────────────────────────────────────────
do_uninstall() {
    check_root
    info "卸载 ED UAV 开机自启服务"

    for svc in "$SVC_DIAGNOSTIC" "$SVC_MISSION_EXECUTOR" "$SVC_VEHICLE_BRIDGE" "$SVC_HOTSPOT_WAIT"; do
        if systemctl is-enabled "$svc" &>/dev/null; then
            systemctl disable --now "$svc" 2>/dev/null || true
            ok "已禁用: $svc"
        fi
        rm -f "/etc/systemd/system/${svc}"
    done

    nmcli connection delete "$CON_NAME" 2>/dev/null || true
    rm -f "$DNSMASQ_CONF"
    rm -f /etc/sysctl.d/99-ed-hotspot.conf
    sed -i '/# ed-boot$/d' /etc/hosts 2>/dev/null || true

    systemctl daemon-reload
    ok "全部服务已卸载"
    echo ""
    echo "  状态目录 ${STATE_DIR} 和日志目录 /var/log/ed-uav 未删除"
    echo "  如需清理: sudo rm -rf ${STATE_DIR} /var/log/ed-uav"
    echo ""
}

# ─── 状态 ───────────────────────────────────────────────────────────────────
do_status() {
    echo ""
    echo -e "${B}${C}┌── ED UAV 开机自启状态 ────────────────────────────────┐${N}"

    # 热点
    if nmcli -t -f NAME connection show --active 2>/dev/null | grep -q "^${CON_NAME}:"; then
        echo -e "  热点:      ${G}● 运行中${N}  (SSID: ${SSID:-?})"
    else
        echo -e "  热点:      ${R}○ 未运行${N}"
    fi

    # 接口
    local iface
    iface=$(nmcli -t -f DEVICE,TYPE device status 2>/dev/null | grep ':wifi$' | head -1 | cut -d: -f1)
    if ip addr show "$iface" 2>/dev/null | grep -q "$NUC_IP"; then
        echo -e "  接口:      ${G}${iface}${N} = ${NUC_IP}"
    else
        echo -e "  接口:      ${Y}${iface:-?}${N} (无 IP)"
    fi

    echo ""

    # systemd 服务
    for svc in "$SVC_HOTSPOT_WAIT" "$SVC_VEHICLE_BRIDGE" "$SVC_MISSION_EXECUTOR" "$SVC_DIAGNOSTIC"; do
        local label="${svc%.service}"
        if [[ ! -f "/etc/systemd/system/${svc}" ]]; then
            echo -e "  ${label}:  ${Y}未安装${N}"
            continue
        fi
        local active
        active=$(systemctl is-active "$svc" 2>/dev/null || true)
        local enabled
        enabled=$(systemctl is-enabled "$svc" 2>/dev/null || true)
        local status_icon
        case "$active" in
            active)   status_icon="${G}● 运行中${N}" ;;
            inactive) status_icon="${Y}○ 已停止${N}" ;;
            *)        status_icon="${R}✗ 失败${N}" ;;
        esac
        local auto_icon=""
        [[ "$enabled" == "enabled" ]] && auto_icon=" [自启]" || auto_icon=" [手动]"
        echo -e "  ${label}:  ${status_icon}${auto_icon}"
    done

    echo ""

    # 客户端
    echo -e "  ${C}已连接设备:${N}"
    if [[ -n "${iface:-}" ]]; then
        ip neigh show dev "$iface" 2>/dev/null | grep -v FAILED | while read -r line; do
            local ip_addr mac
            ip_addr=$(echo "$line" | awk '{print $1}')
            mac=$(echo "$line" | awk '{print $5}')
            local tag=""
            [[ "$ip_addr" == "$CAR_IP" ]] && tag=" ${B}← CAR${N}"
            [[ "$ip_addr" == "$HMI_IP" ]] && tag=" ${B}← HMI${N}"
            echo "    ${ip_addr}  ${mac}${tag}"
        done || echo "    (无)"
    fi

    echo ""
    echo -e "${C}└──────────────────────────────────────────────────────┘${N}"
    echo ""
}

# ─── enable/disable ─────────────────────────────────────────────────────────
do_enable() {
    check_root
    for svc in "$SVC_HOTSPOT_WAIT" "$SVC_VEHICLE_BRIDGE" "$SVC_MISSION_EXECUTOR" "$SVC_DIAGNOSTIC"; do
        [[ -f "/etc/systemd/system/${svc}" ]] && systemctl enable "$svc" 2>/dev/null && ok "已启用: ${svc%.service}"
    done
}

do_disable() {
    check_root
    for svc in "$SVC_HOTSPOT_WAIT" "$SVC_VEHICLE_BRIDGE" "$SVC_MISSION_EXECUTOR" "$SVC_DIAGNOSTIC"; do
        systemctl disable "$svc" 2>/dev/null && ok "已禁用: ${svc%.service}" || true
    done
}

# ─── 启动/停止（立即） ─────────────────────────────────────────────────────
do_start() {
    check_root
    nmcli connection up "$CON_NAME" >/dev/null 2>&1 || true
    for svc in "$SVC_HOTSPOT_WAIT" "$SVC_VEHICLE_BRIDGE" "$SVC_MISSION_EXECUTOR" "$SVC_DIAGNOSTIC"; do
        [[ -f "/etc/systemd/system/${svc}" ]] && systemctl start "$svc" 2>/dev/null && ok "已启动: ${svc%.service}"
    done
}

do_stop() {
    check_root
    for svc in "$SVC_DIAGNOSTIC" "$SVC_MISSION_EXECUTOR" "$SVC_VEHICLE_BRIDGE" "$SVC_HOTSPOT_WAIT"; do
        systemctl stop "$svc" 2>/dev/null && ok "已停止: ${svc%.service}" || true
    done
}

do_restart() {
    do_stop
    do_start
}

# ─── 日志 ───────────────────────────────────────────────────────────────────
do_logs() {
    local svc="${1:-vehicle-bridge}"
    case "$svc" in
        bridge|vehicle-bridge)  journalctl -u "$SVC_VEHICLE_BRIDGE" -f ;;
        mission)                journalctl -u "$SVC_MISSION_EXECUTOR" -f ;;
        diag|diagnostic)        journalctl -u "$SVC_DIAGNOSTIC" -f ;;
        all)                    journalctl -u "ed-*" -f ;;
        *)                      journalctl -u "$svc" -f ;;
    esac
}

# ─── 主入口 ─────────────────────────────────────────────────────────────────
main() {
    local cmd="${1:-help}"

    case "$cmd" in
        install)    do_install ;;
        uninstall)  do_uninstall ;;
        status)     do_status ;;
        enable)     do_enable ;;
        disable)    do_disable ;;
        start)      do_start ;;
        stop)       do_stop ;;
        restart)    do_restart ;;
        logs)       do_logs "${2:-all}" ;;
        help|-h|--help)
            echo ""
            echo "  用法: sudo $0 <命令>"
            echo ""
            echo "  命令:"
            echo "    install    交互式安装全部服务（首次）"
            echo "    uninstall  卸载全部服务"
            echo "    status     查看全部服务状态"
            echo "    start      立即启动全部服务"
            echo "    stop       停止全部服务"
            echo "    restart    重启全部服务"
            echo "    enable     启用开机自启"
            echo "    disable    禁用开机自启"
            echo "    logs [svc] 查看实时日志 (bridge/mission/diag/all)"
            echo ""
            ;;
        *)
            die "未知命令: $cmd (运行 $0 help)"
            ;;
    esac
}

main "$@"
