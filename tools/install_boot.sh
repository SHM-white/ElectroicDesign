#!/usr/bin/env bash
# ==============================================================================
# ED UAV 开机自启一键安装
#
# 功能：
#   1. 配置 Wi-Fi 热点（NetworkManager AP + DHCP 静态绑定 + 开机自启）
#   2. 安装最底层守护进程 (guardian) 托管 vehicle_bridge 通信模块
#   3. 安装 mission executor / 无小车模拟环境服务
#   4. 安装通信诊断日志服务
#   5. 一键 enable/disable/status
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

# 无线接口（由 detect_iface 设置）
IFACE="${ED_HOTSPOT_IFACE:-}"
STA_IFACE=""

# systemd 服务名
SVC_HOTSPOT_WAIT="ed-hotspot-wait.service"
SVC_GUARDIAN="ed-guardian.service"
SVC_NO_CAR_SIM="ed-no-car-sim.service"
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
    # 如果已通过环境变量指定接口，直接使用
    if [[ -n "${ED_HOTSPOT_IFACE:-}" ]]; then
        IFACE="$ED_HOTSPOT_IFACE"
        ok "使用指定接口: $IFACE"
        return
    fi

    # 如果已由调用方设置 IFACE，跳过自动检测
    if [[ -n "$IFACE" ]]; then
        ok "AP 接口: $IFACE"
        return
    fi

    local -a wifi_ifaces
    mapfile -t wifi_ifaces < <(nmcli -t -f DEVICE,TYPE device status 2>/dev/null | grep ':wifi$' | cut -d: -f1)

    if [[ ${#wifi_ifaces[@]} -eq 0 ]]; then
        die "未检测到无线接口。请插入 USB 无线网卡或设置 ED_HOTSPOT_IFACE 手动指定。"
    fi

    STA_IFACE=""
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
        warn "仅检测到 1 个无线接口 ($IFACE)，热点会断开当前 Wi-Fi。建议插入 USB 无线网卡以保持互联网连接。"
    fi
    ok "AP 接口: $IFACE"
}

detect_ros() {
    ROS_SETUP=""
    ROS_UNDERLAY=""
    if [[ -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]]; then
        ROS_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"
        ok "ROS 工作空间: $ROS_SETUP"
    else
        warn "ROS 工作空间未构建 (ros2_ws/install/ 不存在)"
    fi
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        ROS_UNDERLAY="/opt/ros/humble/setup.bash"
        ok "ROS 基础环境: $ROS_UNDERLAY"
    fi
}

# systemd 子进程无登录 shell, 必须显式 source 两层环境
ros_source_line() {
    local line=""
    [[ -n "$ROS_UNDERLAY" ]] && line="source ${ROS_UNDERLAY}; "
    line="${line}source ${ROS_SETUP}"
    echo "$line"
}

# ─── 配置持久化 ─────────────────────────────────────────────────────────────
save_config() {
    mkdir -p "$STATE_DIR"
    cat > "$CONFIG_FILE" <<EOF
# ED UAV 开机自启配置（由 install_boot.sh 生成）
# $(date '+%Y-%m-%d %H:%M:%S')
IFACE=${IFACE}
STA_IFACE=${STA_IFACE:-}
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
NO_CAR_MODE=${NO_CAR_MODE:-false}
TASK3_IDENTITY=${TASK3_IDENTITY:-task3-stability-2026}
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
        read -rp "  频段 (bg=2.4G/a=5G) [${BAND}]: " input; BAND="${input:-$BAND}"
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
        connection.zone trusted
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

    # WPA2 明确参数（ESP32 兼容）
    if [[ -n "${PASSWORD:-}" ]]; then
        nmcli connection modify "$CON_NAME" 802-11-wireless-security.proto rsn 2>/dev/null || true
        nmcli connection modify "$CON_NAME" 802-11-wireless-security.pairwise ccmp 2>/dev/null || true
        nmcli connection modify "$CON_NAME" 802-11-wireless-security.group ccmp 2>/dev/null || true
    fi

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

    # 允许热点子网的 UDP/TCP 入站（ESP32 通讯需要）
    iptables -C INPUT -i "$IFACE" -p udp -s "$SUBNET" -j ACCEPT 2>/dev/null \
        || iptables -I INPUT 1 -i "$IFACE" -p udp -s "$SUBNET" -j ACCEPT
    iptables -C INPUT -i "$IFACE" -p tcp -s "$SUBNET" -j ACCEPT 2>/dev/null \
        || iptables -I INPUT 2 -i "$IFACE" -p tcp -s "$SUBNET" -j ACCEPT

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
  IFACE="${IFACE}"; \
  for i in \$(seq 1 30); do \
    ip addr show "\$IFACE" 2>/dev/null | grep -q "${NUC_IP}" && exit 0; \
    sleep 1; \
  done; \
  echo "热点接口 \$IFACE 超时"; exit 1'

[Install]
WantedBy=multi-user.target
EOF
    ok "热点等待服务: ${SVC_HOTSPOT_WAIT} (接口: ${IFACE})"
}

install_guardian() {
    [[ -n "${ROS_SETUP:-}" ]] || { warn "跳过 guardian（ROS 未构建）"; return; }
    local ros_source
    ros_source="$(ros_source_line)"

    # 自动查找密钥文件
    local effective_key="${HMAC_KEY_FILE:-}"
    if [[ -z "$effective_key" && -f "${REPO_ROOT}/config/hmac.key.hex" ]]; then
        effective_key="${REPO_ROOT}/config/hmac.key.hex"
        ok "自动使用密钥文件: $effective_key"
    fi

    local key_param=""
    [[ -n "$effective_key" ]] && key_param="-p hmac_key_file:=${effective_key}"

    # 无小车模式: bridge 加 no_car_mode + task3 身份参数, 地面站 TASK 直接开始任务
    local bridge_mode_params=""
    if [[ "${NO_CAR_MODE:-false}" == "true" ]]; then
        local field_profile_id
        field_profile_id="$(grep -m1 '^profile_id:' "$PROFILE_PATH" 2>/dev/null | awk '{print $2}')"
        bridge_mode_params=" -p no_car_mode:=true \
 -p task3_mission_id:=${TASK3_IDENTITY:-task3-stability-2026} \
 -p task3_field_profile_id:=${field_profile_id:-d-arena-2026} \
 -p task3_mission_profile_id:=task3-stability \
 -p task3_deployment_preset_id:=field-2026 \
 -p task3_target_revision:=d2026-apriltag-v1 \
 -p task3_timeout_seconds:=120.0"
        ok "无小车模式: 地面站 TASK 指令直接开始任务 (identity=${TASK3_IDENTITY:-task3-stability-2026})"
    fi

    # guardian 环境文件 (START_CMD 由 guardian 负责拉起 bridge)
    local guardian_conf="${STATE_DIR}/guardian.conf"
    mkdir -p "$(dirname "$guardian_conf")"
    cat > "$guardian_conf" <<EOF
ED_GUARDIAN_LOG_DIR=/var/log/ed-uav
ED_GUARDIAN_WATCH_NAME=vehicle_bridge
ED_GUARDIAN_START_CMD=${ros_source}; ros2 run ed_uav_vehicle_bridge vehicle_bridge --ros-args -p bind_host:=${NUC_IP} -p bind_port:=42000 -p car_peer_host:=${CAR_IP} -p car_peer_port:=42001 -p hmi_peer_host:=${HMI_IP} -p hmi_peer_port:=42002 -p car_sender_id:=1128419121 -p hmi_sender_id:=1213024561 -p bridge_sender_id:=1381122353 ${key_param} -p telemetry_stale_seconds:=0.75 -p mission_timeout_seconds:=90.0${bridge_mode_params}
EOF
    chmod 600 "$guardian_conf"
    ok "guardian 配置: $guardian_conf"

    cat > "/etc/systemd/system/${SVC_GUARDIAN}" <<EOF
[Unit]
Description=ED UAV: 最底层守护进程 (监控 vehicle_bridge 崩溃自动拉起并记录日志)
After=${SVC_HOTSPOT_WAIT}
Requires=${SVC_HOTSPOT_WAIT}

[Service]
Type=simple
Environment=HOME=/root
EnvironmentFile=${guardian_conf}
ExecStartPre=/bin/sleep 2
ExecStart=${REPO_ROOT}/tools/ed_guardian.sh
Restart=always
RestartSec=2
StartLimitIntervalSec=0
StandardOutput=journal
StandardError=journal
SyslogIdentifier=ed-guardian

[Install]
WantedBy=multi-user.target
EOF
    ok "Guardian 服务: ${SVC_GUARDIAN} (Restart=always, 接管 vehicle_bridge 生命周期)"
}

install_no_car_sim() {
    [[ -n "${ROS_SETUP:-}" ]] || { warn "跳过 no-car sim（ROS 未构建）"; return; }
    [[ "${NO_CAR_MODE:-false}" == "true" ]] || { info "无小车模式未启用, 跳过模拟环境服务"; return; }
    local ros_source
    ros_source="$(ros_source_line)"

    cat > "/etc/systemd/system/${SVC_NO_CAR_SIM}" <<EOF
[Unit]
Description=ED UAV: 无小车模拟环境 (模拟 /fcu/state /localization/status /fcu/flight_command)
After=${SVC_HOTSPOT_WAIT}
Requires=${SVC_HOTSPOT_WAIT}

[Service]
Type=simple
Environment=HOME=/root
ExecStartPre=/bin/sleep 2
ExecStart=/bin/bash -c '${ros_source}; ros2 run ed_uav_bringup no_car_sim --ros-args -p state_rate_hz:=10.0'
Restart=always
RestartSec=3
StartLimitIntervalSec=0
StandardOutput=journal
StandardError=journal
SyslogIdentifier=ed-no-car-sim

[Install]
WantedBy=multi-user.target
EOF
    ok "无小车模拟环境服务: ${SVC_NO_CAR_SIM}"
}

install_mission_executor() {
    [[ -n "${ROS_SETUP:-}" ]] || { warn "跳过 mission executor（ROS 未构建）"; return; }
    [[ -n "${MISSION_CONFIG:-}" ]] || { warn "跳过 mission executor（未配置任务文件）"; return; }

    local profile="${PROFILE_PATH:-}"
    local sim="${SIMULATION_ONLY:-false}"
    local no_car="${NO_CAR_MODE:-false}"
    [[ "$no_car" == "true" ]] && sim="true"
    local ros_source
    ros_source="$(ros_source_line)"
    local after_unit="${SVC_GUARDIAN}"
    local requires_line="Requires=${SVC_GUARDIAN}"
    if [[ "$no_car" == "true" ]]; then
        after_unit="${SVC_NO_CAR_SIM} ${SVC_GUARDIAN}"
        requires_line="Requires=${SVC_NO_CAR_SIM} ${SVC_GUARDIAN}"
    fi

    # 标定文件: 真机 CALIBRATED; simulation_only 要求 SYNTHETIC, 否则 preflight 拒绝
    local calibration
    if [[ "$sim" == "true" ]]; then
        calibration="${REPO_ROOT}/ros2_ws/src/ed_uav_description/config/synthetic_calibrated.yaml"
    else
        calibration="${CALIBRATION_FILE:-${REPO_ROOT}/calibration_data/field_calibrated_v1.yaml}"
    fi

    cat > "/etc/systemd/system/${SVC_MISSION_EXECUTOR}" <<EOF
[Unit]
Description=ED UAV: ROS mission executor
After=${after_unit}
${requires_line}

[Service]
Type=simple
Environment=HOME=/root
ExecStartPre=/bin/sleep 3
ExecStart=/bin/bash -c '\
  ${ros_source}; \
  ros2 run ed_uav_mission mission_executor \
    --ros-args \
    -p mission_config_path:=${MISSION_CONFIG} \
    ${profile:+-p profile_path:=${profile}} \
    -p calibration_file:=${calibration} \
    -p simulation_only:=${sim} \
    -p task3_mission_profile_id:=task3-stability \
    -p task3_deployment_preset_id:=field-2026 \
    -p task3_target_revision:=d2026-apriltag-v1 \
    -r /vehicle/telemetry:=/d_task/vehicle/telemetry \
    -r /mission/status:=/d_task/mission_status \
    -r /mission/select_d_task:=/d_task/pre_arm/select_mission'
Restart=always
RestartSec=3
StartLimitIntervalSec=0
StandardOutput=journal
StandardError=journal
SyslogIdentifier=ed-mission

[Install]
WantedBy=multi-user.target
EOF
    ok "Mission executor 服务: ${SVC_MISSION_EXECUTOR} (simulation_only=${sim})"
}

install_diagnostic() {
    local diag_script="${REPO_ROOT}/tools/diagnostics/vehicle_comm_diagnostic.py"
    [[ -f "$diag_script" ]] || { warn "跳过诊断服务（脚本不存在）"; return; }

    local log_dir="/var/log/ed-uav"
    mkdir -p "$log_dir"

    # 自动查找密钥文件
    local effective_key="${HMAC_KEY_FILE:-}"
    if [[ -z "$effective_key" && -f "${REPO_ROOT}/config/hmac.key.hex" ]]; then
        effective_key="${REPO_ROOT}/config/hmac.key.hex"
    fi

    local key_arg=""
    [[ -n "$effective_key" ]] && key_arg="--key-file ${effective_key}"

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
    local default_key="${REPO_ROOT}/config/hmac.key.hex"
    [[ -f "$default_key" ]] && default_key_info=" (默认: $default_key)" || default_key_info=""
    read -rp "  HMAC 密钥文件 (十六进制)${default_key_info}: " input; HMAC_KEY_FILE="${input:-${HMAC_KEY_FILE:-$default_key}}"
    read -rp "  任务配置文件 (可选, 留空跳过 mission): " input; MISSION_CONFIG="${input:-${MISSION_CONFIG:-}}"
    read -rp "  场地配置文件 (可选): " input; PROFILE_PATH="${input:-${PROFILE_PATH:-}}"
    read -rp "  无小车模式? (true/false) [${NO_CAR_MODE:-false}]: " input; NO_CAR_MODE="${input:-${NO_CAR_MODE:-false}}"
    read -rp "  仅仿真模式? (true/false) [${SIMULATION_ONLY:-false}]: " input; SIMULATION_ONLY="${input:-${SIMULATION_ONLY:-false}}"

    echo ""
    info "安装服务"

    # 热点
    setup_hotspot

    # systemd
    install_wait_service
    install_guardian
    install_no_car_sim
    install_mission_executor
    install_diagnostic

    # 保存配置
    save_config

    # 重载并启用
    systemctl daemon-reload
    systemctl enable "$SVC_HOTSPOT_WAIT" >/dev/null 2>&1
    [[ -f "/etc/systemd/system/${SVC_GUARDIAN}" ]] && systemctl enable "$SVC_GUARDIAN" >/dev/null 2>&1
    [[ -f "/etc/systemd/system/${SVC_NO_CAR_SIM}" ]] && systemctl enable "$SVC_NO_CAR_SIM" >/dev/null 2>&1
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
    echo "    sudo journalctl -u ed-guardian -f  # 实时日志 (guardian + bridge)"
    echo "    sudo ./tools/install_boot.sh status   # 一键状态"
    echo ""
}

# ─── 卸载 ───────────────────────────────────────────────────────────────────
do_uninstall() {
    check_root
    info "卸载 ED UAV 开机自启服务"

    for svc in "$SVC_DIAGNOSTIC" "$SVC_MISSION_EXECUTOR" "$SVC_NO_CAR_SIM" "$SVC_GUARDIAN" "$SVC_HOTSPOT_WAIT"; do
        if systemctl is-enabled "$svc" &>/dev/null; then
            systemctl disable --now "$svc" 2>/dev/null || true
            ok "已禁用: $svc"
        fi
        rm -f "/etc/systemd/system/${svc}"
    done
    rm -f "${STATE_DIR}/guardian.conf"

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
    # 加载已保存的配置以获取 IFACE/STA_IFACE
    load_config 2>/dev/null || true

    echo ""
    echo -e "${B}${C}┌── ED UAV 开机自启状态 ────────────────────────────────┐${N}"

    # 热点
    if nmcli -t -f NAME connection show --active 2>/dev/null | grep -q "^${CON_NAME}:"; then
        echo -e "  热点:      ${G}● 运行中${N}  (SSID: ${SSID:-?})"
    else
        echo -e "  热点:      ${R}○ 未运行${N}"
    fi

    # 接口
    local iface="${IFACE:-}"
    if [[ -z "$iface" ]]; then
        iface=$(nmcli -t -f DEVICE,TYPE device status 2>/dev/null | grep ':wifi$' | head -1 | cut -d: -f1)
    fi
    if ip addr show "$iface" 2>/dev/null | grep -q "$NUC_IP"; then
        echo -e "  AP 接口:   ${G}${iface}${N} = ${NUC_IP}"
    else
        echo -e "  AP 接口:   ${Y}${iface:-?}${N} (无 IP)"
    fi
    if [[ -n "${STA_IFACE:-}" ]]; then
        local sta_ip
        sta_ip=$(ip -4 addr show "$STA_IFACE" 2>/dev/null | grep 'inet ' | awk '{print $2}' | head -1)
        echo -e "  STA 接口:  ${G}${STA_IFACE}${N} = ${sta_ip:-无IP} (互联网)"
    fi

    echo ""

    # systemd 服务
    for svc in "$SVC_HOTSPOT_WAIT" "$SVC_GUARDIAN" "$SVC_NO_CAR_SIM" "$SVC_MISSION_EXECUTOR" "$SVC_DIAGNOSTIC"; do
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
    for svc in "$SVC_HOTSPOT_WAIT" "$SVC_GUARDIAN" "$SVC_NO_CAR_SIM" "$SVC_MISSION_EXECUTOR" "$SVC_DIAGNOSTIC"; do
        [[ -f "/etc/systemd/system/${svc}" ]] && systemctl enable "$svc" 2>/dev/null && ok "已启用: ${svc%.service}"
    done
}

do_disable() {
    check_root
    for svc in "$SVC_HOTSPOT_WAIT" "$SVC_GUARDIAN" "$SVC_NO_CAR_SIM" "$SVC_MISSION_EXECUTOR" "$SVC_DIAGNOSTIC"; do
        systemctl disable "$svc" 2>/dev/null && ok "已禁用: ${svc%.service}" || true
    done
}

# ─── 启动/停止（立即） ─────────────────────────────────────────────────────
do_start() {
    check_root
    nmcli connection up "$CON_NAME" >/dev/null 2>&1 || true
    for svc in "$SVC_HOTSPOT_WAIT" "$SVC_GUARDIAN" "$SVC_NO_CAR_SIM" "$SVC_MISSION_EXECUTOR" "$SVC_DIAGNOSTIC"; do
        [[ -f "/etc/systemd/system/${svc}" ]] && systemctl start "$svc" 2>/dev/null && ok "已启动: ${svc%.service}"
    done
}

do_stop() {
    check_root
    for svc in "$SVC_DIAGNOSTIC" "$SVC_MISSION_EXECUTOR" "$SVC_NO_CAR_SIM" "$SVC_GUARDIAN" "$SVC_HOTSPOT_WAIT"; do
        systemctl stop "$svc" 2>/dev/null && ok "已停止: ${svc%.service}" || true
    done
}

do_restart() {
    do_stop
    do_start
}

# ─── 日志 ───────────────────────────────────────────────────────────────────
do_logs() {
    local svc="${1:-all}"
    case "$svc" in
        bridge|vehicle-bridge|guardian) journalctl -u "$SVC_GUARDIAN" -f ;;
        sim|no-car-sim)                journalctl -u "$SVC_NO_CAR_SIM" -f ;;
        mission)                       journalctl -u "$SVC_MISSION_EXECUTOR" -f ;;
        diag|diagnostic)               journalctl -u "$SVC_DIAGNOSTIC" -f ;;
        all)                           journalctl -u "ed-*" -f ;;
        *)                             journalctl -u "$svc" -f ;;
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
            echo "    logs [svc] 查看实时日志 (guardian/sim/mission/diag/all)"
            echo ""
            ;;
        *)
            die "未知命令: $cmd (运行 $0 help)"
            ;;
    esac
}

main "$@"
