#!/usr/bin/env bash
# ==============================================================================
# ED UAV 一键清理残余进程
#
# 清理范围：
#   - ROS2 launch / node 进程（vehicle_bridge, mission_executor, camera 等）
#   - 诊断/模拟工具（vehicle_comm_diagnostic, sim_competition, sim_network）
#   - guardian 守护进程
#   - 占用 UDP 42000 端口的进程
#   - orphaned ros2 daemon
#
# 用法：
#   ./tools/cleanup.sh          # 清理（交互确认）
#   ./tools/cleanup.sh -y       # 清理（跳过确认）
#   ./tools/cleanup.sh -n       # 仅显示，不清理
#   ./tools/cleanup.sh -h       # 帮助
# ==============================================================================

set -euo pipefail

R='\033[0;31m'; G='\033[0;32m'; Y='\033[0;33m'; C='\033[0;36m'; B='\033[1m'; N='\033[0m'

DRY_RUN=false
AUTO_YES=false

usage() {
    echo "用法: $0 [-y] [-n] [-h]"
    echo "  -y  跳过确认，直接清理"
    echo "  -n  仅显示，不清理"
    echo "  -h  帮助"
}

while getopts "ynh" opt; do
    case $opt in
        y) AUTO_YES=true ;;
        n) DRY_RUN=true ;;
        h) usage; exit 0 ;;
        *) usage; exit 1 ;;
    esac
done

# ─── 收集目标进程 ────────────────────────────────────────────────────────────

collect_pids() {
    local -a patterns=(
        # ROS2 nodes
        "vehicle_bridge"
        "mission_executor"
        "direct_uvc"
        "ed_uav_"
        # ROS2 launch
        "ros2 launch"
        "ros2 run"
        # 诊断/模拟
        "vehicle_comm_diagnostic"
        "sim_competition"
        "sim_network"
        "no_car_sim"
        # guardian
        "ed_guardian"
        # 其他
        "full_competition"
    )

    FOUND_PIDS=()
    FOUND_INFO=()

    for pat in "${patterns[@]}"; do
        while IFS= read -r line; do
            [[ -z "$line" ]] && continue
            local pid cmd
            pid=$(echo "$line" | awk '{print $1}')
            cmd=$(echo "$line" | awk '{for(i=2;i<=NF;i++) printf "%s ", $i; print ""}')
            # 跳过自身
            [[ "$pid" == "$$" ]] && continue
            # 去重
            for existing in "${FOUND_PIDS[@]+"${FOUND_PIDS[@]}"}"; do
                [[ "$existing" == "$pid" ]] && continue 2
            done
            FOUND_PIDS+=("$pid")
            FOUND_INFO+=("$pid  $cmd")
        done < <(ps aux 2>/dev/null | grep -E "$pat" | grep -v grep | awk '{print $2, $11, $12, $13, $14}')
    done
}

# ─── 检查 42000 端口 ─────────────────────────────────────────────────────────

collect_port_pids() {
    PORT_PIDS=()
    while IFS= read -r line; do
        [[ -z "$line" ]] && continue
        local pid
        pid=$(echo "$line" | grep -oP 'pid=\K[0-9]+' || true)
        if [[ -n "$pid" ]]; then
            for existing in "${PORT_PIDS[@]+"${PORT_PIDS[@]}"}"; do
                [[ "$existing" == "$pid" ]] && continue 2
            done
            # 排除已在 FOUND_PIDS 中的
            for existing in "${FOUND_PIDS[@]+"${FOUND_PIDS[@]}"}"; do
                [[ "$existing" == "$pid" ]] && continue 2
            done
            PORT_PIDS+=("$pid")
        fi
    done < <(ss -ulnp 2>/dev/null | grep ":42000 ")
}

# ─── 显示 ────────────────────────────────────────────────────────────────────

show_results() {
    echo ""
    echo -e "${B}${C}┌── ED UAV 残余进程扫描 ──────────────────────────────┐${N}"

    if [[ ${#FOUND_PIDS[@]} -eq 0 && ${#PORT_PIDS[@]} -eq 0 ]]; then
        echo -e "  ${G}✓ 干净，无残余进程${N}"
        echo -e "${C}└──────────────────────────────────────────────────────┘${N}"
        echo ""
        return 1
    fi

    if [[ ${#FOUND_PIDS[@]} -gt 0 ]]; then
        echo ""
        echo -e "  ${Y}ROS / 诊断 / 模拟进程:${N}"
        for info in "${FOUND_INFO[@]}"; do
            local pid cmd
            pid=$(echo "$info" | awk '{print $1}')
            cmd=$(echo "$info" | awk '{for(i=2;i<=NF;i++) printf "%s ", $i}')
            printf "    ${R}%-7s${N} %s\n" "$pid" "$cmd"
        done
    fi

    if [[ ${#PORT_PIDS[@]} -gt 0 ]]; then
        echo ""
        echo -e "  ${Y}占用 UDP 42000 的进程:${N}"
        for pid in "${PORT_PIDS[@]}"; do
            local cmd
            cmd=$(ps -p "$pid" -o args= 2>/dev/null || echo "(未知)")
            printf "    ${R}%-7s${N} %s\n" "$pid" "$cmd"
        done
    fi

    echo ""
    echo -e "  共 ${B}${#FOUND_PIDS[@]}${N} 个进程, ${B}${#PORT_PIDS[@]}${N} 个端口占用"
    echo -e "${C}└──────────────────────────────────────────────────────┘${N}"
    echo ""
    return 0
}

# ─── 执行清理 ────────────────────────────────────────────────────────────────

do_kill() {
    local -a all_pids=("${FOUND_PIDS[@]+"${FOUND_PIDS[@]}"}" "${PORT_PIDS[@]+"${PORT_PIDS[@]}"}")

    if [[ ${#all_pids[@]} -eq 0 ]]; then
        echo -e "${G}无需清理${N}"
        return
    fi

    # 先 SIGTERM（优雅退出）
    echo -e "${Y}发送 SIGTERM ...${N}"
    for pid in "${all_pids[@]}"; do
        kill "$pid" 2>/dev/null && printf "  TERM %-7s\n" "$pid" || true
    done
    sleep 2

    # 检查哪些还活着，SIGKILL
    local -a alive=()
    for pid in "${all_pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            alive+=("$pid")
        fi
    done

    if [[ ${#alive[@]} -gt 0 ]]; then
        echo -e "${R}发送 SIGKILL ...${N}"
        for pid in "${alive[@]}"; do
            kill -9 "$pid" 2>/dev/null && printf "  KILL %-7s\n" "$pid" || true
        done
        sleep 1
    fi

    # 清理 ros2 daemon
    ros2 daemon stop 2>/dev/null || true

    echo -e "${G}清理完成${N}"
}

# ─── 主流程 ──────────────────────────────────────────────────────────────────

collect_pids
collect_port_pids

if ! show_results; then
    exit 0
fi

if $DRY_RUN; then
    echo -e "${C}（仅显示模式，未执行清理）${N}"
    exit 0
fi

if ! $AUTO_YES; then
    read -rp "确认清理以上进程? [y/N] " ans
    case "$ans" in
        [yY]|[yY][eE][sS]) ;;
        *) echo "已取消"; exit 0 ;;
    esac
fi

do_kill
