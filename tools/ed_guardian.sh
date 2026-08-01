#!/usr/bin/env bash
# ==============================================================================
# ED UAV 最底层守护进程
#
# 职责: 监控地面站通信模块 (vehicle_bridge), 崩溃后自动重新拉起并记录日志。
# 契约: 本进程永不退出 —— 任何错误都被记录后继续循环; 若本进程自身被杀,
#       由上层 systemd (Restart=always) 重新拉起。
#
# 环境变量:
#   ED_GUARDIAN_LOG_DIR        日志目录 (默认 /var/log/ed-uav)
#   ED_GUARDIAN_WATCH_NAME     被监控进程标识 (默认 ed_uav_vehicle_bridge)
#   ED_GUARDIAN_START_CMD      拉起命令 (必填, 建议为 ros2 run ...)
#   ED_GUARDIAN_RESTART_DELAY  重启间隔秒数 (默认 3, 连续失败自动退避到 30s)
#
# 日志:
#   $LOG_DIR/guardian.log      守护进程事件日志
#   $LOG_DIR/<WATCH_NAME>.log  被监控进程输出 (追加)
#   $LOG_DIR/<WATCH_NAME>.pid  被监控进程 PID
# ==============================================================================

set +euo pipefail  # 守护契约: 任何错误都不能让本进程退出

LOG_DIR="${ED_GUARDIAN_LOG_DIR:-/var/log/ed-uav}"
WATCH_NAME="${ED_GUARDIAN_WATCH_NAME:-ed_uav_vehicle_bridge}"
START_CMD="${ED_GUARDIAN_START_CMD:-}"
RESTART_DELAY="${ED_GUARDIAN_RESTART_DELAY:-3}"
PID_FILE="${ED_GUARDIAN_PID_FILE:-$LOG_DIR/$WATCH_NAME.pid}"
GUARDIAN_LOG="$LOG_DIR/guardian.log"

mkdir -p "$LOG_DIR" 2>/dev/null || LOG_DIR="/tmp/ed-uav-guardian"

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [guardian] $*" >> "$GUARDIAN_LOG"
}

watch_alive() {
    [[ -f "$PID_FILE" ]] || return 1
    local pid
    pid="$(cat "$PID_FILE" 2>/dev/null || echo '')"
    [[ -n "$pid" ]] || return 1
    kill -0 "$pid" 2>/dev/null
}

spawn_watched() {
    if watch_alive; then
        log "SKIP: $WATCH_NAME 已在运行 (pid=$(cat "$PID_FILE" 2>/dev/null))"
        return 0
    fi
    if [[ -z "$START_CMD" ]]; then
        log "ERROR: ED_GUARDIAN_START_CMD 为空, 无法拉起 $WATCH_NAME"
        return 1
    fi
    rm -f "$PID_FILE"
    setsid bash -c "$START_CMD; echo '[guardian] watched_exit='\$? >> '$GUARDIAN_LOG'" >> "$LOG_DIR/$WATCH_NAME.log" 2>&1 &
    echo $! > "$PID_FILE"
    log "SPAWNED: $WATCH_NAME pid=$! 日志=$LOG_DIR/$WATCH_NAME.log"
}

FAIL_COUNT=0
SPAWN_AT=0

log "START: watch=$WATCH_NAME delay=${RESTART_DELAY}s log=$GUARDIAN_LOG"

spawn_watched

while true; do
    if watch_alive; then
        # 存活超过 60s 视为稳定, 重置失败计数
        if (( FAIL_COUNT > 0 )) && (( $(date +%s) - SPAWN_AT > 60 )); then
            FAIL_COUNT=0
            log "STABLE: $WATCH_NAME 已稳定运行, 失败计数清零"
        fi
    else
        FAIL_COUNT=$((FAIL_COUNT + 1))
        delay="$RESTART_DELAY"
        if (( FAIL_COUNT > 10 )); then delay=30
        elif (( FAIL_COUNT > 5 )); then delay=15
        fi
        log "DOWN: $WATCH_NAME 已退出(连续第 ${FAIL_COUNT} 次), ${delay}s 后重新拉起"
        spawn_watched
        SPAWN_AT="$(date +%s)"
        sleep "$delay"
    fi
    sleep 1
done
