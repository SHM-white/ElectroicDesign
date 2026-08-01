#!/bin/sh
# brcmfmac-reset.sh — BCM43569 USB 无线网卡开机驱动恢复
#
# 由 udev 规则 99-brcmfmac-reset.rules 触发。
# 延迟后重置 USB 端口，让设备以正常模式重新枚举，brcmfmac 即可成功 probe。
#
# 环境变量（由 udev RUN 设置）:
#   DEVPATH  — USB 设备的 sysfs 绝对路径，如 /devices/pci…/usb2/2-4

set -e

LOGFILE="/var/log/brcmfmac-reset.log"
log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') [brcmfmac-reset] $*" >> "$LOGFILE" 2>/dev/null || true
}

log "触发: DEVPATH=$DEVPATH"

# /sys$DEVPATH 就是该 USB 设备的 sysfs 目录
DEV="/sys${DEVPATH}"
AUTH="$DEV/authorized"

if [ ! -f "$AUTH" ]; then
    log "警告: $AUTH 不存在，跳过"
    exit 0
fi

# 等待 3 秒让固件加载流程完成（或失败）
sleep 3

# 如果 brcmfmac 已经绑定成功，不需要重置
if [ -e "$DEV/driver" ] && [ "$(basename "$(readlink "$DEV/driver" 2>/dev/null)")" = "brcmfmac" ]; then
    log "brcmfmac 已绑定，无需重置"
    exit 0
fi

log "brcmfmac 未绑定，执行 USB 端口重置"

# 禁用设备（触发 disconnect）
echo 0 > "$AUTH"
sleep 1

# 重新启用设备（触发重新枚举）
echo 1 > "$AUTH"

log "重置完成，等待重新枚举"
