#!/usr/bin/env bash
# ==============================================================================
# FCU Serial Port Test
#
# Test FCU serial port connectivity and detect Lingxiao flight controller.
#
# Usage:
#   ./tools/test_fcu_serial.sh [OPTIONS]
# ==============================================================================

set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ─── Colors ──────────────────────────────────────────────────────────────────
R='\033[0;31m'; G='\033[0;32m'; Y='\033[0;33m'; C='\033[0;36m'; B='\033[1m'; N='\033[0m'
ok()   { echo -e "${G}[OK]${N}  $*"; }
warn() { echo -e "${Y}[!!]${N}  $*"; }
fail() { echo -e "${R}[ERR]${N} $*" >&2; }
die()  { fail "$*"; exit 64; }

# ─── Defaults ────────────────────────────────────────────────────────────────
SERIAL_PORT="${FCU_SERIAL_PORT:-/dev/ttyUSB0}"
BAUDRATE=500000
TIMEOUT=5
DRY_RUN=0

# ─── Argument parsing ───────────────────────────────────────────────────────
usage() {
    cat <<'EOF'
Usage: ./tools/test_fcu_serial.sh [OPTIONS]

Test FCU serial port connectivity and detect Lingxiao flight controller.

Options:
  --serial PATH      FCU serial device path (default: /dev/ttyUSB0)
  --baudrate INT     Baud rate (default: 500000)
  --timeout INT      Timeout in seconds (default: 5)
  -d, --dry-run      Only check if device exists, don't open serial port
  -h, --help         Show this help
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --serial)       SERIAL_PORT="$2"; shift 2 ;;
        --baudrate)     BAUDRATE="$2"; shift 2 ;;
        --timeout)      TIMEOUT="$2"; shift 2 ;;
        -d|--dry-run)   DRY_RUN=1; shift ;;
        -h|--help)      usage ;;
        *)              die "未知参数: $1; 使用 --help 查看帮助" ;;
    esac
done

# ─── Device existence check ─────────────────────────────────────────────────
echo "═══════════════════════════════════════════════════════"
echo " FCU 串口测试"
echo "═══════════════════════════════════════════════════════"
echo ""

echo "[1/4] 检查串口设备..."
if [[ ! -e "$SERIAL_PORT" ]]; then
    fail "串口设备不存在: $SERIAL_PORT"
    echo ""
    echo "提示:"
    echo "  1. 检查飞控是否已连接 USB"
    echo "  2. 检查设备权限: ls -la $SERIAL_PORT"
    echo "  3. 可能需要添加用户到 dialout 组: sudo usermod -aG dialout \$USER"
    echo "  4. 可用串口列表:"
    ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "     (无可用串口)"
    exit 1
fi
ok "串口设备存在: $SERIAL_PORT"

# Check device permissions
if [[ ! -r "$SERIAL_PORT" || ! -w "$SERIAL_PORT" ]]; then
    warn "串口设备权限不足 (需要读写权限)"
    echo "  当前权限: $(ls -la "$SERIAL_PORT" | awk '{print $1, $3, $4}')"
    echo "  修复方法: sudo chmod 666 $SERIAL_PORT"
    echo "  或添加用户到 dialout 组: sudo usermod -aG dialout \$USER"
fi

if ((DRY_RUN)); then
    echo ""
    echo "[DRY-RUN] 仅检查设备存在性，跳过串口通信测试"
    exit 0
fi

# ─── Serial port info ───────────────────────────────────────────────────────
echo ""
echo "[2/4] 串口设备信息..."
echo "  设备: $SERIAL_PORT"
echo "  波特率: $BAUDRATE"
echo "  超时: ${TIMEOUT}s"

# Check if stty is available
if command -v stty >/dev/null 2>&1; then
    echo "  当前配置:"
    stty -F "$SERIAL_PORT" 2>/dev/null | head -5 || warn "无法读取串口配置"
fi

# ─── Try to detect flight controller ───────────────────────────────────────
echo ""
echo "[3/4] 尝试检测飞控..."

# Create a temporary Python script for serial communication
PYTHON_SCRIPT=$(mktemp /tmp/test_fcu_serial.XXXXXX.py)
cat > "$PYTHON_SCRIPT" << 'PYEOF'
#!/usr/bin/env python3
"""Test FCU serial port connectivity."""

import sys
import time
import struct

def main():
    port = sys.argv[1]
    baudrate = int(sys.argv[2])
    timeout = int(sys.argv[3])
    
    try:
        import serial
    except ImportError:
        print("ERROR: pyserial 未安装，请运行: pip install pyserial")
        sys.exit(2)
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"  串口已打开: {port} @ {baudrate}")
    except serial.SerialException as e:
        print(f"  ERROR: 无法打开串口: {e}")
        sys.exit(1)
    
    # Clear input buffer
    ser.reset_input_buffer()
    
    # Wait for data from flight controller
    print(f"  等待飞控数据 ({timeout}s)...")
    start_time = time.time()
    data_received = bytearray()
    
    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            data_received.extend(data)
            print(f"  收到 {len(data)} 字节")
            
            # Check for V7 telemetry frame (starts with 0xAA 0x55)
            if len(data_received) >= 2:
                for i in range(len(data_received) - 1):
                    if data_received[i] == 0xAA and data_received[i+1] == 0x55:
                        print(f"  DETECT: 检测到凌霄飞控 V7 遥测帧 (位置 {i})")
                        ser.close()
                        sys.exit(0)
        
        time.sleep(0.1)
    
    if len(data_received) > 0:
        print(f"  收到数据但未检测到 V7 遥测帧")
        print(f"  数据预览: {data_received[:20].hex()}")
    else:
        print(f"  未收到任何数据")
    
    ser.close()
    sys.exit(1)

if __name__ == "__main__":
    main()
PYEOF

# Run the Python script
if python3 "$PYTHON_SCRIPT" "$SERIAL_PORT" "$BAUDRATE" "$TIMEOUT"; then
    ok "飞控检测成功"
else
    EXIT_CODE=$?
    if [[ $EXIT_CODE -eq 2 ]]; then
        fail "pyserial 未安装"
        echo "  修复方法: pip install pyserial"
    else
        warn "未检测到飞控响应"
        echo ""
        echo "可能原因:"
        echo "  1. 飞控未上电"
        echo "  2. 串口线缆连接错误"
        echo "  3. 波特率不匹配 (当前: $BAUDRATE)"
        echo "  4. 飞控固件未正确烧录"
        echo "  5. 飞控正在等待其他指令"
    fi
fi

# Cleanup
rm -f "$PYTHON_SCRIPT"

# ─── Summary ────────────────────────────────────────────────────────────────
echo ""
echo "[4/4] 测试完成"
echo ""
echo "═══════════════════════════════════════════════════════"
