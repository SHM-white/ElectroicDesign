#!/usr/bin/env bash
# ==============================================================================
# Car UDP Link Test
#
# Test UDP communication link with the car (small vehicle).
# Sends heartbeat packets and waits for car telemetry response.
#
# Usage:
#   ./tools/test_car_link.sh [OPTIONS]
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
CAR_IP="192.168.20.2"
CAR_PORT=42001
NUC_IP="192.168.20.1"
NUC_PORT=42000
HMI_IP="192.168.20.3"
HMI_PORT=42000
TIMEOUT=10
NO_HMAC=0

# ─── Argument parsing ───────────────────────────────────────────────────────
usage() {
    cat <<'EOF'
Usage: ./tools/test_car_link.sh [OPTIONS]

Test UDP communication link with the car (small vehicle).

Options:
  --car-ip IP        Car IP address (default: 192.168.20.2)
  --car-port PORT    Car UDP port (default: 42001)
  --nuc-ip IP        NUC IP address (default: 192.168.20.1)
  --nuc-port PORT    NUC UDP port (default: 42000)
  --timeout INT      Timeout in seconds (default: 10)
  --no-hmac          Skip HMAC verification (use default key)
  -h, --help         Show this help
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --car-ip)       CAR_IP="$2"; shift 2 ;;
        --car-port)     CAR_PORT="$2"; shift 2 ;;
        --nuc-ip)       NUC_IP="$2"; shift 2 ;;
        --nuc-port)     NUC_PORT="$2"; shift 2 ;;
        --timeout)      TIMEOUT="$2"; shift 2 ;;
        --no-hmac)      NO_HMAC=1; shift ;;
        -h|--help)      usage ;;
        *)              die "未知参数: $1; 使用 --help 查看帮助" ;;
    esac
done

# ─── Network connectivity check ─────────────────────────────────────────────
echo "═══════════════════════════════════════════════════════"
echo " 小车 UDP 链路测试"
echo "═══════════════════════════════════════════════════════"
echo ""

echo "[1/5] 检查网络配置..."
echo "  NUC (本机): $NUC_IP:$NUC_PORT"
echo "  小车:       $CAR_IP:$CAR_PORT"
echo "  HMI:        $HMI_IP:$HMI_PORT"

# Check if we can ping the car
echo ""
echo "[2/5] 检查小车网络连通性..."
if ping -c 1 -W 2 "$CAR_IP" >/dev/null 2>&1; then
    ok "小车网络可达: $CAR_IP"
else
    warn "小车网络不可达: $CAR_IP"
    echo ""
    echo "可能原因:"
    echo "  1. 小车未开机"
    echo "  2. 小车未连接到同一网络"
    echo "  3. 防火墙阻止 ICMP"
    echo "  4. IP 地址配置错误"
    echo ""
    echo "继续测试 UDP..."
fi

# ─── Check if ports are available ───────────────────────────────────────────
echo ""
echo "[3/5] 检查端口可用性..."

# Check if NUC port is already in use
if ss -uln | grep -q ":$NUC_PORT "; then
    warn "NUC 端口 $NUC_PORT 已被占用"
    echo "  占用进程:"
    ss -ulnp | grep ":$NUC_PORT " || true
    echo ""
    echo "  可能需要停止其他通信进程"
else
    ok "NUC 端口 $NUC_PORT 可用"
fi

# ─── Create Python test script ─────────────────────────────────────────────
echo ""
echo "[4/5] 启动 UDP 链路测试..."

PYTHON_SCRIPT=$(mktemp /tmp/test_car_link.XXXXXX.py)
cat > "$PYTHON_SCRIPT" << 'PYEOF'
#!/usr/bin/env python3
"""Test car UDP communication link."""

import sys
import time
import struct
import socket
import hashlib
import hmac as hmac_mod

# Protocol constants
MAGIC = 0x4454
PROTOCOL_VERSION = 1
HEADER_STRUCT = struct.Struct("<HBBH I I I I")
HEADER_SIZE = 22
CRC_SIZE = 2
HMAC_SIZE = 8
MAX_PAYLOAD = 64
MAX_PACKET = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE + HMAC_SIZE

MSG_HEARTBEAT = 1
MSG_CAR_TELEMETRY = 2
MSG_TASK_SELECTION = 3
MSG_MISSION_STATUS = 4

SENDER_ROS = 0x524F5331  # "ROS1"
SENDER_CAR = 0x43415231  # "CAR1"
SENDER_HMI = 0x484D4931  # "HMI1"


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode_packet(msg_type: int, sender_id: int, boot_id: int, seq: int,
                  source_ms: int, payload: bytes, key: bytes) -> bytes:
    header = HEADER_STRUCT.pack(MAGIC, PROTOCOL_VERSION, msg_type, len(payload),
                                sender_id, boot_id, seq, source_ms)
    packet = header + payload
    crc = crc16_ccitt(packet)
    packet += struct.pack("<H", crc)
    # HMAC disabled - use zero tag
    packet += b'\x00' * HMAC_SIZE
    return packet


def decode_packet(raw: bytes, key: bytes):
    if len(raw) < HEADER_SIZE + CRC_SIZE + HMAC_SIZE:
        return None
    if len(raw) > MAX_PACKET:
        return None
    
    # Skip HMAC verification
    hp = raw[:len(raw) - CRC_SIZE - HMAC_SIZE]
    crc_bytes = raw[len(raw) - CRC_SIZE - HMAC_SIZE:len(raw) - HMAC_SIZE]
    
    if struct.unpack("<H", crc_bytes)[0] != crc16_ccitt(hp):
        return None
    
    magic, ver, msg_type, plen, sender, boot, seq, src_ms = HEADER_STRUCT.unpack(hp[:HEADER_SIZE])
    if magic != MAGIC or ver != PROTOCOL_VERSION:
        return None
    if plen > MAX_PAYLOAD or len(hp) != HEADER_SIZE + plen:
        return None
    
    return (msg_type, sender, boot, seq, src_ms, hp[HEADER_SIZE:])


def main():
    nuc_ip = sys.argv[1]
    nuc_port = int(sys.argv[2])
    car_ip = sys.argv[3]
    car_port = int(sys.argv[4])
    timeout = int(sys.argv[5])
    no_hmac = int(sys.argv[6])
    
    # Use default key (HMAC disabled)
    key = b'\x00' * 32
    
    # Create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((nuc_ip, nuc_port))
    sock.settimeout(1.0)
    
    print(f"  UDP socket 已绑定: {nuc_ip}:{nuc_port}")
    print(f"  目标小车: {car_ip}:{car_port}")
    print(f"  超时: {timeout}s")
    print("")
    
    # Send heartbeat packets
    boot_id = 0x12345678
    seq = 0
    start_time = time.time()
    car_detected = False
    car_boot_id = None
    packets_sent = 0
    packets_received = 0
    
    print("  发送心跳包...")
    
    while time.time() - start_time < timeout:
        # Send heartbeat
        now_ms = int(time.time() * 1000) & 0xFFFFFFFF
        heartbeat = encode_packet(MSG_HEARTBEAT, SENDER_ROS, boot_id, seq, now_ms, b"", key)
        
        try:
            sock.sendto(heartbeat, (car_ip, car_port))
            packets_sent += 1
        except OSError as e:
            print(f"  发送失败: {e}")
        
        # Try to receive response
        try:
            data, addr = sock.recvfrom(MAX_PACKET)
            hdr = decode_packet(data, key)
            if hdr is not None:
                msg_type, sender, recv_boot, recv_seq, src_ms, payload = hdr
                
                if sender == SENDER_CAR:
                    packets_received += 1
                    if not car_detected:
                        car_detected = True
                        car_boot_id = recv_boot
                        print(f"  检测到小车响应! boot_id=0x{recv_boot:08X}")
                    
                    if msg_type == MSG_CAR_TELEMETRY:
                        # Decode telemetry
                        if len(payload) >= 18:
                            state, turn, event, event_id, quality, disp_mm, vel_mm, line_err, faults = \
                                struct.unpack("<BBBHHihhH", payload[:18])
                            print(f"  遥测: state={state} turn={turn} event={event} "
                                  f"disp={disp_mm/1000:.2f}m vel={vel_mm/1000:.2f}m/s")
        except socket.timeout:
            pass
        except OSError as e:
            print(f"  接收错误: {e}")
        
        seq += 1
        time.sleep(0.25)  # 4Hz
    
    # Summary
    print("")
    print("  ─── 测试结果 ───")
    print(f"  发送包数: {packets_sent}")
    print(f"  接收包数: {packets_received}")
    
    if car_detected:
        print(f"  小车 boot_id: 0x{car_boot_id:08X}")
        print(f"  丢包率: {(1 - packets_received/packets_sent)*100:.1f}%")
        sock.close()
        sys.exit(0)
    else:
        print(f"  未检测到小车响应")
        sock.close()
        sys.exit(1)


if __name__ == "__main__":
    main()
PYEOF

# Run the Python script
if python3 "$PYTHON_SCRIPT" "$NUC_IP" "$NUC_PORT" "$CAR_IP" "$CAR_PORT" "$TIMEOUT" "$NO_HMAC"; then
    ok "小车链路测试成功"
else
    EXIT_CODE=$?
    fail "小车链路测试失败"
    echo ""
    echo "可能原因:"
    echo "  1. 小车未开机或未连接网络"
    echo "  2. 小车 UDP 服务未启动"
    echo "  3. IP 地址或端口配置错误"
    echo "  4. 防火墙阻止 UDP 通信"
    echo "  5. 小车固件未正确配置"
fi

# Cleanup
rm -f "$PYTHON_SCRIPT"

# ─── Summary ────────────────────────────────────────────────────────────────
echo ""
echo "[5/5] 测试完成"
echo ""
echo "═══════════════════════════════════════════════════════"
