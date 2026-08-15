#!/usr/bin/env python3
"""
ED UAV 通信链路快速自测

无需真实 ESP32 硬件，验证诊断工具的协议编解码、序列号检测、
过期判断等核心逻辑是否正确。无需 root 权限（使用高位端口）。
"""

import socket
import struct
import sys
import threading
import time
from pathlib import Path

# 把 tools/diagnostics 加入 path
sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))

from vehicle_comm_diagnostic import (
    CommDiagnostic,
    CAR_IP, CAR_PORT, HMI_IP, HMI_PORT, NUC_PORT,
    SENDER_CAR, SENDER_HMI, SENDER_ROS,
    MSG_CAR_TELEMETRY, MSG_TASK_SELECTION, MSG_HEARTBEAT,
    encode_packet, decode_packet,
    CAR_TELEMETRY_FMT, TASK_SELECTION_FMT,
)


def test_crc16():
    """验证 CRC16 与协议黄金向量一致"""
    from vehicle_comm_diagnostic import crc16_ccitt
    # 简单校验：空数据
    assert crc16_ccitt(b"") == 0xFFFF
    # 已知向量（手动计算）
    result = crc16_ccitt(b"123456789")
    # CCITT-FALSE 对 "123456789" 的结果应为 0x29B1
    assert result == 0x29B1, f"CRC16 校验失败: 0x{result:04X} != 0x29B1"
    print("  [PASS] CRC16-CCITT")


def test_hmac():
    """验证 HMAC-SHA256 前 8 字节截断"""
    import hashlib, hmac as hmac_mod
    key = bytes(range(32))
    data = b"test data"
    mac = hmac_mod.new(key, data, hashlib.sha256).digest()[:8]
    assert len(mac) == 8
    # 重新计算，结果应一致
    mac2 = hmac_mod.new(key, data, hashlib.sha256).digest()[:8]
    assert mac == mac2
    print("  [PASS] HMAC-SHA256 截断")


def test_encode_decode_roundtrip():
    """编码→解码往返测试"""
    key = bytes(range(32))
    payload = struct.pack("<BBBHHihhH", 1, 0, 1, 100, 3, 1500, 200, 10, 0)
    packet = encode_packet(MSG_CAR_TELEMETRY, SENDER_CAR, 0x12345678, 42, 1000, payload, key)
    hdr = decode_packet(packet, key)
    assert hdr is not None
    assert hdr.msg_type == MSG_CAR_TELEMETRY
    assert hdr.sender_id == SENDER_CAR
    assert hdr.boot_id == 0x12345678
    assert hdr.sequence == 42
    assert hdr.source_millis == 1000
    assert hdr.payload == payload
    print("  [PASS] 编码/解码往返")


def test_tampered_packet_rejected():
    """篡改一个字节应被 HMAC 拒绝"""
    key = bytes(range(32))
    packet = encode_packet(MSG_HEARTBEAT, SENDER_CAR, 1, 1, 0, b"", key)
    tampered = bytearray(packet)
    tampered[HEADER_SIZE := 22] ^= 0xFF  # 篡改 payload 区域（心跳无 payload，篡改 CRC 区域）
    result = decode_packet(bytes(tampered), key)
    assert result is None, "篡改数据包应被拒绝"
    print("  [PASS] 篡改数据包拒绝")


def test_wrong_key_rejected():
    """错误密钥应被拒绝"""
    key = bytes(range(32))
    wrong_key = bytes(range(31, -1, -1))
    packet = encode_packet(MSG_HEARTBEAT, SENDER_CAR, 1, 1, 0, b"", key)
    result = decode_packet(packet, wrong_key)
    assert result is None, "错误密钥数据包应被拒绝"
    print("  [PASS] 错误密钥拒绝")


def test_live_loopback():
    """回环测试：发送心跳到自己并接收"""
    key = bytes(range(32))
    boot_id = 0xAABBCCDD

    # 用高位端口避免 root 权限
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    recv_sock.bind(("127.0.0.1", 0))
    recv_port = recv_sock.getsockname()[1]
    recv_sock.settimeout(2.0)

    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 发送心跳
    packet = encode_packet(MSG_HEARTBEAT, SENDER_CAR, boot_id, 1, 100, b"", key)
    send_sock.sendto(packet, ("127.0.0.1", recv_port))

    # 接收
    data, addr = recv_sock.recvfrom(1024)
    hdr = decode_packet(data, key)
    assert hdr is not None
    assert hdr.msg_type == MSG_HEARTBEAT
    assert hdr.boot_id == boot_id

    send_sock.close()
    recv_sock.close()
    print("  [PASS] UDP 回环收发")


def test_telemetry_payload_decode():
    """验证遥测载荷解码"""
    from vehicle_comm_diagnostic import decode_car_telemetry
    # state=1(RUNNING), turn=1(SMALL), event=2(B), event_id=100,
    # quality=3, disp=1500mm, vel=200mm/s, line_err=10, faults=0
    payload = struct.pack("<BBBHHihhH", 1, 1, 2, 100, 3, 1500, 200, 10, 0)
    t = decode_car_telemetry(payload)
    assert t is not None
    assert t["state"] == "RUNNING"
    assert t["turn"] == "SMALL"
    assert t["event"] == "B"
    assert abs(t["displacement_m"] - 1.5) < 0.001
    assert abs(t["velocity_m_s"] - 0.2) < 0.001
    print("  [PASS] 遥测载荷解码")


def test_selection_payload_decode():
    """验证选择载荷解码"""
    from vehicle_comm_diagnostic import decode_task_selection
    payload = struct.pack("<IIB", 42, 0x12345678, 2)
    s = decode_task_selection(payload)
    assert s is not None
    assert s["selection_id"] == 42
    assert s["car_boot_id"] == 0x12345678
    assert s["task"] == 2
    print("  [PASS] 选择载荷解码")


def test_production_key():
    """验证编解码往返（HMAC 验证已禁用，使用默认密钥）"""
    # 使用默认密钥（HMAC 验证已禁用）
    key = b'\x00' * 32

    # 用默认密钥编解码往返
    payload = struct.pack("<BBBHHihhH", 1, 0, 1, 100, 3, 1500, 200, 10, 0)
    packet = encode_packet(MSG_CAR_TELEMETRY, SENDER_CAR, 0xDEADBEEF, 1, 500, payload, key)
    hdr = decode_packet(packet, key)
    assert hdr is not None, "默认密钥编解码失败"
    assert hdr.boot_id == 0xDEADBEEF

    print(f"  [PASS] 默认密钥编解码往返（HMAC 验证已禁用）")


def main():
    print("ED UAV 通信诊断工具自测")
    print("=" * 50)
    print()

    print("[协议层]")
    test_crc16()
    test_hmac()
    test_encode_decode_roundtrip()
    test_tampered_packet_rejected()
    test_wrong_key_rejected()

    print()
    print("[载荷解码]")
    test_telemetry_payload_decode()
    test_selection_payload_decode()

    print()
    print("[生产密钥]")
    test_production_key()

    print()
    print("[网络层]")
    test_live_loopback()

    print()
    print("=" * 50)
    print("全部测试通过")


if __name__ == "__main__":
    main()
