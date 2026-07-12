"""
test_h7_gpio_protocol.py — STM32H7 GPIO 串口协议测试
验证 Command frame (0xAA) 和 Response frame (0xBB) 的构建、解析与校验

协议格式:
  Command:  0xAA [PIN] [CMD] [LEN] [PAYLOAD...] [XOR]
  Response: 0xBB [PIN] [CMD] [STATUS] [XOR]
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    from h7_gpio_protocol import (
        build_h7_command,
        parse_h7_response,
        cmd_set_output,
        cmd_configure,
        cmd_pulse,
        verify_h7_frame,
        print_frame,
    )
    PROTOCOL_AVAILABLE = True
except ImportError:
    PROTOCOL_AVAILABLE = False


def _xor_checksum(data: bytes) -> int:
    """辅助: 计算 XOR 校验和（供测试内手动验证用）"""
    x = 0
    for b in data:
        x ^= b
    return x


def _make_response(pin: int, cmd: int, status: int) -> bytes:
    """辅助: 手动构造合法 H7 响应帧"""
    xor = _xor_checksum(bytes([pin, cmd, status]))
    return bytes([0xBB, pin, cmd, status, xor])


# ════════════════════════════════════════════════════════════
#  TestH7FrameBuilding  — 命令帧构建
# ════════════════════════════════════════════════════════════

@unittest.skipUnless(PROTOCOL_AVAILABLE, "h7_gpio_protocol 模块尚未实现")
class TestH7FrameBuilding(unittest.TestCase):
    """测试 H7 命令帧构建"""

    def test_set_output_high(self):
        """验证 SET_OUTPUT(3, HIGH) 帧头"""
        frame = cmd_set_output(3, True)
        # AA 03 01 01 01 [XOR]
        self.assertEqual(frame[:5], bytes([0xAA, 0x03, 0x01, 0x01, 0x01]))
        self.assertEqual(len(frame), 6, "帧长应为6字节 (header+pin+cmd+len+payload+xor)")

    def test_set_output_low(self):
        """验证 SET_OUTPUT(5, LOW) 帧头"""
        frame = cmd_set_output(5, False)
        # AA 05 01 01 00 [XOR]
        self.assertEqual(frame[:5], bytes([0xAA, 0x05, 0x01, 0x01, 0x00]))
        self.assertEqual(len(frame), 6)

    def test_configure_output(self):
        """验证 CONFIGURE(7, OUTPUT) 帧头"""
        frame = cmd_configure(7, True)
        # AA 07 02 01 01 [XOR]
        self.assertEqual(frame[:5], bytes([0xAA, 0x07, 0x02, 0x01, 0x01]))
        self.assertEqual(len(frame), 6)

    def test_configure_input(self):
        """验证 CONFIGURE(2, INPUT) 帧头"""
        frame = cmd_configure(2, False)
        # AA 02 02 01 00 [XOR]
        self.assertEqual(frame[:5], bytes([0xAA, 0x02, 0x02, 0x01, 0x00]))
        self.assertEqual(len(frame), 6)

    def test_pulse_basic(self):
        """验证 PULSE(1, count=5, period=200ms) 帧"""
        frame = cmd_pulse(1, 5, 200)
        # AA 01 03 03 05 C8 00 [XOR]
        # LEN=3 (count + 2字节period), period 200=0x00C8 LE → C8 00
        self.assertEqual(frame[:7], bytes([0xAA, 0x01, 0x03, 0x03, 0x05, 0xC8, 0x00]))
        self.assertEqual(len(frame), 8, "帧长应为8字节")

    def test_pulse_large_period(self):
        """验证 PULSE(0, count=3, period=65535ms) 的周期字节"""
        frame = cmd_pulse(0, 3, 65535)
        # period 65535 = 0xFFFF LE → FF FF
        self.assertEqual(frame[0], 0xAA)
        self.assertEqual(frame[1], 0x00, "PIN=0")
        self.assertEqual(frame[2], 0x03, "CMD=PULSE")
        self.assertEqual(frame[5], 0xFF, "PERIOD_LO")
        self.assertEqual(frame[6], 0xFF, "PERIOD_HI")

    def test_build_h7_command_raw(self):
        """验证 build_h7_command 裸接口 (PIN=4, CMD=0x01, payload=b'\\x01')"""
        frame = build_h7_command(4, 0x01, b'\x01')
        # AA 04 01 01 01 [XOR]
        self.assertEqual(frame, bytes([0xAA, 0x04, 0x01, 0x01, 0x01,
                                       _xor_checksum(bytes([0x04, 0x01, 0x01, 0x01]))]))


# ════════════════════════════════════════════════════════════
#  TestH7ConvenienceCommands  — 便捷命令封装
# ════════════════════════════════════════════════════════════

@unittest.skipUnless(PROTOCOL_AVAILABLE, "h7_gpio_protocol 模块尚未实现")
class TestH7ConvenienceCommands(unittest.TestCase):
    """测试便捷命令的参数映射"""

    def test_set_output_true_uses_cmd_01(self):
        """cmd_set_output(pin, True) → CMD=0x01, payload=[0x01]"""
        frame = cmd_set_output(10, True)
        self.assertEqual(frame[2], 0x01, "CMD 字段应为 0x01 (SET_OUTPUT)")
        self.assertEqual(frame[4], 0x01, "payload[0] 应为 0x01 (HIGH)")

    def test_set_output_false_uses_cmd_01(self):
        """cmd_set_output(pin, False) → CMD=0x01, payload=[0x00]"""
        frame = cmd_set_output(10, False)
        self.assertEqual(frame[2], 0x01, "CMD 字段应为 0x01 (SET_OUTPUT)")
        self.assertEqual(frame[4], 0x00, "payload[0] 应为 0x00 (LOW)")

    def test_configure_as_output_uses_cmd_02(self):
        """cmd_configure(pin, True) → CMD=0x02, payload=[0x01]"""
        frame = cmd_configure(10, True)
        self.assertEqual(frame[2], 0x02, "CMD 字段应为 0x02 (CONFIGURE)")
        self.assertEqual(frame[4], 0x01, "payload[0] 应为 0x01 (OUTPUT)")

    def test_configure_as_input_uses_cmd_02(self):
        """cmd_configure(pin, False) → CMD=0x02, payload=[0x00]"""
        frame = cmd_configure(10, False)
        self.assertEqual(frame[2], 0x02, "CMD 字段应为 0x02 (CONFIGURE)")
        self.assertEqual(frame[4], 0x00, "payload[0] 应为 0x00 (INPUT)")


# ════════════════════════════════════════════════════════════
#  TestH7ResponseParsing  — 响应帧解析
# ════════════════════════════════════════════════════════════

@unittest.skipUnless(PROTOCOL_AVAILABLE, "h7_gpio_protocol 模块尚未实现")
class TestH7ResponseParsing(unittest.TestCase):
    """测试 H7 响应帧解析"""

    def test_parse_response_ok(self):
        """验证正常 OK 响应 (status=0x00)"""
        buf = _make_response(pin=3, cmd=1, status=0x00)
        result = parse_h7_response(buf)
        self.assertIsNotNone(result)
        self.assertEqual(result['pin'], 3)
        self.assertEqual(result['cmd'], 1)
        self.assertEqual(result['status'], 0x00)

    def test_parse_response_error(self):
        """验证 ERROR 响应 (status=0x01)"""
        buf = _make_response(pin=7, cmd=2, status=0x01)
        result = parse_h7_response(buf)
        self.assertIsNotNone(result)
        self.assertEqual(result['status'], 0x01, "ERROR 状态码")

    def test_parse_response_timeout(self):
        """验证 TIMEOUT 响应 (status=0x02)"""
        buf = _make_response(pin=0, cmd=3, status=0x02)
        result = parse_h7_response(buf)
        self.assertIsNotNone(result)
        self.assertEqual(result['status'], 0x02, "TIMEOUT 状态码")

    def test_parse_response_short_buffer(self):
        """验证过短缓冲区返回 None"""
        self.assertIsNone(parse_h7_response(b'\xBB\x03\x01'))

    def test_parse_response_wrong_header(self):
        """验证错误帧头返回 None"""
        buf = _make_response(pin=3, cmd=1, status=0x00)
        bad = bytearray(buf)
        bad[0] = 0xCC  # 非法帧头
        self.assertIsNone(parse_h7_response(bytes(bad)))

    def test_parse_response_bad_checksum(self):
        """验证篡改校验和返回 None"""
        buf = bytearray(_make_response(pin=3, cmd=1, status=0x00))
        buf[-1] ^= 0xFF  # 翻转 XOR 字节
        self.assertIsNone(parse_h7_response(bytes(buf)))

    def test_parse_response_none_for_none(self):
        """验证 parse_h7_response(None) 不崩溃"""
        result = parse_h7_response(None)
        self.assertIsNone(result)


# ════════════════════════════════════════════════════════════
#  TestH7Checksum  — 校验和验证
# ════════════════════════════════════════════════════════════

@unittest.skipUnless(PROTOCOL_AVAILABLE, "h7_gpio_protocol 模块尚未实现")
class TestH7Checksum(unittest.TestCase):
    """测试 XOR 校验和逻辑"""

    def test_verify_valid_frame(self):
        """验证合法帧通过校验"""
        frame = cmd_set_output(3, True)
        self.assertTrue(verify_h7_frame(frame), "合法帧应通过校验")

    def test_verify_tampered_frame(self):
        """验证篡改后帧校验失败"""
        frame = bytearray(cmd_set_output(3, True))
        frame[3] ^= 0x01  # 翻转 LEN 字节
        self.assertFalse(verify_h7_frame(bytes(frame)), "篡改帧应校验失败")

    def test_verify_too_short(self):
        """验证过短帧返回 False"""
        self.assertFalse(verify_h7_frame(b'\xAA\x03'))

    def test_checksum_xor_property(self):
        """验证 XOR 恒等式: XOR(pin,cmd,len,payload) ^ stored_xor == 0"""
        frame = cmd_set_output(9, True)
        # 按协议: XOR over [PIN, CMD, LEN, PAYLOAD...] = stored_xor
        # 因此: XOR(pin) ^ XOR(cmd) ^ ... ^ stored_xor == 0
        data_bytes = frame[1:-1]  # 去掉 header(0xAA) 和 XOR 字节
        stored_xor = frame[-1]
        computed = 0
        for b in data_bytes:
            computed ^= b
        self.assertEqual(computed, stored_xor,
                         "手动计算的 XOR 应与帧内存储值一致")
        # 恒等式: 全部 XOR 应为 0
        self.assertEqual(computed ^ stored_xor, 0,
                         "XOR 恒等式失败")

    def test_pin_range_all_valid(self):
        """验证 PIN 0-15 均可构建合法帧"""
        for pin in range(16):
            frame = cmd_set_output(pin, True)
            self.assertEqual(frame[1], pin, f"PIN={pin} 字节不匹配")
            self.assertTrue(verify_h7_frame(frame),
                            f"PIN={pin} 校验失败")


if __name__ == '__main__':
    unittest.main(verbosity=2)
