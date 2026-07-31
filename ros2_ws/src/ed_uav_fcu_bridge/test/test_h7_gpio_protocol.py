"""Tests for the H7 GPIO 0xAA protocol encoder/decoder."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Final

import pytest

PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_fcu_bridge.h7_gpio_protocol import (
    CMD_CONFIGURE,
    CMD_PULSE,
    CMD_SET_OUTPUT,
    H7GpioFrameError,
    build_command,
    cmd_configure,
    cmd_pulse,
    cmd_set_output,
    format_frame,
    parse_response,
    verify_checksum,
)


class TestBuildCommand:
    def test_set_output_high_frame_layout(self) -> None:
        frame = cmd_set_output(2, True)
        # AA 02 01 01 01 [XOR]
        assert frame[0] == 0xAA
        assert frame[1] == 0x02  # pin
        assert frame[2] == CMD_SET_OUTPUT
        assert frame[3] == 0x01  # len
        assert frame[4] == 0x01  # value HIGH
        assert verify_checksum(frame)

    def test_set_output_low_frame_layout(self) -> None:
        frame = cmd_set_output(2, False)
        assert frame[4] == 0x00  # value LOW
        assert verify_checksum(frame)

    def test_set_output_matches_legacy_reference(self) -> None:
        # 旧 drone/h7_gpio_protocol.py 黄金帧: pin=2, high=True -> AA 02 01 01 01 03
        assert cmd_set_output(2, True) == bytes.fromhex("AA0201010103")
        # pin=2, high=False -> AA 02 01 01 00 02
        assert cmd_set_output(2, False) == bytes.fromhex("AA0201010002")

    def test_configure_output(self) -> None:
        frame = cmd_configure(3, True)
        assert frame[2] == CMD_CONFIGURE
        assert frame[3] == 0x01
        assert frame[4] == 0x01

    def test_pulse_payload_layout(self) -> None:
        frame = cmd_pulse(1, count=3, period_ms=1500)
        assert frame[2] == CMD_PULSE
        assert frame[3] == 0x03  # len = count(1) + period(2)
        assert frame[4] == 0x03  # count
        assert frame[5] == 0xDC  # 1500 & 0xFF
        assert frame[6] == 0x05  # 1500 >> 8
        assert verify_checksum(frame)

    def test_pulse_clamps_period(self) -> None:
        frame = cmd_pulse(1, count=1, period_ms=99999)
        assert (frame[5] | (frame[6] << 8)) == 65535

    def test_pin_out_of_range(self) -> None:
        with pytest.raises(ValueError):
            build_command(16, CMD_SET_OUTPUT, b"\x01")
        with pytest.raises(ValueError):
            build_command(-1, CMD_SET_OUTPUT, b"\x01")

    def test_payload_too_long(self) -> None:
        with pytest.raises(ValueError):
            build_command(1, CMD_SET_OUTPUT, b"\x00" * 9)

    def test_xor_checksum_math(self) -> None:
        frame = cmd_set_output(2, True)
        xor = frame[1]
        for value in frame[2:-1]:
            xor ^= value
        assert xor == frame[-1]


class TestParseResponse:
    def test_valid_response(self) -> None:
        # BB 02 01 00 03: pin=2, cmd=SET_OUTPUT, status=0
        frame = bytes.fromhex("BB02010003")
        response = parse_response(frame)
        assert response is not None
        assert response.pin == 2
        assert response.command == CMD_SET_OUTPUT
        assert response.status == 0
        assert response.raw == frame

    def test_wrong_header(self) -> None:
        assert parse_response(bytes.fromhex("AA02010002")) is None

    def test_wrong_length(self) -> None:
        assert parse_response(bytes.fromhex("BB020100")) is None
        assert parse_response(bytes.fromhex("BB0201000300")) is None

    def test_bad_checksum(self) -> None:
        assert parse_response(bytes.fromhex("BB020100FF")) is None

    def test_none_input(self) -> None:
        assert parse_response(None) is None  # type: ignore[arg-type]


class TestHelpers:
    def test_verify_checksum_false_for_bad_frame(self) -> None:
        assert not verify_checksum(bytes.fromhex("AA0201010100"))

    def test_format_frame(self) -> None:
        assert format_frame(bytes.fromhex("AA0201010103")) == "AA 02 01 01 01 03"
