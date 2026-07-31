"""ROS 2 adapter exposing H7 GPIO board control over a command topic.

订阅 /h7_gpio/command (ed_uav_interfaces/msg/H7GpioCommand):
  - command=CMD_SET_OUTPUT: value 1=HIGH (吸合), 0=LOW (释放), period_ms 忽略
  - command=CMD_CONFIGURE:  value 1=OUTPUT, 0=INPUT
  - command=CMD_PULSE:      value=count, period_ms=周期(ms)

激光头和电磁铁接在 C 板同一个引脚 (01 脚), C 板固件内部已封装 PWM 驱动,
上位机沿用旧激光头 0xAA 协议数据包即可, 无需关心具体引脚。
"""

from __future__ import annotations

from pathlib import Path

import rclpy
from ed_uav_interfaces.msg import H7GpioCommand
from rclpy.node import Node

from .h7_gpio_protocol import (
    CMD_CONFIGURE,
    CMD_PULSE,
    CMD_SET_OUTPUT,
    cmd_configure,
    cmd_pulse,
    cmd_set_output,
)
from .h7_gpio_serial import H7GpioTransport
from .serial_port import SerialOpenError

__all__ = ("H7GpioNode", "main")


class H7GpioNode(Node):
    """Own the H7 GPIO serial endpoint and translate topic commands to 0xAA frames."""

    def __init__(self) -> None:
        super().__init__("ed_uav_h7_gpio_bridge")
        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        self.declare_parameter("serial_lock_dir", "/tmp")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("response_timeout_s", 0.5)

        device = str(self.get_parameter("serial_port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        lock_dir = Path(str(self.get_parameter("serial_lock_dir").value))
        self._response_timeout_s = float(
            self.get_parameter("response_timeout_s").value
        )

        self._transport = H7GpioTransport(device, baudrate, lock_dir)
        try:
            self._transport.open()
        except SerialOpenError as error:
            self.get_logger().error("H7 GPIO serial open failed: %s", error)
            raise

        self._subscription = self.create_subscription(
            H7GpioCommand,
            "/h7_gpio/command",
            self._on_command,
            10,
        )
        self.get_logger().info(
            f"H7 GPIO bridge ready on {device} @ {baudrate} (topic /h7_gpio/command)"
        )

    def destroy_node(self) -> bool:
        self._transport.close()
        return super().destroy_node()

    def _on_command(self, message: H7GpioCommand) -> None:
        try:
            frame = self._build_frame(message)
        except ValueError as error:
            self.get_logger().warn(f"invalid H7 GPIO command: {error}")
            return
        try:
            self._transport.send(frame)
        except SerialOpenError as error:
            self.get_logger().error(f"H7 GPIO TX failed: {error}")
            return
        response = self._transport.read_response(self._response_timeout_s)
        if response is None:
            self.get_logger().debug(
                f"H7 GPIO TX (pin={message.pin}, cmd=0x{message.command:02X}) "
                f"no response within {self._response_timeout_s:.1f}s"
            )
        else:
            self.get_logger().debug(
                f"H7 GPIO ACK pin={response.pin} cmd=0x{response.command:02X} "
                f"status=0x{response.status:02X}"
            )

    @staticmethod
    def _build_frame(message: H7GpioCommand) -> bytes:
        pin = int(message.pin)
        command = int(message.command)
        if command == CMD_SET_OUTPUT:
            return cmd_set_output(pin, message.value != 0)
        if command == CMD_CONFIGURE:
            return cmd_configure(pin, message.value != 0)
        if command == CMD_PULSE:
            return cmd_pulse(pin, int(message.value), int(message.period_ms))
        raise ValueError(f"unknown command 0x{command:02X}")


def main() -> None:
    """Run the H7 GPIO bridge node."""
    rclpy.init()
    node = H7GpioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
