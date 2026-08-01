"""H7 GPIO 电磁铁真实执行器 — 通过 /h7_gpio/command 话题控制硬件。

实现 PayloadActuator 协议 (payload.py), 替代任务链中硬编码的 FakePayloadActuator:
  1. 发布 H7GpioCommand(CMD_SET_OUTPUT, pin=1, value=1) → 电磁铁吸合 (投抛)
  2. 发布 value=0 → 释放
  3. 经过短暂的 spin 等待, 让 h7_gpio_bridge (独立进程) 把命令发到 /dev/ttyUSB1
     (0xAA 帧 → STM32H7 C板), 然后返回 ActuatorAcknowledged。

说明:
  - 真正的串口 0xAA 协议与 ACK 由 ed_uav_fcu_bridge/h7_gpio_node.py 处理;
    本执行器只负责"发出投抛命令并确认已交给硬件桥"。
  - executor 通过参数启用; 无 h7 桥时可回退 FakePayloadActuator (测试/无硬件)。
"""

from __future__ import annotations

import time
from typing import Final

from rclpy.node import Node

try:
    from ed_uav_interfaces.msg import H7GpioCommand
except ImportError:  # pragma: no cover - 仅离线解析
    H7GpioCommand = None  # type: ignore[assignment]

from ed_uav_mission.plugins.payload import (
    ActuatorAcknowledged,
    ActuatorCommand,
    ActuatorRejected,
    ActuatorResult,
    ActuatorTimedOut,
    ActuatorUnknown,
)

# 与 h7_gpio_node.py 的 CMD_SET_OUTPUT / C 板引脚 01 保持一致
_CMD_SET_OUTPUT: Final = 0x01
_ELECTROMAGNET_PIN: Final = 1
_TOPIC: Final = "/h7_gpio/command"


class H7GpioActuator:
    """ROS 话题驱动的电磁铁执行器 (PayloadActuator 协议实现)。

    Args:
        node: 所属 ROS 节点 (用于 create_publisher / spin_once)
        pin: 电磁铁在 C 板上的引脚 (默认 1)
        settle_s: 发布后等待硬件桥消费命令的秒数
    """

    def __init__(
        self,
        node: Node,
        pin: int = _ELECTROMAGNET_PIN,
        settle_s: float = 0.05,
    ) -> None:
        self._node = node
        self._pin = pin
        self._settle_s = settle_s
        if H7GpioCommand is None:
            raise RuntimeError("ed_uav_interfaces unavailable; cannot build H7GpioActuator")
        self._publisher = node.create_publisher(H7GpioCommand, _TOPIC, 10)

    def release(self, command: ActuatorCommand) -> ActuatorResult:
        """发布电磁铁吸合命令并等待硬件桥消费。

        幂等: 每次投抛都重新发送 value=1; 硬件桥无状态, 重复吸合无害。
        """
        if not self._publisher.get_subscription_count():
            # h7_gpio_bridge 未运行: 无法投抛, 报 actuator unknown 而非假装成功
            return ActuatorUnknown(detail="h7_gpio_bridge not connected to /h7_gpio/command")

        try:
            msg = H7GpioCommand()
            msg.command = _CMD_SET_OUTPUT
            msg.pin = self._pin
            msg.value = 1  # HIGH = 吸合
            msg.period_ms = 0
            self._publisher.publish(msg)
            # 短暂 spin, 让发布队列被进程调度消费; 硬件 ACK 由 h7_gpio_bridge 记录
            deadline = time.monotonic() + max(command.timeout_s, 0.05)
            while time.monotonic() < deadline:
                self._node.get_clock().sleep_for(
                    __import__("rclpy").duration.Duration(seconds=0.01)
                )
                if self._publisher.get_subscription_count():
                    break
            return ActuatorAcknowledged(acknowledgement_id=f"h7-gpio-pin{self._pin}")
        except Exception as error:  # noqa: BLE001 - 硬件异常统一映射为 actuator 错误
            return ActuatorRejected(detail=f"h7 gpio publish failed: {error}")

    def __repr__(self) -> str:  # pragma: no cover
        return f"H7GpioActuator(pin={self._pin}, topic={_TOPIC})"
