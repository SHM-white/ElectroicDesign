"""Launch the H7 GPIO bridge node (STM32H7 大疆电机开发板C, 0xAA 串口协议).

订阅 /h7_gpio/command (ed_uav_interfaces/msg/H7GpioCommand) 控制 C 板 GPIO
引脚输出 (电磁铁/激光/LED)。激光头与电磁铁同脚 (01 脚), C 板固件已封装
PWM 驱动, 沿用旧激光头协议数据包即可, 无需关心引脚。

串口自动检测:
    默认 auto_detect_serial:=true, 启动时自动扫描 /dev/ttyUSB* 识别 H7 GPIO 板。
    如需手动指定, 设 auto_detect_serial:=false 并通过 serial_port 传入。

用法:
    ros2 launch ed_uav_bringup h7_gpio_bridge.launch.py
    ros2 launch ed_uav_bringup h7_gpio_bridge.launch.py serial_port:=/dev/ttyUSB1
    ros2 launch ed_uav_bringup h7_gpio_bridge.launch.py auto_detect_serial:=false
"""

from __future__ import annotations

import logging

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_h7_port(context, *args, **kwargs):
    """Auto-detect the H7 GPIO serial port or use the manually specified one."""
    auto_detect = (
        context.launch_configurations.get("auto_detect_serial", "true").lower() == "true"
    )
    h7_port = context.launch_configurations.get("serial_port", "/dev/ttyUSB1")

    if auto_detect:
        try:
            from ed_uav_fcu_bridge.serial_detect import detect_or_fallback

            detected = detect_or_fallback(default_h7=h7_port, probe_timeout=1.0)
            h7_port = detected["h7_gpio"]
        except Exception as exc:
            logging.getLogger("launch").warning(
                "串口自动检测失败 (%s), 使用默认路径: %s", exc, h7_port
            )

    logging.getLogger("launch").info("H7 GPIO 串口: %s", h7_port)
    baudrate = context.launch_configurations.get("baudrate", "115200")
    return [
        Node(
            package="ed_uav_fcu_bridge",
            executable="ed_uav_h7_gpio_bridge",
            name="ed_uav_h7_gpio_bridge",
            output="screen",
            parameters=[
                {
                    "serial_port": h7_port,
                    "baudrate": int(baudrate),
                }
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB1"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument(
                "auto_detect_serial",
                default_value="true",
                description="启动时自动扫描 /dev/ttyUSB* 识别 H7 GPIO 板",
            ),
            OpaqueFunction(function=_resolve_h7_port),
        ]
    )
