"""Launch the H7 GPIO bridge node (STM32H7 大疆电机开发板C, 0xAA 串口协议).

订阅 /h7_gpio/command (ed_uav_interfaces/msg/H7GpioCommand) 控制 C 板 GPIO
引脚输出 (电磁铁/激光/LED)。激光头与电磁铁同脚 (01 脚), C 板固件已封装
PWM 驱动, 沿用旧激光头协议数据包即可, 无需关心引脚。

用法:
    ros2 launch ed_uav_bringup h7_gpio_bridge.launch.py serial_port:=/dev/ttyUSB1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB1"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            Node(
                package="ed_uav_fcu_bridge",
                executable="ed_uav_h7_gpio_bridge",
                name="ed_uav_h7_gpio_bridge",
                output="screen",
                parameters=[
                    {
                        "serial_port": serial_port,
                        "baudrate": baudrate,
                    }
                ],
            ),
        ]
    )
