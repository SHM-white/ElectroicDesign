"""Full competition chain — one-command bringup of the complete D-task loop.

组合链路 (对应竞赛完整闭环):

  地面站确认任务 ──UDP──▶ vehicle_bridge ──/d_task/pre_arm/select_mission──▶ mission_executor
  小车遥测      ──UDP──▶ vehicle_bridge ──/d_task/vehicle/telemetry──▶ mission_executor (等待 start_event)
  任务状态      ◀──/d_task/mission_status── mission_executor ◀──UDP── 地面站
  mission_executor ──/fcu/flight_command──▶ fcu_bridge ──串口──▶ 飞控
  mission_executor ──/localization/status──▶ localization (source_supervisor)
  mission_executor ──/d_task/target_observation──▶ perception (target_observation)
  h7_gpio_bridge ◀──/h7_gpio/command── (电磁铁/激光控制)

串口自动检测:
    默认 auto_detect_serial:=true, 启动时自动扫描 /dev/ttyUSB* 并通过
    探测指令识别 H7 GPIO 板 (0xBB 响应) 和凌霄飞控 (V7 遥测帧),
    无需硬编码设备路径。如需手动指定, 设 auto_detect_serial:=false
    并通过 fcu_serial_port / h7_serial_port 传入。

注意: mission 侧源码使用 /vehicle/telemetry、/mission/status、/mission/select_d_task,
而 vehicle_bridge 使用 /d_task/ 前缀, 本 launch 通过 remap 对齐两者。

用法:
    ros2 launch ed_uav_bringup full_competition.launch.py
    ros2 launch ed_uav_bringup full_competition.launch.py simulation_only:=true
    ros2 launch ed_uav_bringup full_competition.launch.py auto_detect_serial:=false
    ros2 launch ed_uav_bringup full_competition.launch.py \\
        mission_config_path:=.../simulation_competition.yaml
"""

from __future__ import annotations

import logging
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ── UDP 三端地址 (与 PROTOCOL.md / install_boot.sh 一致) ──────────────────
NUC_IP = "192.168.20.1"
CAR_IP = "192.168.20.2"
HMI_IP = "192.168.20.3"
CAR_SENDER_ID = "1128419121"   # 0x43415231 "CAR1"
HMI_SENDER_ID = "1213024561"   # 0x484D4931 "HMI1"
BRIDGE_SENDER_ID = "1381122353"  # 0x524F5331 "ROS1"


def _resolve_serial_ports(context, *args, **kwargs):
    """Run serial auto-detection and return nodes with resolved device paths."""
    auto_detect = (
        context.launch_configurations.get("auto_detect_serial", "true").lower() == "true"
    )
    fcu_port = context.launch_configurations.get("fcu_serial_port", "/dev/ttyUSB0")
    h7_port = context.launch_configurations.get("h7_serial_port", "/dev/ttyUSB1")

    if auto_detect:
        try:
            from ed_uav_fcu_bridge.serial_detect import detect_or_fallback

            detected = detect_or_fallback(
                default_h7=h7_port,
                default_fcu=fcu_port,
                probe_timeout=1.0,
            )
            fcu_port = detected["fcu"]
            h7_port = detected["h7_gpio"]
        except Exception as exc:
            logging.getLogger("launch").warning(
                "串口自动检测失败 (%s), 使用默认路径", exc
            )

    logging.getLogger("launch").info("飞控串口: %s, H7 GPIO 串口: %s", fcu_port, h7_port)
    return _build_nodes(context, fcu_port, h7_port)


def _build_nodes(context, fcu_port: str, h7_port: str):
    """Build the full set of launch nodes with resolved serial paths."""
    mission_share = Path(get_package_share_directory("ed_uav_mission"))
    localization_share = Path(get_package_share_directory("ed_uav_localization"))
    description_share = Path(get_package_share_directory("ed_uav_description"))

    return [
        # ── 1. 车辆/地面站 UDP 桥 ─────────────────────────────────────
        Node(
            package="ed_uav_vehicle_bridge",
            executable="vehicle_bridge",
            name="vehicle_bridge",
            output="screen",
            parameters=[
                {
                    "bind_host": NUC_IP,
                    "bind_port": 42000,
                    "car_peer_host": CAR_IP,
                    "car_peer_port": 42001,
                    "hmi_peer_host": HMI_IP,
                    "hmi_peer_port": 42002,
                    "car_sender_id": CAR_SENDER_ID,
                    "hmi_sender_id": HMI_SENDER_ID,
                    "bridge_sender_id": BRIDGE_SENDER_ID,
                    "hmac_key_file": LaunchConfiguration("hmac_key_file").perform(context),
                    "mission_timeout_seconds": 90.0,
                    "telemetry_stale_seconds": 0.75,
                }
            ],
        ),

        # ── 2. 飞控串口桥 (FlightCommand action server) ──────────────
        Node(
            package="ed_uav_fcu_bridge",
            executable="ed_uav_fcu_bridge",
            name="ed_uav_fcu_bridge",
            output="screen",
            arguments=["--ros-args", "--enclave", "/ed_uav_fcu_bridge"],
            parameters=[
                {
                    "serial_port": fcu_port,
                    "baudrate": 500000,
                    "enable_flight_commands": LaunchConfiguration("enable_flight_commands").perform(context),
                    "enable_experimental_0x32_0x33": False,
                }
            ],
        ),

        # ── 3. 任务执行器 (D-task 行为树) ─────────────────────────────
        Node(
            package="ed_uav_mission",
            executable="mission_executor",
            name="mission_executor",
            output="screen",
            arguments=["--ros-args", "--enclave", "/ed_uav_mission_executor"],
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
                    "profile_path": LaunchConfiguration("profile_path").perform(context),
                    "mission_config_path": LaunchConfiguration("mission_config_path").perform(context),
                    "calibration_file": LaunchConfiguration("calibration_file").perform(context),
                    "simulation_only": LaunchConfiguration("simulation_only").perform(context),
                    "payload_config_path": LaunchConfiguration("payload_config_path").perform(context),
                    "programmable_capability_report": "",
                    "fcu_device_identity": "",
                }
            ],
            remappings=[
                ("/vehicle/telemetry", "/d_task/vehicle/telemetry"),
                ("/mission/status", "/d_task/mission_status"),
                ("/mission/select_d_task", "/d_task/pre_arm/select_mission"),
            ],
        ),

        # ── 4. H7 GPIO 桥 (电磁铁/激光, 0xAA 协议) ────────────────────
        Node(
            package="ed_uav_fcu_bridge",
            executable="ed_uav_h7_gpio_bridge",
            name="ed_uav_h7_gpio_bridge",
            output="screen",
            parameters=[
                {
                    "serial_port": h7_port,
                    "baudrate": 115200,
                }
            ],
        ),

        # ── 5. 定位融合 (source_supervisor → /localization/status) ────
        Node(
            package="ed_uav_localization",
            executable="source_supervisor",
            name="source_supervisor",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time").perform(context)}
            ],
        ),
        Node(
            package="ed_uav_localization",
            executable="field_anchor",
            name="field_anchor",
            output="screen",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
                    "profile_path": LaunchConfiguration("profile_path").perform(context),
                }
            ],
        ),

        # ── 6. 感知目标观测 (→ /d_task/target_observation) ────────────
        Node(
            package="ed_uav_perception",
            executable="target_observation_node",
            name="target_observation_node",
            output="screen",
            parameters=[
                {"target_revision": "d2026-circle-cross-v1"},
            ],
            remappings=[
                ("/camera/narrow/image_raw", LaunchConfiguration("image_topic").perform(context)),
                ("/camera/narrow/camera_info", LaunchConfiguration("camera_info_topic").perform(context)),
                ("/d_task/vehicle/telemetry", "/d_task/vehicle/telemetry"),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    mission_share = Path(get_package_share_directory("ed_uav_mission"))
    localization_share = Path(get_package_share_directory("ed_uav_localization"))
    description_share = Path(get_package_share_directory("ed_uav_description"))
    repo_root = Path(__file__).resolve().parents[4]

    return LaunchDescription(
        [
            # ── 公共参数 ──────────────────────────────────────────────────
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "mission_config_path",
                default_value=str(
                    mission_share / "config" / "missions" / "simulation_competition.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "profile_path",
                default_value=str(
                    localization_share / "config" / "fields" / "simulation_arena.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=str(
                    description_share / "config" / "example_uncalibrated.yaml"
                ),
            ),
            DeclareLaunchArgument("simulation_only", default_value="false"),
            DeclareLaunchArgument(
                "payload_config_path",
                default_value=str(mission_share / "config" / "payload_adapter.yaml"),
            ),
            DeclareLaunchArgument(
                "hmac_key_file",
                default_value=str(repo_root / "config" / "hmac.key.hex"),
            ),
            DeclareLaunchArgument(
                "enable_flight_commands",
                default_value="false",
                description=(
                    "暴露 /fcu/flight_command action; true 需要 SROS2 keystore "
                    "(ROS_SECURITY_ENABLE=true + Enforce + keystore), 无 keystore 时保持 false"
                ),
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/narrow/image_raw",
                description="perception 目标观测输入图像 topic",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/narrow/camera_info",
                description="perception 目标观测相机内参 topic",
            ),

            # ── 串口自动检测参数 ────────────────────────────────────────
            DeclareLaunchArgument(
                "auto_detect_serial",
                default_value="true",
                description="启动时自动扫描 /dev/ttyUSB* 识别飞控和 H7 GPIO 板",
            ),
            DeclareLaunchArgument(
                "fcu_serial_port",
                default_value="/dev/ttyUSB0",
                description="飞控串口路径 (auto_detect_serial=false 时使用)",
            ),
            DeclareLaunchArgument(
                "h7_serial_port",
                default_value="/dev/ttyUSB1",
                description="H7 GPIO 板串口路径 (auto_detect_serial=false 时使用)",
            ),

            # ── 使用 OpaqueFunction 在启动时执行串口检测 ────────────────
            OpaqueFunction(function=_resolve_serial_ports),
        ]
    )
