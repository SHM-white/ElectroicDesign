"""Full competition chain — one-command bringup of the complete D-task loop.

组合链路 (对应竞赛完整闭环):

  地面站确认任务 ──UDP──▶ vehicle_bridge ──/d_task/pre_arm/select_mission──▶ mission_executor
  小车遥测      ──UDP──▶ vehicle_bridge ──/d_task/vehicle/telemetry──▶ mission_executor (等待 start_event)
  任务状态      ◀──/d_task/mission_status── mission_executor ◀──UDP── 地面站
  mission_executor ──/fcu/flight_command──▶ fcu_bridge ──串口──▶ 飞控
  mission_executor ──/localization/status──▶ localization (source_supervisor)
  mission_executor ──/d_task/target_observation──▶ perception (target_observation)
  h7_gpio_bridge ◀──/h7_gpio/command── (电磁铁/激光控制)

注意: mission 侧源码使用 /vehicle/telemetry、/mission/status、/mission/select_d_task,
而 vehicle_bridge 使用 /d_task/ 前缀, 本 launch 通过 remap 对齐两者。

用法:
    ros2 launch ed_uav_bringup full_competition.launch.py
    ros2 launch ed_uav_bringup full_competition.launch.py simulation_only:=true
    ros2 launch ed_uav_bringup full_competition.launch.py \
        mission_config_path:=.../simulation_competition.yaml
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ── UDP 三端地址 (与 PROTOCOL.md / install_boot.sh 一致) ──────────────────
NUC_IP = "192.168.20.1"
CAR_IP = "192.168.20.2"
HMI_IP = "192.168.20.3"
CAR_SENDER_ID = "1128419121"   # 0x43415231 "CAR1"
HMI_SENDER_ID = "1212563761"   # 0x484D4931 "HMI1"
BRIDGE_SENDER_ID = "1381122353"  # 0x524F5331 "ROS1"


def generate_launch_description() -> LaunchDescription:
    mission_share = Path(get_package_share_directory("ed_uav_mission"))
    localization_share = Path(get_package_share_directory("ed_uav_localization"))
    perception_share = Path(get_package_share_directory("ed_uav_perception"))
    description_share = Path(get_package_share_directory("ed_uav_description"))
    # 仓库根 = ros2_ws/src/ed_uav_bringup/launch 上溯 4 级
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
                        "hmac_key_file": LaunchConfiguration("hmac_key_file"),
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
                        "serial_port": "/dev/ttyUSB0",
                        "baudrate": 500000,
                        # 需要 SROS2 (ROS_SECURITY_ENABLE=true + keystore) 才会暴露 action
                        "enable_flight_commands": LaunchConfiguration("enable_flight_commands"),
                        "enable_experimental_0x32_0x33": False,
                    }
                ],
            ),

            # ── 3. 任务执行器 (D-task 行为树) ─────────────────────────────
            # 注意 remap: vehicle_bridge 使用 /d_task/ 前缀, mission 源码用
            # /vehicle/telemetry、/mission/status、/mission/select_d_task。
            Node(
                package="ed_uav_mission",
                executable="mission_executor",
                name="mission_executor",
                output="screen",
                arguments=["--ros-args", "--enclave", "/ed_uav_mission_executor"],
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "profile_path": LaunchConfiguration("profile_path"),
                        "mission_config_path": LaunchConfiguration("mission_config_path"),
                        "calibration_file": LaunchConfiguration("calibration_file"),
                        "simulation_only": LaunchConfiguration("simulation_only"),
                        "payload_config_path": LaunchConfiguration("payload_config_path"),
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
                        "serial_port": "/dev/ttyUSB1",
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
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}
                ],
            ),
            Node(
                package="ed_uav_localization",
                executable="field_anchor",
                name="field_anchor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "profile_path": LaunchConfiguration("profile_path"),
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
                    ("/camera/narrow/image_raw", LaunchConfiguration("image_topic")),
                    ("/camera/narrow/camera_info", LaunchConfiguration("camera_info_topic")),
                    ("/d_task/vehicle/telemetry", "/d_task/vehicle/telemetry"),
                ],
            ),
        ]
    )
