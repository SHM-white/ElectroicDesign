"""Full competition chain — one-command bringup of the complete D-task loop.

组合链路 (对应竞赛完整闭环, 真实硬件):

  地面站确认任务 ──UDP──▶ vehicle_bridge ──/d_task/pre_arm/select_mission──▶ mission_executor
  小车遥测      ──UDP──▶ vehicle_bridge ──/d_task/vehicle/telemetry──▶ mission_executor
  任务状态      ◀──/d_task/mission_status── mission_executor ◀──UDP── 地面站
  mission_executor ──/fcu/flight_command──▶ fcu_bridge ──串口──▶ 飞控
  mission_executor ──/localization/status──▶ localization (source_supervisor)
  mission_executor ──/d_task/target_observation──▶ perception (双相机 AprilTag PnP)
  h7_gpio_bridge ◀──/h7_gpio/command── (电磁铁/激光控制)
  MID360 雷达 ──▶ FAST-LIO ──▶ lio_adapter ──▶ source_supervisor ──▶ /localization/odom
  双相机 (narrow+wide) ──▶ target_observation_node ──▶ /d_task/target_observation
  mission_display ◀── 双相机 + 任务 HUD + FCU 遥测 + 飞行指令反馈

串口自动检测:
    默认 auto_detect_serial:=true, 启动时自动扫描 /dev/ttyUSB* 并通过
    探测指令识别 H7 GPIO 板 (0xBB 响应) 和凌霄飞控 (V7 遥测帧),
    无需硬编码设备路径。如需手动指定, 设 auto_detect_serial:=false
    并通过 fcu_serial_port / h7_serial_port 传入。

实飞门控 (语义完全按 drone/ 工具):
    启动开关: AUX6 (第10通道) >1700us (drone/monitor_aux6.py, AUX1~AUX5 不可用)
    程控门控: AUX1 (第5通道) 1400~1600us (fcu_bridge 模式2 位置控制)
    硬锁    : AUX1 (第5通道) >=1800us (fcu_bridge 紧急停止)

用法:
    ros2 launch ed_uav_bringup full_competition.launch.py
    ros2 launch ed_uav_bringup full_competition.launch.py simulation_only:=true
    ros2 launch ed_uav_bringup full_competition.launch.py dry_run:=true
    ros2 launch ed_uav_bringup full_competition.launch.py auto_detect_serial:=false
    ros2 launch ed_uav_bringup full_competition.launch.py \\
        mission_config_path:=.../d_arena_stability_test.yaml
"""

from __future__ import annotations

import json
import logging
import subprocess
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── UDP 三端地址 (与 PROTOCOL.md / install_boot.sh 一致) ──────────────────
NUC_IP = "192.168.20.1"
CAR_IP = "192.168.20.2"
HMI_IP = "192.168.20.3"
CAR_SENDER_ID = "1128419121"   # 0x43415231 "CAR1"
HMI_SENDER_ID = "1213024561"   # 0x484D4931 "HMI1"
BRIDGE_SENDER_ID = "1381122353"  # 0x524F5331 "ROS1"

# Target revision for AprilTag detection
_TARGET_REVISION = "d2026-apriltag-v1"
_TASK3_MISSION_PROFILE_ID = "task3-stability"
_TASK3_DEPLOYMENT_PRESET_ID = "field-2026"

# Lidar transport mode
_LIDAR_TRANSPORT = "mid360"

# Launch file names
_LIDAR_LAUNCH = "lidar.launch.py"
_DUAL_UVC_LAUNCH = "dual_uvc.launch.py"
_TARGET_OBSERVATION_LAUNCH = "target_observation.launch.py"


def _lidar_reachable(ip: str) -> bool:
    """Ping the MID-360; mirrors the reachability gate in field_test.sh."""
    try:
        return (
            subprocess.run(
                ["ping", "-c", "1", "-W", "1", ip],
                capture_output=True,
                timeout=3,
                check=False,
            ).returncode
            == 0
        )
    except (OSError, subprocess.SubprocessError):
        return False


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
    camera_share = Path(get_package_share_directory("ed_uav_camera"))
    perception_share = Path(get_package_share_directory("ed_uav_perception"))
    lidar_share = Path(get_package_share_directory("ed_uav_lidar"))
    localization_share = Path(get_package_share_directory("ed_uav_localization"))
    description_share = Path(get_package_share_directory("ed_uav_description"))

    # Resolve launch configurations
    hmac_key_file = LaunchConfiguration("hmac_key_file").perform(context)
    profile_path = LaunchConfiguration("profile_path").perform(context)
    mission_config_path = LaunchConfiguration("mission_config_path").perform(context)
    calibration_file = LaunchConfiguration("calibration_file").perform(context)
    camera_runtime_plan = LaunchConfiguration("camera_runtime_plan").perform(context)
    fast_lio_launch_path = LaunchConfiguration("fast_lio_launch_path").perform(context)
    mid360_driver_config_path = LaunchConfiguration("mid360_driver_config_path").perform(context)
    ros_security_keystore = LaunchConfiguration("ros_security_keystore").perform(context)
    ros_security_enable = LaunchConfiguration("ros_security_enable").perform(context)
    ros_security_strategy = LaunchConfiguration("ros_security_strategy").perform(context)
    simulation_only = LaunchConfiguration("simulation_only").perform(context).lower() in ("true", "1", "yes")
    dry_run = LaunchConfiguration("dry_run").perform(context).lower() in ("true", "1", "yes")
    enable_display = LaunchConfiguration("enable_display").perform(context).lower() in ("true", "1", "yes")
    task3_immediate_start = LaunchConfiguration("task3_immediate_start").perform(context).lower() in ("true", "1", "yes")
    lidar_ip = LaunchConfiguration("lidar_ip").perform(context)
    vehicle_bind_host = LaunchConfiguration("vehicle_bind_host").perform(context)

    actions = [
        SetEnvironmentVariable("ROS_SECURITY_ENABLE", ros_security_enable),
        SetEnvironmentVariable("ROS_SECURITY_STRATEGY", ros_security_strategy),
        SetEnvironmentVariable("ROS_SECURITY_KEYSTORE", ros_security_keystore),
    ]

    # ── 1. 车辆/地面站 UDP 桥 ─────────────────────────────────────────────
    profile_document = yaml.safe_load(Path(profile_path).read_text(encoding="utf-8"))
    field_profile_id = str(profile_document["profile_id"])

    actions.append(
        Node(
            package="ed_uav_vehicle_bridge",
            executable="vehicle_bridge",
            name="vehicle_bridge",
            output="screen",
            parameters=[
                {
                    "bind_host": vehicle_bind_host,
                    "bind_port": 42000,
                    "car_peer_host": CAR_IP,
                    "car_peer_port": 42001,
                    "hmi_peer_host": HMI_IP,
                    "hmi_peer_port": 42002,
                    "car_sender_id": CAR_SENDER_ID,
                    "hmi_sender_id": HMI_SENDER_ID,
                    "bridge_sender_id": BRIDGE_SENDER_ID,
                    "hmac_key_file": hmac_key_file,
                    "mission_timeout_seconds": 90.0,
                    "telemetry_stale_seconds": 0.75,
                    "task3_flight_test_mode": not simulation_only,
                    "task3_immediate_start": task3_immediate_start,
                    "task3_mission_id": _TASK3_MISSION_PROFILE_ID,
                    "task3_field_profile_id": field_profile_id,
                    "task3_mission_profile_id": _TASK3_MISSION_PROFILE_ID,
                    "task3_deployment_preset_id": _TASK3_DEPLOYMENT_PRESET_ID,
                    "task3_target_revision": _TARGET_REVISION,
                }
            ],
        )
    )

    # ── 1.5 Dry-run vehicle telemetry simulator ──────────────────────────
    if dry_run:
        actions.append(
            Node(
                package="ed_uav_vehicle_bridge",
                executable="dry_run_telemetry",
                name="dry_run_telemetry",
                output="screen",
            )
        )

    # ── 2. 飞控串口桥 (FlightCommand action server) — dry-run 跳过 ──────
    if not dry_run:
        actions.append(
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
                        "enable_flight_commands": True,
                        "enable_realtime_control": True,
                        "enable_programmable_commands": False,
                    }
                ],
            )
        )

    # ── 3. H7 GPIO 桥 (电磁铁/激光, 0xAA 协议) ────────────────────────
    h7_device_present = Path(h7_port).exists()
    if h7_device_present:
        actions.append(
            Node(
                package="ed_uav_fcu_bridge",
                executable="ed_uav_h7_gpio_bridge",
                name="ed_uav_h7_gpio_bridge",
                output="screen",
                parameters=[{"serial_port": h7_port, "baudrate": 115200}],
            )
        )
    else:
        logging.getLogger("launch").warning("%s 不存在 — H7 GPIO (电磁铁/激光) 跳过", h7_port)

    # ── 4. 任务执行器 (D-task 行为树) ──────────────────────────────────
    actions.append(
        Node(
            package="ed_uav_mission",
            executable="mission_executor",
            name="mission_executor",
            output="screen",
            arguments=["--ros-args", "--enclave", "/ed_uav_mission_executor"],
            parameters=[
                {
                    "use_sim_time": simulation_only,
                    "profile_path": profile_path,
                    "mission_config_path": mission_config_path,
                    "calibration_file": calibration_file,
                    "simulation_only": simulation_only,
                    "payload_config_path": str(mission_share / "config" / "payload_adapter.yaml"),
                    "payload_actuator": "fake" if simulation_only else "h7",
                    "programmable_capability_report": "",
                    "fcu_device_identity": "",
                    "task3_mission_profile_id": _TASK3_MISSION_PROFILE_ID,
                    "task3_deployment_preset_id": _TASK3_DEPLOYMENT_PRESET_ID,
                    "task3_target_revision": _TARGET_REVISION,
                }
            ],
            remappings=[
                ("/vehicle/telemetry", "/d_task/vehicle/telemetry"),
                ("/mission/status", "/d_task/mission_status"),
                ("/mission/select_d_task", "/d_task/pre_arm/select_mission"),
            ],
        )
    )

    # ── 5. 定位融合 (source_supervisor + field_anchor) ──────────────────
    actions.append(
        Node(
            package="ed_uav_localization",
            executable="source_supervisor",
            name="source_supervisor",
            output="screen",
            parameters=[{"use_sim_time": simulation_only}],
        )
    )
    actions.append(
        Node(
            package="ed_uav_localization",
            executable="field_anchor",
            name="field_anchor",
            output="screen",
            parameters=[{"use_sim_time": simulation_only, "profile_path": profile_path}],
        )
    )

    # ── 6. 雷达 + FAST-LIO — dry-run 时 gate on MID-360 reachability ───
    lidar_chain_up = not dry_run or _lidar_reachable(lidar_ip)
    if not lidar_chain_up:
        logging.getLogger("launch").info(
            "MID-360 %s 不可达 — 雷达/FAST-LIO/lio_adapter 跳过 (dry-run)", lidar_ip
        )
    if lidar_chain_up:
        repo_root = Path(__file__).resolve().parents[4]
        extrinsics = repo_root / "ros2_ws" / "src" / "ed_uav_lidar" / "config" / "fields" / "field_extrinsics.yaml"
        actions.append(
            Node(
                package="ed_uav_localization",
                executable="lio_adapter",
                name="lio_adapter",
                output="screen",
                parameters=[{"calibration_file": str(extrinsics)}],
            )
        )
        mid360_driver_config_path = Path(mid360_driver_config_path)
        lidar_manifest = json.loads(mid360_driver_config_path.read_text(encoding="utf-8"))
        lidar_serial = str(lidar_manifest["serial_number"])
        lidar_sensor_ip = str(lidar_manifest["lidar_ip"])
        lidar_host_ip = str(lidar_manifest["host_ip"])
        lidar_firmware = str(lidar_manifest["firmware"])
        lidar_driver_json = Path(lidar_manifest["driver_json"])
        if not lidar_driver_json.is_absolute():
            lidar_driver_json = mid360_driver_config_path.parent / lidar_driver_json
        actions.append(SetEnvironmentVariable("MID360_HOST_IP", lidar_host_ip))
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(lidar_share / "launch" / _LIDAR_LAUNCH)),
                launch_arguments={
                    "lidar_enabled": "true",
                    "transport": _LIDAR_TRANSPORT,
                    "serial_number": lidar_serial,
                    "sensor_ip": lidar_sensor_ip,
                    "firmware_version": lidar_firmware,
                    "time_authority": "host",
                    "driver_config_path": str(lidar_driver_json),
                }.items(),
            )
        )
        fast_lio = Path(fast_lio_launch_path)
        if not fast_lio.is_absolute():
            fast_lio = lidar_share / "config" / "fields" / fast_lio.name
        actions.append(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(fast_lio)))
        )

    # ── 7. 双相机 — direct capture (v4l2_camera 无法处理 MJPG) ────────
    if camera_runtime_plan:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(camera_share / "launch" / _DUAL_UVC_LAUNCH)),
                launch_arguments={
                    "camera_plan": camera_runtime_plan,
                    "use_direct_capture": "true",
                }.items(),
            )
        )

    # ── 8. 感知目标观测 (AprilTag PnP) ────────────────────────────────
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(perception_share / "launch" / _TARGET_OBSERVATION_LAUNCH)),
            launch_arguments={
                "target_revision": _TARGET_REVISION,
                "vehicle_topic": "/d_task/vehicle/telemetry",
            }.items(),
        )
    )

    # ── 9. 任务显示 (可选) ────────────────────────────────────────────
    if enable_display:
        actions.append(
            Node(
                package="ed_uav_mission",
                executable="mission_display",
                name="mission_display",
                output="screen",
                arguments=["--ros-args", "--enclave", "/ed_uav_mission_display"],
                parameters=[{"max_display_width": 960, "headless_log_interval_sec": 5.0}],
                remappings=[("/mission/status", "/d_task/mission_status")],
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    """Expose the full competition launch surface."""
    mission_share = Path(get_package_share_directory("ed_uav_mission"))
    localization_share = Path(get_package_share_directory("ed_uav_localization"))
    description_share = Path(get_package_share_directory("ed_uav_description"))
    repo_root = Path(__file__).resolve().parents[4]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "mission_config_path",
                default_value=str(mission_share / "config" / "missions" / "d_arena_stability_test.yaml"),
            ),
            DeclareLaunchArgument(
                "profile_path",
                default_value=str(localization_share / "config" / "fields" / "d_arena_2026.yaml"),
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=str(description_share / "config" / "example_uncalibrated.yaml"),
            ),
            DeclareLaunchArgument(
                "camera_runtime_plan",
                default_value=str(repo_root / "calibration_data" / "camera_runtime_plan.local.json"),
            ),
            DeclareLaunchArgument("simulation_only", default_value="false"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument(
                "payload_config_path",
                default_value=str(mission_share / "config" / "payload_adapter.yaml"),
            ),
            DeclareLaunchArgument("hmac_key_file", default_value=str(repo_root / "config" / "hmac.key.hex")),
            DeclareLaunchArgument("ros_security_enable", default_value="true"),
            DeclareLaunchArgument("ros_security_strategy", default_value="Enforce"),
            DeclareLaunchArgument("ros_security_keystore", default_value=str(repo_root / "keystore")),
            DeclareLaunchArgument(
                "mid360_driver_config_path",
                default_value=str(repo_root / "ros2_ws" / "src" / "ed_uav_lidar" / "config" / "fields" / "mid360_field_manifest.local.json"),
            ),
            DeclareLaunchArgument(
                "fast_lio_launch_path",
                default_value=str(repo_root / "ros2_ws" / "src" / "ed_uav_lidar" / "config" / "fields" / "fast_lio.launch.py"),
            ),
            DeclareLaunchArgument("lidar_ip", default_value="192.168.1.3"),
            DeclareLaunchArgument("task3_immediate_start", default_value="false"),
            DeclareLaunchArgument("vehicle_bind_host", default_value=NUC_IP),
            DeclareLaunchArgument("enable_display", default_value="false"),
            DeclareLaunchArgument("image_topic", default_value="/camera/narrow/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/narrow/camera_info"),
            # 串口自动检测
            DeclareLaunchArgument("auto_detect_serial", default_value="true"),
            DeclareLaunchArgument("fcu_serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("h7_serial_port", default_value="/dev/ttyUSB1"),
            OpaqueFunction(function=_resolve_serial_ports),
        ]
    )
