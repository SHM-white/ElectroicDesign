"""Launch the authenticated UDP vehicle/HMI bridge with local provisioning."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = (
    ("bind_host", "0.0.0.0"),
    ("bind_port", "0"),
    ("car_peer_host", ""),
    ("car_peer_port", "0"),
    ("hmi_peer_host", ""),
    ("hmi_peer_port", "0"),
    ("car_sender_id", ""),
    ("hmi_sender_id", ""),
    ("bridge_sender_id", ""),
    ("hmac_key_file", ""),
    ("mission_timeout_seconds", "90.0"),
    ("telemetry_stale_seconds", "0.75"),
)


def generate_launch_description() -> LaunchDescription:
    declarations = [
        DeclareLaunchArgument(name, default_value=default) for name, default in ARGUMENTS
    ]
    parameters = {name: LaunchConfiguration(name) for name, _ in ARGUMENTS}
    return LaunchDescription(
        declarations
        + [
            Node(
                package="ed_uav_vehicle_bridge",
                executable="vehicle_bridge",
                name="vehicle_bridge",
                output="screen",
                parameters=[parameters],
            )
        ]
    )
