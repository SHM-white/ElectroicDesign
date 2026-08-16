"""广角相机检测节点launch文件"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('target_tag_id', default_value='0'),
        DeclareLaunchArgument('target_revision', default_value='d2026-apriltag-v1'),
        DeclareLaunchArgument('enable_recording', default_value='false'),
        DeclareLaunchArgument('recording_dir', default_value='debug_recordings'),

        Node(
            package='ed_uav_perception',
            executable='wide_detector',
            name='wide_detector',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'target_tag_id': LaunchConfiguration('target_tag_id'),
                'target_revision': LaunchConfiguration('target_revision'),
                'enable_recording': LaunchConfiguration('enable_recording'),
                'recording_dir': LaunchConfiguration('recording_dir'),
            }],
            output='screen',
        ),
    ])
