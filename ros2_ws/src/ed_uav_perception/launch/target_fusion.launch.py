"""融合节点launch文件"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('max_position_jump', default_value='0.5'),
        DeclareLaunchArgument('slop', default_value='0.1'),

        Node(
            package='ed_uav_perception',
            executable='target_fusion',
            name='target_fusion',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'max_position_jump': LaunchConfiguration('max_position_jump'),
                'slop': LaunchConfiguration('slop'),
            }],
            output='screen',
        ),
    ])
