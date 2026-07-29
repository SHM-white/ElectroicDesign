from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('ed_uav_gazebo'))
    world = package_share / 'worlds' / 'ed_uav_arena.sdf'
    bridge = package_share / 'config' / 'bridge.yaml'
    localization_share = Path(get_package_share_directory('ed_uav_localization'))
    calibration = Path(get_package_share_directory('ed_uav_description')) / 'config' / 'synthetic_calibrated.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py')), launch_arguments={'gz_args': f'-r -s {world}'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gazebo_bridge', output='screen', parameters=[{'config_file': str(bridge), 'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package_share / 'fast_lio_simulation.launch.py')), launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'), 'calibration_file': str(calibration)}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(localization_share / 'launch' / 'localization_simulation.launch.py')), launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'), 'profile_path': str(localization_share / 'config' / 'fields' / 'simulation_arena.yaml')}.items()),
    ])
