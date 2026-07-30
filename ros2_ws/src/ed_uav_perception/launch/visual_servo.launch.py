"""Launch file for visual servo precision landing node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "target_topic",
            default_value="/d_task/target_observation",
            description="Topic name for target observation",
        ),
        DeclareLaunchArgument(
            "velocity_topic",
            default_value="/cmd_vel_stamped",
            description="Topic name for velocity commands",
        ),
        DeclareLaunchArgument(
            "enabled",
            default_value="true",
            description="Enable visual servo on startup",
        ),
        DeclareLaunchArgument(
            "approach_kp_xy",
            default_value="0.3",
            description="Proportional gain for approach phase XY",
        ),
        DeclareLaunchArgument(
            "descent_kp_xy",
            default_value="0.5",
            description="Proportional gain for descent phase XY",
        ),
        DeclareLaunchArgument(
            "final_kp_xy",
            default_value="0.8",
            description="Proportional gain for final phase XY",
        ),
        DeclareLaunchArgument(
            "touchdown_kp_xy",
            default_value="1.0",
            description="Proportional gain for touchdown phase XY",
        ),
        DeclareLaunchArgument(
            "position_tolerance_m",
            default_value="0.02",
            description="Position tolerance for convergence",
        ),
        DeclareLaunchArgument(
            "stable_time_sec",
            default_value="0.5",
            description="Time to remain stable before declaring landed",
        ),
        Node(
            package="ed_uav_perception",
            executable="visual_servo_node",
            name="visual_servo_node",
            output="screen",
            parameters=[{
                "target_topic": LaunchConfiguration("target_topic"),
                "velocity_topic": LaunchConfiguration("velocity_topic"),
                "enabled": LaunchConfiguration("enabled"),
                "approach_kp_xy": LaunchConfiguration("approach_kp_xy"),
                "descent_kp_xy": LaunchConfiguration("descent_kp_xy"),
                "final_kp_xy": LaunchConfiguration("final_kp_xy"),
                "touchdown_kp_xy": LaunchConfiguration("touchdown_kp_xy"),
                "position_tolerance_m": LaunchConfiguration("position_tolerance_m"),
                "stable_time_sec": LaunchConfiguration("stable_time_sec"),
            }],
        ),
    ])
