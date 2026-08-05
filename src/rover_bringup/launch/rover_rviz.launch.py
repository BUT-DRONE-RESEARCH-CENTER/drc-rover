#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(
        get_package_share_directory('rover_bringup')
    )

    urdf_path = bringup_share / 'urdf' / 'rover.urdf'
    rviz_config_path = bringup_share / 'rviz' / 'rover.rviz'

    robot_description = urdf_path.read_text(encoding='utf-8')

    return LaunchDescription([
        # Publishes fixed transforms defined in the URDF.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
        ),

        # Produces desired test velocity.
        Node(
            package='rover_autonomy',
            executable='test_motion_node',
            name='test_motion_node',
            output='screen',
            parameters=[{
                'linear_velocity': 0.4,
                'angular_velocity': 0.2,
                'publish_rate': 10.0,
            }],
        ),

        # Validates command freshness.
        Node(
            package='rover_controls',
            executable='safety_watchdog_node',
            name='safety_watchdog_node',
            output='screen',
            parameters=[{
                'command_timeout': 0.5,
                'check_rate': 20.0,
            }],
        ),

        # Generates odometry and odom -> base_link TF.
        Node(
            package='rover_controls',
            executable='base_simulator_node',
            name='base_simulator_node',
            output='screen',
            parameters=[{
                'update_rate': 50.0,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }],
        ),

        # Visualizes the robot.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                str(rviz_config_path),
            ],
        ),
    ])
