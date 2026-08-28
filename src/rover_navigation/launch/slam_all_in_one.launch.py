import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Cesta ke konfiguračnímu souboru SLAM Toolboxu
    default_slam_params = '/workspace/src/rover_navigation/slam_configs/mapper_params_online_async.yaml'

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='Full path to the ROS2 parameters file for slam_toolbox'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    # 1. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # 2. Statická transformace (base_link -> laser)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser'
        ],
        output='screen'
    )

    # 3. RPLiDAR A3
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rplidar_ros'), '/launch/rplidar_a3_launch.py'
        ])
    )

    # 4. RF2O Laser Odometry
    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rf2o_laser_odometry'), '/launch/rf2o_laser_odometry.launch.py'
        ]),
        launch_arguments={
            'laser_scan_topic': '/scan',
            'base_frame_id': 'base_link'
        }.items()
    )

    # 5. SLAM Toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'), '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    return LaunchDescription([
        slam_params_arg,
        use_sim_time_arg,
        static_tf_node,
        rplidar_launch,
        rf2o_launch,
        slam_toolbox_launch,
        rviz_node
    ])