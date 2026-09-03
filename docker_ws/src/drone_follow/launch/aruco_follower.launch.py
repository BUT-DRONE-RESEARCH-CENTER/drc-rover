import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('drone_follow')
    mavros_params_path = os.path.join(pkg_share, 'config', 'mavros_params.yaml')
    calib_file_path = os.path.join(pkg_share, 'config', 'realsense_calib.yaml')
    camera_info_url = f'file://{calib_file_path}'

    # 1. Uzly spouštěné ihned na začátku
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': '/dev/video4',
            'image_size': [1280, 800],
            'camera_info_url': camera_info_url,
        }]
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[mavros_params_path]
    )

    # 2. Uzly, které budou čekat na spuštění předchozích
    aruco_detector_node = Node(
        package='drone_follow',
        executable='aruco_detector',
        name='aruco_detector',
    )

    mavros_handler_node = Node(
        package='drone_follow',
        executable='mavros_handler',
        name='mavros_handler',
        output='screen',
    )

    # 3. Definice posloupnosti spouštění
    # Krok 2: Po startu kamery spustí aruco_detector
    start_aruco_after_camera = RegisterEventHandler(
        OnProcessStart(
            target_action=v4l2_camera_node,
            on_start=[aruco_detector_node]
        )
    )

    # Krok 3: Po startu aruco_detectoru spustí mavros_handler
    start_handler_after_aruco = RegisterEventHandler(
        OnProcessStart(
            target_action=aruco_detector_node,
            on_start=[mavros_handler_node]
        )
    )

    return LaunchDescription([
        v4l2_camera_node,
        mavros_node,
        start_aruco_after_camera,
        start_handler_after_aruco,
    ])