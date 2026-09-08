from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("rf2o_mavros_bridge")) / "config" / "bridge.yaml"

    return LaunchDescription([
        Node(
            package="rf2o_mavros_bridge",
            executable="rf2o_mavros_bridge_node",
            name="rf2o_mavros_bridge",
            output="screen",
            parameters=[str(config)],
        )
    ])
