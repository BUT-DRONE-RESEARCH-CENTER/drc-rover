from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_bringup')
    world_file = os.path.join(pkg_share, 'worlds', 'rover_world.sdf')

    # First, try to locate a gazebo_ros launch file and use it if present.
    try:
        gazebo_pkg_share = get_package_share_directory('gazebo_ros')
        gazebo_launch = os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')
        if os.path.exists(gazebo_launch):
            spawn_cmd = [
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-file', os.path.join(pkg_share, 'urdf', 'rover.urdf'),
                '-entity', 'rover'
            ]
            return LaunchDescription([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(gazebo_launch),
                    launch_arguments={'world': world_file}.items(),
                ),
                ExecuteProcess(cmd=spawn_cmd, output='screen'),
            ])
    except Exception:
        pass

    # Next, if gazebo_ros isn't available, attempt a ros_gz spawn command
    # while *not* assuming a ros_gz launch file exists in the package.
    try:
        # This will succeed if the ros_gz package is installed (even if it has no launch files)
        get_package_share_directory('ros_gz')
        spawn_cmd_ros_gz = [
            'ros2', 'run', 'ros_gz', 'create',
            '-file', os.path.join(pkg_share, 'urdf', 'rover.urdf'),
            '-name', 'rover'
        ]

        # Log a helpful note: user may need to start ros_gz simulator separately.
        note = (
            'ros_gz package found. If the simulator is not running, start it separately.\n'
            'Example to inspect available ros_gz executables:\n'
            '  ros2 pkg executables ros_gz\n'
            'To spawn the robot (if simulator running):\n'
            '  ros2 run ros_gz create -file ' + os.path.join(pkg_share, 'urdf', 'rover.urdf') + ' -name rover\n'
        )

        return LaunchDescription([
            LogInfo(msg=note),
            ExecuteProcess(cmd=spawn_cmd_ros_gz, output='screen'),
        ])
    except Exception:
        # Neither integration found; instruct the user how to install or run the simulator.
        msg = (
            'Neither gazebo_ros nor ros_gz packages were detected.\n'
            'If you use ros_gz (ros-jazzy-ros-gz) install it with:\n'
            '  sudo apt update && sudo apt install ros-jazzy-ros-gz\n'
            'Or install the classic Gazebo ROS integration (gazebo_ros) for your distro.\n'
            'Once installed, re-source and re-run this launch.\n'
            'Alternatively start your simulator manually and then spawn the robot with:\n'
            '  ros2 run ros_gz create -file ' + os.path.join(pkg_share, 'urdf', 'rover.urdf') + ' -name rover'
        )
        return LaunchDescription([LogInfo(msg=msg)])

