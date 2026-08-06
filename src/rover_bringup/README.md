Gazebo integration
-----------------

Quick steps to run the packaged Gazebo world and spawn the rover:

1. Build the workspace:

   ```bash
   colcon build --symlink-install
   ```

2. Source the install overlay:

   ```bash
   source install/setup.bash
   ```

3. Launch Gazebo with the packaged world and spawn the rover:

   ```bash
   ros2 launch rover_bringup rover_gazebo.launch.py
   ```

Prerequisites
-------------

- Install a ROS 2 Gazebo integration package appropriate for your ROS distribution (for example `gazebo_ros` / `ros-<distro>-gazebo-ros-pkgs`).
