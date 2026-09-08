# rf2o_mavros_bridge

Minimal ROS 2 Humble C++ bridge for an ArduPilot rover through MAVROS.

It performs only two operations:

1. republishes RF2O `nav_msgs/msg/Odometry` from `/odom_rf2o` to
   `/mavros/odometry/in`;
2. converts `/cmd_vel` (`geometry_msgs/msg/Twist`) to MAVROS
   `/mavros/setpoint_velocity/cmd_vel` (`geometry_msgs/msg/TwistStamped`).

For the rover, `linear.x` is forward velocity `v` in m/s and `angular.z` is
yaw rate `w` in rad/s. Both are limited by parameters. A 0.5 s watchdog sends
one zero command after commands stop arriving.

## Build

```bash
cd /workspace
colcon build --packages-select rf2o_mavros_bridge
source install/setup.bash
```

## Run

```bash
ros2 launch rf2o_mavros_bridge bridge.launch.py
```

or directly:

```bash
ros2 run rf2o_mavros_bridge rf2o_mavros_bridge_node --ros-args \
  --params-file src/rf2o_mavros_bridge/config/bridge.yaml
```

## Quick velocity test

Send commands continuously because MAVROS/ArduPilot velocity control requires
a stream of setpoints:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

Verify forwarding:

```bash
ros2 topic echo /mavros/odometry/in
ros2 topic echo /mavros/setpoint_velocity/cmd_vel
```

The node deliberately does not arm the rover, switch it to `GUIDED`, change EKF
sources, or validate estimator status. Configure ArduPilot EKF and MAVROS
separately. MAVROS handles ROS ENU/FLU to MAVLink coordinate conversion; do not
manually negate RF2O axes in this bridge.
