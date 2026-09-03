# rover_mavlink_communication

ROS 2 node for guarded ArduPilot Rover control through MAVROS. It requests
GUIDED, checks EKF health, optionally selects EKF source set 2 for RF2O
ExternalNav, optionally arms the rover, and only then forwards `/cmd_vel`.

## Safety defaults

- `autonomous_enabled=false`
- `auto_arm=false`
- `/cmd_vel` timeout: 0.5 s
- RF2O timeout: 0.5 s
- Automatic arming requires the explicit `auto_arm:=true` parameter.

## Run

```bash
ros2 run rover_mavlink_communication rover_mavlink_node --ros-args \
  -p rf2o_topic:=/odom_rf2o \
  -p forward_rf2o_to_mavros:=true \
  -p auto_arm:=false
```

Request autonomous operation:

```bash
ros2 topic pub --once /autonomous_enabled std_msgs/msg/Bool "{data: true}"
```

Disable autonomous operation and request MANUAL:

```bash
ros2 topic pub --once /autonomous_enabled std_msgs/msg/Bool "{data: false}"
```

When RF2O fallback selected `SRC2`, disabling autonomous operation requests
`MANUAL` and restores `SRC1`. It does not automatically disarm the rover.

## Required ArduPilot setup

EKF source set 2 must already be configured for ExternalNav, for example:

```text
EK3_SRC2_POSXY = 6
EK3_SRC2_VELXY = 6
EK3_SRC2_YAW   = 6
```

The runtime command only selects source set 2; it does not write these
parameters. MAVROS must load the odometry plugin. The ROS-to-FCU input is
`/mavros/odometry/out`.

Do not enable both `forward_rf2o_to_mavros` and
`enable_external_pose_publisher`; the node suppresses the pose publisher when
both are requested to prevent duplicate ExternalNav inputs.
