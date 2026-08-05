#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class BaseSimulatorNode(Node):
    """
    Simple planar kinematic simulator.

    It assumes that linear.x is forward velocity and angular.z is
    angular velocity around the vertical axis.
    """

    def __init__(self) -> None:
        super().__init__('base_simulator_node')

        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        update_rate = float(
            self.get_parameter('update_rate').value
        )
        self.odom_frame = str(
            self.get_parameter('odom_frame').value
        )
        self.base_frame = str(
            self.get_parameter('base_frame').value
        )

        if update_rate <= 0.0:
            raise ValueError('update_rate must be greater than zero')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.last_update_time = self.get_clock().now()

        self.command_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.command_callback,
            10,
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10,
        )

        self.moving_publisher = self.create_publisher(
            Bool,
            '/rover/moving',
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(
            1.0 / update_rate,
            self.update_simulation,
        )

        self.get_logger().info(
            f'Base simulator started at {update_rate:.1f} Hz.'
        )

    def command_callback(self, message: Twist) -> None:
        self.linear_velocity = float(message.linear.x)
        self.angular_velocity = float(message.angular.z)

    def update_simulation(self) -> None:
        current_time = self.get_clock().now()

        delta_time = (
            current_time - self.last_update_time
        ).nanoseconds / 1e9

        self.last_update_time = current_time

        if delta_time <= 0.0:
            return

        self.x += (
            self.linear_velocity
            * math.cos(self.yaw)
            * delta_time
        )

        self.y += (
            self.linear_velocity
            * math.sin(self.yaw)
            * delta_time
        )

        self.yaw += self.angular_velocity * delta_time

        # Keep yaw in the interval <-pi, pi>.
        self.yaw = math.atan2(
            math.sin(self.yaw),
            math.cos(self.yaw),
        )

        orientation = self.yaw_to_quaternion(self.yaw)

        self.publish_odometry(
            current_time,
            orientation,
        )

        self.publish_transform(
            current_time,
            orientation,
        )

        self.publish_moving_state()

    def publish_odometry(
        self,
        current_time,
        orientation: Quaternion,
    ) -> None:
        message = Odometry()

        message.header.stamp = current_time.to_msg()
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame

        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation = orientation

        message.twist.twist.linear.x = self.linear_velocity
        message.twist.twist.angular.z = self.angular_velocity

        self.odom_publisher.publish(message)

    def publish_transform(
        self,
        current_time,
        orientation: Quaternion,
    ) -> None:
        transform = TransformStamped()

        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation = orientation

        self.tf_broadcaster.sendTransform(transform)

    def publish_moving_state(self) -> None:
        message = Bool()

        message.data = (
            abs(self.linear_velocity) > 1e-3
            or abs(self.angular_velocity) > 1e-3
        )

        self.moving_publisher.publish(message)

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        quaternion = Quaternion()

        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)

        return quaternion


def main(args=None) -> None:
    rclpy.init(args=args)

    node = BaseSimulatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
