#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TestMotionNode(Node):
    """
    Generates a simple velocity command for testing.

    This node represents the autonomy layer. Later it can be replaced
    by navigation, waypoint following or obstacle avoidance.
    """

    def __init__(self) -> None:
        super().__init__('test_motion_node')

        self.declare_parameter('linear_velocity', 0.4)
        self.declare_parameter('angular_velocity', 0.2)
        self.declare_parameter('publish_rate', 10.0)

        self.linear_velocity = float(
            self.get_parameter('linear_velocity').value
        )
        self.angular_velocity = float(
            self.get_parameter('angular_velocity').value
        )
        publish_rate = float(
            self.get_parameter('publish_rate').value
        )

        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')

        self.command_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_raw',
            10,
        )

        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_command,
        )

        self.get_logger().info(
            'Test motion node started: '
            f'linear={self.linear_velocity:.2f} m/s, '
            f'angular={self.angular_velocity:.2f} rad/s'
        )

    def publish_command(self) -> None:
        command = Twist()

        command.linear.x = self.linear_velocity
        command.angular.z = self.angular_velocity

        self.command_publisher.publish(command)

    def stop_rover(self) -> None:
        self.command_publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)

    node = TestMotionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_rover()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
