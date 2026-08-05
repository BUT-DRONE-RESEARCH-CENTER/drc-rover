#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CommandNode(Node):
    """Publishes a simple test velocity command for the rover."""

    def __init__(self) -> None:
        super().__init__('command_node')

        self.declare_parameter('linear_velocity', 0.3)
        self.declare_parameter('angular_velocity', 0.0)
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
            raise ValueError('Parameter publish_rate must be greater than zero.')

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel_raw',
            10,
        )

        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_command,
        )

        self.get_logger().info(
            f'Command node started: '
            f'linear={self.linear_velocity:.2f} m/s, '
            f'angular={self.angular_velocity:.2f} rad/s'
        )

    def publish_command(self) -> None:
        message = Twist()

        message.linear.x = self.linear_velocity
        message.angular.z = self.angular_velocity

        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = CommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_message = Twist()
        node.publisher.publish(stop_message)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
