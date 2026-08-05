#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time


class SafetyWatchdogNode(Node):
    """
    Forwards fresh velocity commands and stops the rover when they expire.
    """

    def __init__(self) -> None:
        super().__init__('safety_watchdog_node')

        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('check_rate', 20.0)

        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        check_rate = float(
            self.get_parameter('check_rate').value
        )

        if self.command_timeout <= 0.0:
            raise ValueError(
                'command_timeout must be greater than zero'
            )

        if check_rate <= 0.0:
            raise ValueError(
                'check_rate must be greater than zero'
            )

        self.output_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10,
        )

        self.command_subscription = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.command_callback,
            10,
        )

        self.last_command_time: Optional[Time] = None
        self.watchdog_active = False

        self.timer = self.create_timer(
            1.0 / check_rate,
            self.check_command_age,
        )

        self.get_logger().info(
            f'Safety watchdog started. '
            f'Timeout: {self.command_timeout:.2f} s'
        )

    def command_callback(self, message: Twist) -> None:
        self.last_command_time = self.get_clock().now()
        self.output_publisher.publish(message)

        if self.watchdog_active:
            self.get_logger().info(
                'Velocity command communication restored.'
            )

        self.watchdog_active = False

    def check_command_age(self) -> None:
        if self.last_command_time is None:
            self.publish_stop()
            return

        current_time = self.get_clock().now()

        command_age = (
            current_time - self.last_command_time
        ).nanoseconds / 1e9

        if command_age > self.command_timeout:
            self.publish_stop()

            if not self.watchdog_active:
                self.get_logger().warning(
                    f'Command timeout after {command_age:.2f} s. '
                    'Stopping rover.'
                )
                self.watchdog_active = True

    def publish_stop(self) -> None:
        self.output_publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)

    node = SafetyWatchdogNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
