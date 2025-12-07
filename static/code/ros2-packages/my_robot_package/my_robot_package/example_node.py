#!/usr/bin/env python3
"""
Example Node for my_robot_package

Demonstrates a complete ROS 2 node with:
- Publisher and Subscriber
- Parameters
- Timer
- Logging
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class ExampleNode(Node):
    """
    Example node demonstrating ROS 2 features.
    """

    def __init__(self):
        super().__init__('example_node')

        # Declare parameters
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('message_prefix', 'Hello')

        # Get parameters
        publish_rate = self.get_parameter('publish_rate').value
        self.message_prefix = self.get_parameter('message_prefix').value

        # Create publisher
        self.string_publisher = self.create_publisher(String, 'output_topic', 10)

        # Create subscriber
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create timer
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # Counter
        self.count = 0

        self.get_logger().info('Example node started!')
        self.get_logger().info(f'Publish rate: {publish_rate} Hz')
        self.get_logger().info(f'Message prefix: {self.message_prefix}')

    def timer_callback(self):
        """Publish a message periodically."""
        msg = String()
        msg.data = f'{self.message_prefix} {self.count}'
        self.string_publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands."""
        self.get_logger().info(
            f'Received cmd_vel: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
