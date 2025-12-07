#!/usr/bin/env python3
"""
Minimal ROS 2 Publisher Example
Physical AI & Humanoid Robotics Course - Module 1

This node publishes string messages to the 'chatter' topic every 0.5 seconds.

Usage:
    python3 minimal_publisher.py

Prerequisites:
    - ROS 2 Humble installed
    - rclpy package available
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """A simple publisher node that sends string messages."""

    def __init__(self):
        super().__init__('minimal_publisher')

        # Create publisher
        # Parameters: message_type, topic_name, queue_size
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create timer (calls callback every 0.5 seconds)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for message numbering
        self.i = 0

        self.get_logger().info('Minimal Publisher started')

    def timer_callback(self):
        """Callback function called by the timer."""
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log to console
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.i += 1


def main(args=None):
    """Main function to initialize and spin the node."""
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    minimal_publisher = MinimalPublisher()

    # Spin the node (keeps it running and processing callbacks)
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
