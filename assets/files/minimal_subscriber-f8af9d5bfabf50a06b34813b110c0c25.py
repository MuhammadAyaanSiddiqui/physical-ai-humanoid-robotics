#!/usr/bin/env python3
"""
Minimal ROS 2 Subscriber Example
Physical AI & Humanoid Robotics Course - Module 1

This node subscribes to the 'chatter' topic and logs received messages.

Usage:
    python3 minimal_subscriber.py

Prerequisites:
    - ROS 2 Humble installed
    - rclpy package available
    - minimal_publisher.py running in another terminal
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """A simple subscriber node that receives string messages."""

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Create subscription
        # Parameters: message_type, topic_name, callback, queue_size
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

        # Prevent unused variable warning (subscription must be saved)
        self.subscription

        self.get_logger().info('Minimal Subscriber started - listening on /chatter')

    def listener_callback(self, msg):
        """
        Callback function called whenever a message is received.

        Args:
            msg (std_msgs.msg.String): The received message
        """
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Main function to initialize and spin the node."""
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    minimal_subscriber = MinimalSubscriber()

    # Spin the node (keeps it running and processing callbacks)
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
