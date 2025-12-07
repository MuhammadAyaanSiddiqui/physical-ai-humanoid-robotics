#!/usr/bin/env python3
"""
Minimal Publisher Example for ROS 2

This example demonstrates:
- Creating a ROS 2 node
- Publishing String messages to a topic
- Using timers for periodic publishing
- Proper logging with ROS 2 logger

Usage:
    ros2 run <your_package> minimal_publisher

To see output:
    ros2 topic echo /chatter
    ros2 topic hz /chatter
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal ROS 2 publisher node that publishes String messages.

    This node creates a publisher that sends "Hello ROS 2: <count>" messages
    to the '/chatter' topic at 1 Hz (once per second).
    """

    def __init__(self):
        """
        Initialize the MinimalPublisher node.

        Sets up:
        - Node name: 'minimal_publisher'
        - Publisher on topic '/chatter' with message type String
        - Timer that triggers publishing every 1.0 seconds
        - Counter for numbering messages
        """
        super().__init__('minimal_publisher')

        # Create publisher
        # Parameters:
        #   - String: message type
        #   - 'chatter': topic name
        #   - 10: queue size (number of messages to buffer)
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create timer
        # Parameters:
        #   - 1.0: timer period in seconds (1 Hz)
        #   - self.timer_callback: function to call each period
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Message counter
        self.i = 0

        # Log startup message
        self.get_logger().info('Minimal Publisher node has started!')
        self.get_logger().info('Publishing to topic: /chatter')

    def timer_callback(self):
        """
        Timer callback function - called every 1.0 seconds.

        Creates and publishes a String message with incrementing counter.
        Logs the published message for debugging.
        """
        # Create message
        msg = String()
        msg.data = f'Hello ROS 2: {self.i}'

        # Publish message
        self.publisher_.publish(msg)

        # Log published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter
        self.i += 1


def main(args=None):
    """
    Main function - entry point for the node.

    This function:
    1. Initializes the ROS 2 Python client library
    2. Creates the MinimalPublisher node
    3. Spins the node (processes callbacks) until shutdown
    4. Cleans up resources

    Args:
        args: Command line arguments (optional)
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create node instance
    minimal_publisher = MinimalPublisher()

    try:
        # Spin the node (process callbacks)
        # This keeps the node running and processing timer callbacks
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        minimal_publisher.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        # Cleanup
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
