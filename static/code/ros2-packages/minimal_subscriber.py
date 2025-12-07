#!/usr/bin/env python3
"""
Minimal Subscriber Example for ROS 2

This example demonstrates:
- Creating a ROS 2 node
- Subscribing to String messages from a topic
- Processing received messages in a callback
- Proper logging with ROS 2 logger

Usage:
    # Terminal 1: Start subscriber
    ros2 run <your_package> minimal_subscriber

    # Terminal 2: Publish test messages
    ros2 topic pub /chatter std_msgs/String "data: 'Hello from command line'"

    # Or run with minimal_publisher.py
    ros2 run <your_package> minimal_publisher
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A minimal ROS 2 subscriber node that receives String messages.

    This node creates a subscriber that listens to '/chatter' topic
    and logs each received message.
    """

    def __init__(self):
        """
        Initialize the MinimalSubscriber node.

        Sets up:
        - Node name: 'minimal_subscriber'
        - Subscriber on topic '/chatter' with message type String
        - Callback function to process received messages
        """
        super().__init__('minimal_subscriber')

        # Create subscriber
        # Parameters:
        #   - String: message type (must match publisher)
        #   - 'chatter': topic name (must match publisher)
        #   - self.listener_callback: function called when message received
        #   - 10: queue size (number of messages to buffer)
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

        # Prevent unused variable warning
        # (self.subscription is stored to keep the subscription alive)
        self.subscription

        # Message counter
        self.message_count = 0

        # Log startup message
        self.get_logger().info('Minimal Subscriber node has started!')
        self.get_logger().info('Listening to topic: /chatter')
        self.get_logger().info('Waiting for messages...')

    def listener_callback(self, msg):
        """
        Callback function - called whenever a message is received.

        This function is automatically called by ROS 2 when a new message
        arrives on the '/chatter' topic.

        Args:
            msg (std_msgs.msg.String): The received message
        """
        # Increment counter
        self.message_count += 1

        # Log received message
        self.get_logger().info(f'I heard: "{msg.data}" (message #{self.message_count})')

        # You can add custom processing here
        # For example:
        # - Parse the message data
        # - Trigger actions based on content
        # - Store data for later use
        # - Publish to another topic


def main(args=None):
    """
    Main function - entry point for the node.

    This function:
    1. Initializes the ROS 2 Python client library
    2. Creates the MinimalSubscriber node
    3. Spins the node (processes callbacks) until shutdown
    4. Cleans up resources

    Args:
        args: Command line arguments (optional)
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create node instance
    minimal_subscriber = MinimalSubscriber()

    try:
        # Spin the node (process callbacks)
        # This keeps the node running and processing subscriber callbacks
        # The node will run until Ctrl+C is pressed
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        minimal_subscriber.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        # Cleanup
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
