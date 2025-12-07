#!/usr/bin/env python3
"""
Visual SLAM configuration example for Isaac ROS
Demonstrates stereo VSLAM setup with Carter robot
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

class VSLAMExample(Node):
    def __init__(self):
        super().__init__('vslam_example')

        # Subscriptions
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.odom_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("VSLAM Example Node Started")

    def odom_callback(self, msg):
        # Log robot pose from VSLAM
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        self.get_logger().info(f"VSLAM Pose: x={x:.2f}, y={y:.2f}, z={z:.2f}")

    def drive_square_pattern(self):
        """Drive robot in square to test VSLAM"""
        commands = [
            (0.3, 0.0, 3.0),   # Forward 3 seconds
            (0.0, 0.785, 2.0), # Turn 90° (pi/4 rad/s for 2s)
            (0.3, 0.0, 3.0),   # Forward
            (0.0, 0.785, 2.0), # Turn
            (0.3, 0.0, 3.0),
            (0.0, 0.785, 2.0),
            (0.3, 0.0, 3.0),
            (0.0, 0.785, 2.0),
        ]

        for linear, angular, duration in commands:
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular

            start = time.time()
            while time.time() - start < duration:
                self.cmd_pub.publish(msg)
                time.sleep(0.1)

        # Stop
        self.cmd_pub.publish(Twist())
        self.get_logger().info("Square pattern complete")

def main():
    rclpy.init()
    node = VSLAMExample()

    # Start driving after 2 seconds
    time.sleep(2.0)
    node.drive_square_pattern()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
