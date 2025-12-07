#!/usr/bin/env python3
"""
YOLOv8 Object Detection with Isaac ROS
Processes camera stream and publishes detections
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.bridge = CvBridge()

        # Subscriptions
        self.create_subscription(Image, '/front_camera/image_raw', self.image_callback, 10)
        self.create_subscription(Detection2DArray, '/detections', self.detection_callback, 10)

        # Publishers (for annotated image)
        self.annotated_pub = self.create_publisher(Image, '/annotated_image', 10)

        self.latest_detections = None
        self.get_logger().info("Object Detection Node Started")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Draw bounding boxes if detections available
        if self.latest_detections:
            cv_image = self.draw_detections(cv_image, self.latest_detections)

        # Publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.annotated_pub.publish(annotated_msg)

    def detection_callback(self, msg):
        self.latest_detections = msg
        self.get_logger().info(f"Detected {len(msg.detections)} objects")

        for det in msg.detections:
            class_id = det.results[0].id
            confidence = det.results[0].score
            bbox = det.bbox

            self.get_logger().info(
                f"  Class {class_id}: confidence={confidence:.2f}, "
                f"center=({bbox.center.x:.0f}, {bbox.center.y:.0f})"
            )

    def draw_detections(self, image, detections):
        for det in detections.detections:
            bbox = det.bbox
            confidence = det.results[0].score
            class_id = det.results[0].id

            # Calculate bounding box corners
            x1 = int(bbox.center.x - bbox.size_x / 2)
            y1 = int(bbox.center.y - bbox.size_y / 2)
            x2 = int(bbox.center.x + bbox.size_x / 2)
            y2 = int(bbox.center.y + bbox.size_y / 2)

            # Draw rectangle
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw label
            label = f"ID{class_id}: {confidence:.2f}"
            cv2.putText(image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return image

def main():
    rclpy.init()
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
