---
sidebar_position: 3
---

# Sensor Ecosystems

## Introduction

Sensors are the eyes, ears, and touch of Physical AI systems. They transform physical phenomena (light, sound, motion) into digital signals that robots can process. Understanding sensor capabilities and limitations is fundamental to building robust Physical AI systems.

## Core Sensor Modalities

### 1. Vision Sensors (Cameras)

**Types**:
- **Monocular**: Single camera (cheap, limited depth perception)
- **Stereo**: Two cameras (depth from disparity)
- **RGB-D**: Color + depth (e.g., Intel RealSense, Microsoft Kinect)
- **Event cameras**: Asynchronous pixel-level changes (high-speed, low latency)

**Specifications**:
- **Resolution**: 640x480 (VGA) to 3840x2160 (4K)
- **Frame Rate**: 30 FPS (standard) to 240+ FPS (high-speed)
- **Field of View**: 60° (narrow) to 180° (fisheye)

**Use Cases**:
- Object detection and classification
- Semantic segmentation (pixel-level labeling)
- Visual odometry (ego-motion estimation)
- Human pose estimation

**Advantages**:
- Rich semantic information (colors, textures)
- Passive sensing (no emitted signals)
- Low cost ($10-$500)

**Limitations**:
- Sensitive to lighting conditions
- Computationally intensive processing
- Ambiguity in depth (monocular)

### 2. LiDAR (Light Detection and Ranging)

**How It Works**:
- Emit laser pulses
- Measure time-of-flight to obstacles
- Generate 3D point clouds

**Types**:
- **2D LiDAR**: Scans a single plane (navigation)
- **3D LiDAR**: Scans multiple planes (mapping)
- **Solid-state**: No moving parts (automotive)

**Specifications**:
- **Range**: 5m (indoor) to 200m+ (automotive)
- **Accuracy**: ±2cm to ±10cm
- **Scan Rate**: 5 Hz to 100+ Hz

**Use Cases**:
- SLAM (Simultaneous Localization and Mapping)
- Obstacle detection
- Terrain mapping
- Autonomous driving

**Advantages**:
- Accurate 3D measurements
- Works in low light
- Long range

**Limitations**:
- Expensive ($100-$10,000+)
- Struggles with reflective/transparent surfaces
- Large, heavy (mechanical LiDAR)

### 3. IMU (Inertial Measurement Unit)

**Components**:
- **Accelerometer**: Measures linear acceleration
- **Gyroscope**: Measures angular velocity
- **Magnetometer**: Measures magnetic field (optional)

**Specifications**:
- **Degrees of Freedom**: 6-DOF (accel + gyro) or 9-DOF (+ mag)
- **Update Rate**: 100 Hz to 1000+ Hz
- **Noise**: 0.01°/s (high-end) to 1°/s (low-cost)

**Use Cases**:
- Attitude estimation (roll, pitch, yaw)
- Dead reckoning
- Vibration monitoring
- Fall detection

**Advantages**:
- High update rate
- Small, lightweight
- Low cost ($5-$100)

**Limitations**:
- Drift over time (integration errors)
- No absolute position
- Sensitive to vibrations

## Sensor Fusion

Combining multiple sensors overcomes individual limitations:

### Example: Visual-Inertial Odometry (VIO)

**Problem**: Cameras struggle with fast motion (blur); IMUs drift over time
**Solution**: Fuse camera poses with IMU measurements

**Benefits**:
- More accurate than either sensor alone
- Robust to camera occlusions
- High-frequency pose estimates

**Algorithms**: Extended Kalman Filter (EKF), particle filters, graph-based optimization

## Sensor Comparison Table

| Sensor | Range | Accuracy | Cost | Power | Frame Rate | Weather Robust? |
|--------|-------|----------|------|-------|------------|-----------------|
| **Monocular Camera** | 1-50m | Low (depth) | $10-$100 | Low | 30-240 FPS | No (lighting) |
| **Stereo Camera** | 1-50m | Medium | $50-$500 | Low | 30-90 FPS | No (lighting) |
| **RGB-D Camera** | 0.5-10m | High (depth) | $100-$300 | Medium | 30-90 FPS | No (lighting, IR interference) |
| **2D LiDAR** | 5-100m | High | $100-$2,000 | Medium | 5-40 Hz | Yes |
| **3D LiDAR** | 5-200m | Very High | $1,000-$100,000 | High | 10-20 Hz | Yes |
| **IMU (6-DOF)** | N/A | Medium (drift) | $5-$100 | Very Low | 100-1000 Hz | Yes |
| **GPS** | Global | 5-50m | $20-$500 | Low | 1-10 Hz | Partial (requires sky view) |

## Hardware Selection Guidelines

### Budget Option (Under $500)
- **Camera**: Raspberry Pi Camera Module v2 ($25)
- **2D LiDAR**: RPLiDAR A1 ($100)
- **IMU**: MPU-6050 ($5)
- **Platform**: Raspberry Pi 4 ($55)

### Mid-Range ($500-$2,000)
- **Camera**: Intel RealSense D435i ($200)
- **2D LiDAR**: SICK TiM571 ($1,500)
- **IMU**: Built into RealSense
- **Platform**: NVIDIA Jetson Orin Nano ($500)

### High-End ($2,000-$10,000)
- **Camera**: Stereolabs ZED 2i ($450)
- **3D LiDAR**: Velodyne VLP-16 ($4,000)
- **IMU**: Xsens MTi-30 ($2,000)
- **Platform**: NVIDIA Jetson AGX Orin ($2,000)

## Sensor Integration with ROS 2

All sensors in this course will interface via ROS 2:

### Camera Example (Python)
```python
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Process image here
```

### LiDAR Example (Python)
```python
from sensor_msgs.msg import LaserScan

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        ranges = msg.ranges  # Array of distances
        # Process LiDAR data here
```

### IMU Example (Python)
```python
from sensor_msgs.msg import Imu

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.subscription = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

    def imu_callback(self, msg):
        accel = msg.linear_acceleration
        gyro = msg.angular_velocity
        # Process IMU data here
```

## Hands-On Exercise

**Goal**: Visualize sensor data in RViz

1. Install ROS 2 Humble (covered in Module 1)
2. Launch a camera node:
   ```bash
   ros2 run usb_cam usb_cam_node_exe
   ```
3. Launch RViz:
   ```bash
   ros2 run rviz2 rviz2
   ```
4. Add displays for Image, LaserScan, and IMU
5. Observe how each sensor perceives the same environment

## Next Steps

Continue to [Course Roadmap](./course-roadmap.md) to see how these sensors will be used throughout the course.

---

**Prerequisites**: Basic understanding of robotics
**Estimated Time**: 25 minutes
**Learning Outcomes**: Identify common sensor modalities, compare sensor capabilities, understand sensor fusion benefits
