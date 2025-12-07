# Depth Estimation

## Overview

**Depth estimation** enables robots to perceive 3D structure from 2D images, answering "how far away is each pixel?" This is critical for navigation (obstacle avoidance), manipulation (grasp planning), and scene understanding. We'll explore both **stereo depth** (using two cameras like human eyes) and **monocular depth** (single camera with deep learning).

**What You'll Learn**:
- Stereo depth estimation principles and algorithms
- Monocular depth estimation with neural networks
- Configure Isaac ROS depth nodes
- Generate 3D point clouds from RGB-D data
- Combine depth with object detection for 3D localization

**Prerequisites**:
- Completed: Object Detection (previous lesson)
- Understanding of camera geometry
- Basic linear algebra (transformations, projections)

**Estimated Time**: 3 hours

---

## Learning Objectives

By the end of this lesson, you will be able to:

1. Explain stereo triangulation and disparity maps
2. Run stereo depth estimation in Isaac ROS
3. Use monocular depth networks (MiDaS, DPT)
4. Generate and visualize 3D point clouds
5. Fuse depth with object detection for 3D bounding boxes
6. Optimize depth estimation for real-time performance

---

## Part 1: Depth Estimation Fundamentals

### Why Depth Matters for Robotics

| Task | Why Depth is Needed |
|------|---------------------|
| **Obstacle Avoidance** | Know distance to obstacles for safe navigation |
| **Grasping** | Calculate 3D position of objects for inverse kinematics |
| **3D Mapping** | Build volumetric maps (octomap) for path planning |
| **Human Detection** | Measure distance to people for safe interaction |

### Stereo vs. Monocular Depth

| Method | Accuracy | Speed | Hardware Cost | Outdoor Performance |
|--------|----------|-------|---------------|---------------------|
| **Stereo** | High (mm precision) | Fast (30+ FPS) | Medium ($100-300) | Excellent |
| **Monocular** | Medium (10-20cm error) | Medium (15-30 FPS) | Low ($50) | Good |
| **LiDAR** | Very High (&lt;1mm) | Fast (10-30 FPS) | High ($500-$10K) | Excellent |

**For this course**: We'll cover both stereo (primary) and monocular (fallback when stereo unavailable).

---

## Part 2: Stereo Depth Estimation

### How Stereo Depth Works

**Principle**: Objects closer to cameras appear at different horizontal positions in left vs. right images. This offset (**disparity**) reveals depth.

**Mathematics**:

```
Depth (meters) = (Baseline × Focal Length) / Disparity

Where:
- Baseline: Distance between left/right cameras (e.g., 0.12m)
- Focal Length: Camera focal length in pixels (e.g., 700px)
- Disparity: Horizontal pixel offset between left/right images
```

**Example**:
- Baseline = 0.12m
- Focal Length = 700px
- Object appears at pixel 400 (left), pixel 380 (right) → Disparity = 20px
- Depth = (0.12 × 700) / 20 = **4.2 meters**

### Step 1: Setup Stereo Camera in Isaac Sim

1. Launch Isaac Sim
2. Load warehouse scene
3. Add **Carter robot** (has built-in stereo camera)
   - Left camera: `/front_stereo_camera/left/image_raw`
   - Right camera: `/front_stereo_camera/right/image_raw`

4. Enable ROS 2 Bridge for stereo topics:
   ```python
   # In Isaac Sim Script Editor
   import omni.isaac.ros2_bridge as ros_bridge

   ros_bridge.enable_topic('/front_stereo_camera/left/image_raw')
   ros_bridge.enable_topic('/front_stereo_camera/right/image_raw')
   ros_bridge.enable_topic('/front_stereo_camera/left/camera_info')
   ros_bridge.enable_topic('/front_stereo_camera/right/camera_info')
   ```

### Step 2: Install Isaac ROS Stereo Depth

```bash
cd ~/ros2_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select isaac_ros_stereo_image_proc
source install/setup.bash
```

### Step 3: Run Stereo Depth Node

```bash
ros2 launch isaac_ros_stereo_image_proc isaac_ros_stereo_image_pipeline.launch.py \
  left_image_topic:=/front_stereo_camera/left/image_raw \
  right_image_topic:=/front_stereo_camera/right/image_raw \
  left_camera_info_topic:=/front_stereo_camera/left/camera_info \
  right_camera_info_topic:=/front_stereo_camera/right/camera_info \
  output_topic:=/depth_image
```

**Output Topics**:
- `/depth_image` - Depth map (16-bit, distance in mm)
- `/disparity_image` - Disparity map (visualization)
- `/point_cloud` - 3D point cloud

### Step 4: Visualize Depth in RViz

```bash
rviz2
```

Add displays:
1. **Image** → Topic: `/depth_image` (set color scheme to "Rainbow")
2. **PointCloud2** → Topic: `/point_cloud`

You should see:
- Depth image: Closer objects appear red, farther objects blue
- Point cloud: 3D visualization of the scene

---

## Part 3: Monocular Depth Estimation

When stereo cameras aren't available, use monocular depth estimation (single camera + deep learning).

### Step 1: Install Isaac ROS Depth Segmentation

```bash
cd ~/ros2_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_depth_segmentation.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select isaac_ros_bi3d isaac_ros_depth_segmentation
source install/setup.bash
```

### Step 2: Download Pre-trained MiDaS Model

**MiDaS** is a state-of-the-art monocular depth network.

```bash
mkdir -p ~/models/midas
cd ~/models/midas

# Download MiDaS v3.0 (DPT-Hybrid model)
wget https://github.com/isl-org/MiDaS/releases/download/v3_0/dpt_hybrid-midas-501f0c75.pt

# Convert to TensorRT (optional for GPU acceleration)
python3 convert_midas_to_trt.py dpt_hybrid-midas-501f0c75.pt
```

### Step 3: Run Monocular Depth Node

```bash
ros2 launch isaac_ros_depth_segmentation isaac_ros_depth_segmentation.launch.py \
  model_file_path:=~/models/midas/dpt_hybrid-midas-501f0c75.engine \
  input_image_topic:=/front_camera/image_raw \
  output_topic:=/monocular_depth
```

**Performance**: 15-25 FPS on RTX 4070 Ti

---

## Part 4: 3D Point Cloud Generation

Depth maps are 2D (image-like). Convert to 3D point clouds for robotics applications.

### Using ROS 2 depth_image_proc

```bash
# Install depth image processing tools
sudo apt install ros-humble-depth-image-proc

# Launch point cloud converter
ros2 run depth_image_proc point_cloud_xyz_node \
  --ros-args \
  --remap depth/image_rect:=/depth_image \
  --remap depth/camera_info:=/front_stereo_camera/left/camera_info \
  --remap points:=/point_cloud_xyz
```

**Output**: `/point_cloud_xyz` contains `sensor_msgs/PointCloud2` messages

### Python Script for Point Cloud Processing

```python
import rclpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor:
    def __init__(self):
        self.node = rclpy.create_node('pc_processor')
        self.sub = self.node.create_subscription(
            PointCloud2,
            '/point_cloud_xyz',
            self.callback,
            10
        )

    def callback(self, msg):
        # Convert ROS PointCloud2 to list of (x, y, z)
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        # Filter points within range (e.g., 0.5m to 5m)
        filtered = [(x, y, z) for x, y, z in points if 0.5 < z < 5.0]

        print(f"Received {len(points)} points, filtered to {len(filtered)}")

        # Find closest obstacle
        if filtered:
            closest = min(filtered, key=lambda p: p[2])  # Minimum Z distance
            print(f"Closest obstacle at: {closest[2]:.2f}m")

rclpy.init()
processor = PointCloudProcessor()
rclpy.spin(processor.node)
```

---

## Part 5: Combining Depth + Object Detection

**Goal**: Get 3D position of detected objects (e.g., "Apple is 1.2m away at 30° right").

### Step 1: Synchronize Detection and Depth

```python
from message_filters import ApproximateTimeSynchronizer, Subscriber
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class Detection3D:
    def __init__(self):
        self.node = rclpy.create_node('detection_3d')
        self.bridge = CvBridge()

        # Subscribe to detections and depth (synchronized)
        det_sub = Subscriber(self.node, Detection2DArray, '/detections')
        depth_sub = Subscriber(self.node, Image, '/depth_image')

        self.sync = ApproximateTimeSynchronizer(
            [det_sub, depth_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.sync.registerCallback(self.callback)

    def callback(self, det_msg, depth_msg):
        # Convert depth image to numpy array
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        for det in det_msg.detections:
            # Get bounding box center
            cx = int(det.bbox.center.x)
            cy = int(det.bbox.center.y)

            # Sample depth at bbox center
            depth_meters = depth_image[cy, cx]

            # Get class and confidence
            class_id = det.results[0].id
            confidence = det.results[0].score

            print(f"Detected object {class_id} at depth {depth_meters:.2f}m (confidence: {confidence:.2f})")

rclpy.init()
node = Detection3D()
rclpy.spin(node.node)
```

### Step 2: Convert to 3D World Coordinates

```python
def pixel_to_3d(pixel_x, pixel_y, depth_meters, camera_info):
    """
    Convert pixel + depth to 3D point in camera frame.

    Args:
        pixel_x, pixel_y: Pixel coordinates
        depth_meters: Depth at that pixel
        camera_info: CameraInfo message (contains intrinsics)

    Returns:
        (x, y, z) in meters (camera frame)
    """
    # Extract camera intrinsics
    fx = camera_info.k[0]  # Focal length X
    fy = camera_info.k[4]  # Focal length Y
    cx = camera_info.k[2]  # Principal point X
    cy = camera_info.k[5]  # Principal point Y

    # Unproject pixel to 3D
    z = depth_meters
    x = (pixel_x - cx) * z / fx
    y = (pixel_y - cy) * z / fy

    return (x, y, z)

# Usage
camera_info = ...  # Receive from /front_camera/camera_info
bbox_center_x = det.bbox.center.x
bbox_center_y = det.bbox.center.y
depth = depth_image[int(bbox_center_y), int(bbox_center_x)]

x, y, z = pixel_to_3d(bbox_center_x, bbox_center_y, depth, camera_info)
print(f"Object 3D position: ({x:.2f}, {y:.2f}, {z:.2f}) meters")
```

---

## Part 6: Hands-On Exercise

### Exercise: 3D Object Localization

**Objective**: Detect objects and report their 3D positions for robot grasping.

#### Requirements

1. **Scene**: Tabletop with 5 objects (apples, mugs)
2. **Detection**: Run YOLOv8 object detector (from previous lesson)
3. **Depth**: Run stereo depth estimation
4. **Fusion**: Combine detection + depth to get 3D positions
5. **Output**: For each detected object, print:
   ```
   Apple: (x=0.45m, y=-0.12m, z=0.82m) - confidence: 0.93
   Mug: (x=0.67m, y=0.23m, z=0.79m) - confidence: 0.88
   ```

#### Deliverables

- Python script: `object_3d_localization.py`
- RViz screenshot showing:
  - RGB image with 2D bounding boxes
  - Point cloud with objects visible
- Accuracy test: Manually measure 3 objects' distances, compare with depth estimates (error &lt;10cm)

**Estimated Time**: 2 hours

---

## Summary

In this lesson, you learned:

✅ Stereo depth estimation via triangulation
✅ Monocular depth estimation with neural networks (MiDaS)
✅ 3D point cloud generation from depth maps
✅ Combining depth with object detection for 3D localization
✅ Real-time depth processing with Isaac ROS

**Key Takeaways**:
- **Stereo depth** provides high accuracy for robotics applications
- **Monocular depth** is a viable fallback with single cameras
- **Depth + detection** enables 3D scene understanding
- **Isaac ROS** GPU acceleration achieves real-time performance

---

## Next Steps

In the next lesson (**Pose Estimation**), you'll learn:

- 6DOF pose estimation (position + orientation)
- Object pose from RGB-D data
- Grasp pose generation for manipulation
- Pose tracking across frames

---

## Additional Resources

- **Stereo Vision Tutorial**: https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html
- **MiDaS Paper**: https://arxiv.org/abs/1907.01341
- **Isaac ROS Depth**: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_depth_segmentation/index.html
- **Point Cloud Processing**: http://www.open3d.org/docs/latest/tutorial/geometry/pointcloud.html
- **3D Vision Fundamentals**: Multiple View Geometry in Computer Vision (Hartley & Zisserman)

:::tip Real-World Example
Self-driving cars use stereo depth (and LiDAR) to measure distances to pedestrians, vehicles, and obstacles in real-time at 30+ FPS. The same techniques work for humanoid robot navigation!
:::
