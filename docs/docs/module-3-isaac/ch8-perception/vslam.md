# Visual SLAM (VSLAM)

## Overview

**Visual SLAM (Simultaneous Localization and Mapping)** enables robots to build maps of unknown environments while simultaneously tracking their position within those maps, using only camera input. This is a foundational capability for autonomous navigation, allowing robots to operate without GPS or pre-built maps.

**What You'll Learn**:
- Understand VSLAM principles and algorithms (ORB-SLAM, RTAB-Map)
- Configure Isaac ROS VSLAM nodes
- Run visual SLAM in Isaac Sim with camera sensors
- Evaluate map quality and localization accuracy
- Integrate VSLAM with ROS 2 navigation stack

**Prerequisites**:
- Completed Chapter 7 (Isaac Sim fundamentals)
- ROS 2 fundamentals from Module 1
- Understanding of coordinate frames and transformations

**Estimated Time**: 3-4 hours

---

## Learning Objectives

By the end of this lesson, you will be able to:

1. Explain how visual SLAM works (feature extraction, tracking, mapping)
2. Install and configure Isaac ROS VSLAM
3. Run VSLAM in Isaac Sim with simulated cameras
4. Visualize maps and trajectories in RViz
5. Evaluate SLAM performance (drift, loop closure)
6. Export maps for autonomous navigation

---

## Part 1: VSLAM Fundamentals

### What is SLAM?

**SLAM (Simultaneous Localization and Mapping)** solves two interdependent problems:

1. **Localization**: "Where am I?" - Estimate robot pose (position + orientation)
2. **Mapping**: "What does the environment look like?" - Build a map of surroundings

**The Challenge**: You need a map to localize, but you need to know your location to build a map. SLAM solves both simultaneously.

### Visual SLAM vs. LiDAR SLAM

| Aspect | Visual SLAM | LiDAR SLAM |
|--------|-------------|------------|
| **Sensor** | Camera (RGB or stereo) | Laser rangefinder |
| **Data Type** | Images | Point clouds |
| **Cost** | Low ($50-200 for camera) | High ($500-$10,000 for LiDAR) |
| **Indoor Performance** | Excellent (texture-rich) | Good (geometry-based) |
| **Outdoor Performance** | Good (depends on lighting) | Excellent (sunlight invariant) |
| **Compute** | High (image processing) | Medium |

**For this course**: We focus on **visual SLAM** due to lower cost and Isaac ROS GPU acceleration.

### How Visual SLAM Works

**Pipeline**:

```
Camera Image → Feature Detection → Feature Tracking → Pose Estimation → Map Update → Loop Closure
     │              │                    │                  │               │             │
   640x480        Keypoints          Match keypoints     Calculate       Add new      Detect
    RGB          (ORB, SIFT)         across frames       camera pose    landmarks   revisited areas
```

**Key Concepts**:

1. **Features/Keypoints**: Distinctive image points (corners, edges) that can be reliably detected
2. **Descriptors**: Mathematical representations of features for matching across frames
3. **Pose Estimation**: Calculate camera movement from matched features (using geometry)
4. **Bundle Adjustment**: Optimize camera poses and 3D landmark positions jointly
5. **Loop Closure**: Detect when robot returns to a previously visited location (corrects drift)

---

## Part 2: Isaac ROS VSLAM Setup

Isaac ROS provides GPU-accelerated VSLAM optimized for NVIDIA hardware.

### Step 1: Install Isaac ROS VSLAM

```bash
# Navigate to ROS 2 workspace
cd ~/ros2_ws/src

# Clone Isaac ROS VSLAM repository
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select isaac_ros_visual_slam

# Source workspace
source install/setup.bash
```

**Installation Time**: 5-10 minutes

### Step 2: Verify Installation

```bash
# Check if VSLAM node is available
ros2 pkg list | grep isaac_ros_visual_slam
```

**Expected Output**:
```
isaac_ros_visual_slam
```

---

## Part 3: Running VSLAM in Isaac Sim

### Step 1: Create a Test Scene in Isaac Sim

We need a textured environment for VSLAM (features need visual variation):

1. Launch Isaac Sim
2. Load a pre-built environment:
   - **File** > **Open**
   - Navigate to: `omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Simple_Warehouse/warehouse.usd`

This warehouse has good visual texture (shelves, boxes, markings) for VSLAM.

### Step 2: Add a Robot with Stereo Camera

We'll use the Carter robot (NVIDIA's reference mobile robot):

1. **Isaac Utils** > **Assets** > **Robots** > **Carter**
2. Drag Carter into the scene at position `(0, 0, 0)`

Carter comes pre-configured with:
- Stereo camera pair (left/right)
- IMU
- Wheel odometry

### Step 3: Configure ROS 2 Bridge

To get camera data from Isaac Sim into ROS 2:

1. **Isaac Utils** > **ROS 2 Bridge**
2. Enable the following topics:
   - `/front_stereo_camera/left/image_raw` (left camera RGB)
   - `/front_stereo_camera/right/image_raw` (right camera RGB)
   - `/front_stereo_camera/left/camera_info` (camera calibration)
   - `/front_stereo_camera/right/camera_info`

3. Click **Enable ROS 2 Bridge**

### Step 4: Launch Isaac ROS VSLAM Node

In a terminal:

```bash
# Source ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Launch VSLAM node
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

**Expected Output**:
```
[isaac_ros_visual_slam]: Visual SLAM node started
[isaac_ros_visual_slam]: Waiting for stereo camera images...
[isaac_ros_visual_slam]: Received first image pair
[isaac_ros_visual_slam]: Initialization complete
```

### Step 5: Drive the Robot in Isaac Sim

VSLAM needs camera motion to build a map. Move Carter around:

**Option 1: Keyboard Teleop**

```bash
# In a new terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

Use keyboard controls:
- `i` = Forward
- `,` = Backward
- `j` = Turn left
- `l` = Turn right

**Option 2: Scripted Path (Python)**

```python
import rclpy
from geometry_msgs.msg import Twist

# Initialize ROS 2 node
rclpy.init()
node = rclpy.create_node('vslam_test_driver')
pub = node.create_publisher(Twist, '/cmd_vel', 10)

# Drive in a square pattern
commands = [
    (0.5, 0.0, 3.0),  # Forward 3 seconds
    (0.0, 0.5, 3.14), # Turn 90° (3.14 seconds at 0.5 rad/s)
    (0.5, 0.0, 3.0),  # Forward
    (0.0, 0.5, 3.14), # Turn 90°
    # ... repeat
]

for linear, angular, duration in commands:
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    pub.publish(msg)
    time.sleep(duration)
```

---

## Part 4: Visualizing VSLAM in RViz

### Step 1: Launch RViz

```bash
rviz2
```

### Step 2: Add VSLAM Visualizations

1. **Add** > **TF** (coordinate frame transformations)
   - Shows robot pose and camera frames

2. **Add** > **PointCloud2**
   - Topic: `/visual_slam/tracking/slam_map`
   - This displays the 3D map (sparse point cloud of features)

3. **Add** > **Path**
   - Topic: `/visual_slam/tracking/odometry`
   - Shows robot trajectory (where it has traveled)

4. **Add** > **Image**
   - Topic: `/front_stereo_camera/left/image_raw`
   - Shows live camera feed

5. Set **Fixed Frame**: `map`

### Step 3: Observe VSLAM Performance

As the robot moves, you should see:

- **Point cloud** growing (map being built)
- **Path** extending (robot trajectory)
- **TF frames** updating (robot pose changing)

**Healthy VSLAM Indicators**:
- Smooth trajectory (no sudden jumps)
- Dense point cloud in explored areas
- Consistent pose updates (30 Hz)

---

## Part 5: Understanding VSLAM Output

### Key ROS 2 Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | Robot pose estimate (position + orientation) |
| `/visual_slam/tracking/slam_map` | `sensor_msgs/PointCloud2` | 3D map (feature landmarks) |
| `/visual_slam/tracking/vo_pose` | `geometry_msgs/PoseStamped` | Visual odometry (without loop closure) |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | SLAM state (tracking/lost) |

### Inspecting Odometry

```bash
# View robot pose in real-time
ros2 topic echo /visual_slam/tracking/odometry
```

**Output Example**:
```yaml
pose:
  position:
    x: 2.34
    y: -1.56
    z: 0.12
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707
    w: 0.707  # Facing 90° (heading east)
```

### Inspecting Map Quality

```bash
# Count number of landmarks in map
ros2 topic echo /visual_slam/tracking/slam_map --field data | wc -l
```

**Good map**: 500+ landmarks for small room, 2000+ for warehouse

---

## Part 6: Evaluating SLAM Performance

### Metric 1: Trajectory Accuracy (Drift)

**Test**: Drive robot in a square loop back to start position. Measure error.

```python
import rclpy
from nav_msgs.msg import Odometry
import math

class DriftEvaluator:
    def __init__(self):
        self.start_pose = None
        self.current_pose = None

        self.subscription = node.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        if self.start_pose is None:
            self.start_pose = msg.pose.pose
        self.current_pose = msg.pose.pose

    def calculate_drift(self):
        dx = self.current_pose.position.x - self.start_pose.position.x
        dy = self.current_pose.position.y - self.start_pose.position.y
        drift = math.sqrt(dx**2 + dy**2)
        return drift

# After loop closure
evaluator = DriftEvaluator()
# ... drive loop ...
print(f"Drift: {evaluator.calculate_drift():.3f} meters")
```

**Acceptable Drift**:
- &lt;5% of total distance traveled (e.g., &lt;0.5m drift on 10m loop)

### Metric 2: Loop Closure Detection

**Test**: Return to a previously visited location. VSLAM should recognize it and correct drift.

**Check** `/visual_slam/status` topic:

```bash
ros2 topic echo /visual_slam/status
```

Look for `loop_closure_count` field incrementing when revisiting areas.

### Metric 3: Tracking Loss

**Test**: Cover camera or enter a textureless area (e.g., blank wall).

**Check** `/visual_slam/status`:

```yaml
tracking_status: 2  # 0=TRACKING_LOST, 1=TRACKING_UNCERTAIN, 2=TRACKING_GOOD
```

**Good VSLAM**: Recovers quickly (&lt;2 seconds) when returning to textured area.

---

## Part 7: Advanced VSLAM Configuration

### Tuning Parameters

Edit Isaac ROS VSLAM configuration file:

```bash
nano ~/ros2_ws/src/isaac_ros_visual_slam/config/visual_slam.yaml
```

**Key Parameters**:

```yaml
visual_slam:
  ros__parameters:
    # Feature detection
    num_features_per_frame: 1000  # More features = better but slower

    # Tracking
    min_num_features: 50  # Minimum to maintain tracking
    max_feature_age: 25   # How long to track a feature

    # Mapping
    enable_loop_closure: true  # Detect revisited locations
    loop_closure_frequency: 1.0  # Check every 1 second

    # Performance
    enable_imu_fusion: true  # Use IMU for better pose estimation
    enable_gpu_acceleration: true  # Use CUDA
```

### Using IMU Fusion

If your robot has an IMU (Carter does), enable fusion for better performance:

```yaml
enable_imu_fusion: true
imu_topic: '/imu/data'
```

**Benefits**:
- Reduces drift during fast motions
- Handles motion blur better
- Improves loop closure detection

---

## Part 8: Exporting Maps for Navigation

Once VSLAM builds a good map, save it for autonomous navigation.

### Step 1: Save Map

```bash
# Save point cloud map
ros2 service call /visual_slam/save_map isaac_ros_visual_slam_interfaces/srv/SaveMap "{map_url: '/home/user/maps/warehouse_map.pcd'}"
```

### Step 2: Convert to Occupancy Grid (for Nav2)

Isaac ROS VSLAM produces a **point cloud** (sparse 3D landmarks). Nav2 needs an **occupancy grid** (2D grid of free/occupied cells).

**Conversion**:

```bash
# Install octomap tools
sudo apt install ros-humble-octomap-server

# Convert point cloud to occupancy grid
ros2 run octomap_server octomap_saver_node -f /home/user/maps/warehouse_map.bt
```

This creates a 2D occupancy grid `.pgm` file and `.yaml` metadata.

### Step 3: Use Map with Nav2

```bash
# Launch Nav2 with saved map
ros2 launch nav2_bringup bringup_launch.py map:=/home/user/maps/warehouse_map.yaml
```

Now the robot can navigate autonomously using the VSLAM-generated map!

---

## Part 9: Common VSLAM Issues & Solutions

### Issue 1: VSLAM Fails to Initialize

**Symptom**: No map points appear, tracking status = LOST

**Causes & Solutions**:

1. **Insufficient texture**: Environment too uniform (blank walls)
   - **Solution**: Add visual markers (posters, tape, objects with patterns)

2. **Camera not moving**: VSLAM needs parallax to triangulate depth
   - **Solution**: Move robot (or camera) to generate motion

3. **Camera calibration incorrect**: Wrong intrinsic parameters
   - **Solution**: Re-calibrate camera using ROS 2 `camera_calibration` package

### Issue 2: High Drift (Map Warps Over Time)

**Symptom**: Robot returns to start but map shows it 1-2 meters off

**Causes & Solutions**:

1. **No loop closures**: Robot doesn't revisit areas
   - **Solution**: Drive overlapping paths, ensure `enable_loop_closure: true`

2. **Poor feature quality**: Too few stable features
   - **Solution**: Increase lighting, avoid reflective surfaces

3. **Fast motion**: Camera moves too quickly for feature tracking
   - **Solution**: Reduce robot speed to &lt;0.5 m/s during mapping

### Issue 3: Tracking Lost in Textureless Areas

**Symptom**: Tracking status drops to LOST near blank walls or uniform floors

**Solution**:
- Add **fiducial markers** (AprilTags) in sparse areas
- Fuse with **wheel odometry** to maintain pose during tracking loss

```yaml
enable_imu_fusion: true
enable_wheel_odometry_fusion: true
wheel_odom_topic: '/odom'
```

---

## Part 10: Hands-On Exercise

### Exercise: Map a Multi-Room Environment

**Objective**: Use VSLAM to create a complete map of a simulated office environment

#### Requirements

1. **Scene Setup**:
   - Load: `NVIDIA/Assets/Isaac/Environments/Office/office.usd` in Isaac Sim
   - Add Carter robot at entrance

2. **VSLAM Configuration**:
   - Enable loop closure
   - Enable IMU fusion
   - Set `num_features_per_frame: 1200`

3. **Mapping Task**:
   - Drive robot through all rooms
   - Return to start location (trigger loop closure)
   - Map should cover 100+ square meters

4. **Validation**:
   - Drift at loop closure < 0.5 meters
   - Map has 3000+ landmarks
   - No tracking loss events (or recovers within 2 seconds)

#### Deliverables

- Saved map file: `office_vslam_map.pcd`
- RViz screenshot showing:
  - Complete trajectory path
  - Point cloud map
  - Robot at final position
- Performance report:
  - Total distance traveled
  - Final drift error
  - Number of loop closures detected

**Estimated Time**: 1-2 hours

---

## Summary

In this lesson, you learned:

✅ Visual SLAM principles (feature tracking, pose estimation, mapping)
✅ Installing and configuring Isaac ROS VSLAM
✅ Running VSLAM in Isaac Sim with simulated cameras
✅ Visualizing maps and trajectories in RViz
✅ Evaluating SLAM performance (drift, loop closure)
✅ Exporting maps for autonomous navigation
✅ Troubleshooting common VSLAM issues

**Key Takeaways**:
- **Visual SLAM** enables map-free autonomous navigation using only cameras
- **Loop closure** is critical for correcting accumulated drift
- **Texture-rich environments** perform best for visual SLAM
- **Isaac ROS** provides GPU-accelerated VSLAM optimized for NVIDIA hardware

---

## Next Steps

In the next lesson (**Object Detection**), you'll learn:

- Deep neural network inference for object detection
- Running YOLO and Faster R-CNN in Isaac ROS
- Real-time detection on camera streams
- Integrating detection with robot actions (navigate to object, grasp, etc.)

---

## Additional Resources

- **ORB-SLAM3 Paper**: https://arxiv.org/abs/2007.11898 (state-of-the-art VSLAM algorithm)
- **Isaac ROS VSLAM Documentation**: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html
- **RTAB-Map**: http://introlab.github.io/rtabmap/ (alternative VSLAM system)
- **Visual SLAM Tutorial**: https://www.doc.ic.ac.uk/~ajd/Publications/cadena_etal_arxiv16.pdf
- **SLAM Benchmarking**: https://github.com/MichaelGrupp/evo (evaluate trajectory accuracy)

:::tip Real-World Application
Tesla's Autopilot uses a form of visual SLAM (called "Visual Odometry") to track the car's position when GPS is unavailable (tunnels, parking garages). The same techniques work for humanoid robots!
:::
