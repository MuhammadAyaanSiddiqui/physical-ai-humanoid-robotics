# 6DOF Pose Estimation

## Overview

**6DOF (Six Degrees of Freedom) pose estimation** determines an object's complete position and orientation in 3D space. This is essential for robotic manipulation - to grasp an object, the robot needs to know not just *where* it is (3D position), but also *how it's oriented* (3D rotation). This enables precise pick-and-place, assembly, and manipulation tasks.

**What You'll Learn**:
- Understand 6DOF pose representation (position + rotation)
- Run pose estimation networks (DOPE, FoundationPose)
- Estimate poses from RGB-D data
- Generate grasp poses for manipulation
- Track object poses across frames

**Prerequisites**:
- Completed: Depth Estimation (previous lesson)
- Understanding of 3D transformations (rotation matrices, quaternions)
- Object detection and depth sensing from previous lessons

**Estimated Time**: 3-4 hours

---

## Learning Objectives

By the end of this lesson, you will be able to:

1. Represent 6DOF poses (translation vectors and rotation quaternions)
2. Run DOPE (Deep Object Pose Estimation) in Isaac ROS
3. Estimate object poses from RGB-D images
4. Generate viable grasp poses for robot manipulation
5. Track object poses over time for dynamic grasping
6. Integrate pose estimation with robot planning

---

## Part 1: 6DOF Pose Fundamentals

### What is 6DOF?

**6 Degrees of Freedom** = 3 position coordinates + 3 rotation angles

| DOF | Description | Range | Example |
|-----|-------------|-------|---------|
| **X** | Position along X-axis | ±∞ meters | 0.5m (right of camera) |
| **Y** | Position along Y-axis | ±∞ meters | -0.2m (below camera) |
| **Z** | Position along Z-axis | ±∞ meters | 1.2m (forward from camera) |
| **Roll** | Rotation around X-axis | 0-360° | 15° (tilted right) |
| **Pitch** | Rotation around Y-axis | 0-360° | -30° (tilted down) |
| **Yaw** | Rotation around Z-axis | 0-360° | 90° (facing right) |

**ROS 2 Representation** (`geometry_msgs/Pose`):

```yaml
position:
  x: 0.5
  y: -0.2
  z: 1.2
orientation:  # Quaternion (x, y, z, w)
  x: 0.130
  y: -0.259
  z: 0.389
  w: 0.909
```

:::tip Why Quaternions?
Quaternions avoid "gimbal lock" (mathematical singularities) that occur with Euler angles. They're the standard for 3D rotation in robotics.
:::

### Why Pose Estimation is Hard

**Challenges**:
1. **Ambiguity**: Symmetric objects (spheres, cylinders) have infinite valid orientations
2. **Occlusion**: Objects partially hidden behind others
3. **Lighting**: Shiny surfaces create reflections, textures change appearance
4. **Scale**: Similar-looking objects at different distances

**Solutions**:
- Use **depth data** (RGB-D) to resolve scale ambiguity
- Use **learning-based methods** (neural networks) to handle lighting/texture variation
- Use **tracking** to maintain pose across frames even with occlusion

---

## Part 2: DOPE (Deep Object Pose Estimation)

**DOPE** is NVIDIA's pose estimation network optimized for Isaac ROS.

### Step 1: Install Isaac ROS DOPE

```bash
cd ~/ros2_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select isaac_ros_dope
source install/setup.bash
```

### Step 2: Download Pre-trained DOPE Models

DOPE provides models trained on common objects (YCB dataset):

```bash
mkdir -p ~/models/dope
cd ~/models/dope

# Download cracker box model (example)
wget https://nvidia-isaac-ros.github.io/models/Cracker.pth

# Available models:
# - Cracker.pth (cracker box)
# - Soup.pth (soup can)
# - Mustard.pth (mustard bottle)
# - TomatoSoup.pth (tomato soup can)
```

### Step 3: Configure DOPE Parameters

Create `dope_config.yaml`:

```yaml
dope:
  ros__parameters:
    # Model configuration
    model_file_path: "/home/user/models/dope/Cracker.pth"
    object_name: "cracker_box"

    # Detection threshold
    detection_threshold: 0.5  # Confidence threshold

    # Camera intrinsics (from camera_info topic)
    camera_info_topic: "/front_camera/camera_info"

    # Input topics
    image_topic: "/front_camera/image_raw"

    # Output topics
    pose_topic: "/dope/pose"
    marker_topic: "/dope/markers"  # For RViz visualization
```

### Step 4: Launch DOPE Node

```bash
ros2 launch isaac_ros_dope isaac_ros_dope.launch.py config_file:=dope_config.yaml
```

**Expected Output**:
```
[isaac_ros_dope]: Model loaded: Cracker.pth
[isaac_ros_dope]: Listening for images on /front_camera/image_raw
[isaac_ros_dope]: Publishing poses to /dope/pose
```

### Step 5: Visualize Pose in RViz

```bash
rviz2
```

Add displays:
1. **Image** → `/front_camera/image_raw` (camera view)
2. **Marker** → `/dope/markers` (shows object pose as 3D axes)
3. **TF** → Shows coordinate frames

You should see:
- Camera image with detected object
- 3D coordinate axes overlaid on object (X=red, Y=green, Z=blue)

---

## Part 3: Understanding Pose Output

### DOPE Pose Message

Topic: `/dope/pose`
Type: `geometry_msgs/PoseStamped`

```yaml
header:
  stamp: {sec: 1234567890, nanosec: 123456789}
  frame_id: "camera_color_optical_frame"
pose:
  position:
    x: 0.45  # Meters from camera
    y: -0.12
    z: 0.82
  orientation:  # Quaternion
    x: 0.013
    y: 0.987
    z: -0.159
    w: 0.019
```

### Converting Quaternion to Euler Angles (for Debugging)

```python
import math

def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to roll, pitch, yaw (radians).
    """
    # Roll (X-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (Y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90° if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (Z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# Usage
roll, pitch, yaw = quaternion_to_euler(0.013, 0.987, -0.159, 0.019)
print(f"Roll: {math.degrees(roll):.1f}°")
print(f"Pitch: {math.degrees(pitch):.1f}°")
print(f"Yaw: {math.degrees(yaw):.1f}°")
```

---

## Part 4: Training Custom DOPE Models

For custom objects (e.g., tools, parts specific to your application), train DOPE on synthetic data.

### Step 1: Generate Synthetic Training Data

Use Isaac Sim Replicator (from Chapter 7):

```python
import omni.replicator.core as rep

# Load your custom object model
custom_object = rep.create.from_usd("path/to/your/object.usd")

# Randomize pose
with rep.trigger.on_frame(num_frames=5000):
    with custom_object:
        rep.modify.pose(
            position=rep.distribution.uniform((-0.3, -0.3, 0.5), (0.3, 0.3, 1.5)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )

# Capture RGB + ground truth pose annotations
camera = rep.create.camera(position=(-1.0, 0, 0.8), look_at=(0, 0, 0.8))
camera.add_annotator(rep.AnnotatorRegistry.get_annotator("rgb"))
camera.add_annotator(rep.AnnotatorRegistry.get_annotator("6dof_pose"))

# Output writer
rep.WriterRegistry.get("BasicWriter").initialize(output_dir="/data/custom_object_poses")
rep.orchestrator.run()
```

**Result**: 5000 images with ground truth 6DOF pose labels

### Step 2: Train DOPE Model

```bash
# Clone DOPE training repository
git clone https://github.com/NVlabs/Deep_Object_Pose.git
cd Deep_Object_Pose

# Prepare dataset (convert to DOPE format)
python scripts/prepare_dataset.py \
  --input /data/custom_object_poses \
  --output /data/dope_dataset

# Train model
python train.py \
  --data /data/dope_dataset \
  --object custom_tool \
  --epochs 60 \
  --outf /models/dope/custom_tool

# Convert trained model to TensorRT (for Isaac ROS)
python export_to_trt.py --model /models/dope/custom_tool/net_epoch_60.pth
```

**Training Time**: 2-4 hours on RTX 4070 Ti for 5000 images, 60 epochs

---

## Part 5: Grasp Pose Generation

Once you have an object's 6DOF pose, generate viable grasp poses for a robot gripper.

### Step 1: Define Gripper Parameters

```python
class GripperParams:
    def __init__(self):
        self.finger_length = 0.08  # 8cm gripper fingers
        self.max_opening = 0.10    # 10cm max grasp width
        self.approach_distance = 0.15  # 15cm pre-grasp distance
```

### Step 2: Compute Grasp Pose from Object Pose

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

def generate_grasp_pose(object_pose, gripper_params):
    """
    Generate grasp pose from detected object pose.

    Args:
        object_pose: geometry_msgs/Pose of detected object
        gripper_params: GripperParams instance

    Returns:
        geometry_msgs/Pose of grasp (gripper target pose)
    """
    # Extract object position and orientation
    obj_pos = np.array([object_pose.position.x,
                         object_pose.position.y,
                        object_pose.position.z])

    obj_quat = [object_pose.orientation.x,
                object_pose.orientation.y,
                object_pose.orientation.z,
                object_pose.orientation.w]

    obj_rot = R.from_quat(obj_quat)

    # Grasp from top (approach along Z-axis)
    approach_vector = np.array([0, 0, -1])  # Downward

    # Apply object orientation to approach vector
    rotated_approach = obj_rot.apply(approach_vector)

    # Pre-grasp position (offset by approach distance)
    pre_grasp_pos = obj_pos + rotated_approach * gripper_params.approach_distance

    # Gripper orientation: align gripper Z-axis with approach vector
    # (Implementation depends on gripper URDF frame definitions)
    grasp_orientation = obj_quat  # Simplified: use object orientation

    # Create grasp pose
    grasp_pose = Pose()
    grasp_pose.position.x = pre_grasp_pos[0]
    grasp_pose.position.y = pre_grasp_pos[1]
    grasp_pose.position.z = pre_grasp_pos[2]
    grasp_pose.orientation = object_pose.orientation

    return grasp_pose
```

### Step 3: Publish Grasp Pose for Motion Planning

```python
class GraspPlanner:
    def __init__(self):
        self.node = rclpy.create_node('grasp_planner')

        # Subscribe to object poses
        self.sub = self.node.create_subscription(
            PoseStamped,
            '/dope/pose',
            self.pose_callback,
            10
        )

        # Publish grasp poses
        self.pub = self.node.create_publisher(
            PoseStamped,
            '/grasp_target',
            10
        )

        self.gripper = GripperParams()

    def pose_callback(self, msg):
        # Generate grasp pose
        grasp_pose_msg = PoseStamped()
        grasp_pose_msg.header = msg.header
        grasp_pose_msg.pose = generate_grasp_pose(msg.pose, self.gripper)

        # Publish for motion planner
        self.pub.publish(grasp_pose_msg)
        self.node.get_logger().info(f"Generated grasp pose at {grasp_pose_msg.pose.position.z:.2f}m height")
```

---

## Part 6: Pose Tracking

For moving objects, track pose across frames for dynamic grasping.

### Using Kalman Filter for Pose Smoothing

```python
from filterpy.kalman import KalmanFilter

class PoseTracker:
    def __init__(self):
        self.kf = KalmanFilter(dim_x=7, dim_z=7)  # State: [x, y, z, qx, qy, qz, qw]

        # State transition matrix (constant position model)
        self.kf.F = np.eye(7)

        # Measurement function (direct observation)
        self.kf.H = np.eye(7)

        # Process and measurement noise (tune these)
        self.kf.Q *= 0.01  # Process noise
        self.kf.R *= 0.1   # Measurement noise

        # Initial state
        self.kf.x = np.zeros(7)
        self.kf.P *= 1000  # High initial uncertainty

    def update(self, pose):
        """Update filter with new pose measurement."""
        measurement = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])

        self.kf.predict()
        self.kf.update(measurement)

        # Return smoothed pose
        smoothed_pose = Pose()
        smoothed_pose.position.x = self.kf.x[0]
        smoothed_pose.position.y = self.kf.x[1]
        smoothed_pose.position.z = self.kf.x[2]
        smoothed_pose.orientation.x = self.kf.x[3]
        smoothed_pose.orientation.y = self.kf.x[4]
        smoothed_pose.orientation.z = self.kf.x[5]
        smoothed_pose.orientation.w = self.kf.x[6]

        return smoothed_pose
```

---

## Part 7: Hands-On Exercise

### Exercise: Pick-and-Place with 6DOF Pose Estimation

**Objective**: Estimate object poses and execute pick-and-place operations in simulation.

#### Requirements

1. **Scene Setup**:
   - Tabletop with 3 objects (e.g., cracker boxes)
   - Robot arm (Franka Emika) with gripper

2. **Pose Estimation**:
   - Run DOPE on camera stream
   - Detect and estimate 6DOF pose of each object
   - Visualize poses in RViz (3D coordinate axes)

3. **Grasp Planning**:
   - Generate grasp poses for detected objects
   - Compute pre-grasp and grasp waypoints

4. **Execution** (manual for now - full automation in Module 4):
   - Display grasp target pose in RViz
   - Verify pose is reachable by robot arm (within workspace)

#### Deliverables

- DOPE configuration file: `pick_place_dope.yaml`
- Python script: `grasp_pose_planner.py`
- RViz recording showing:
  - Object detection and pose estimation
  - Generated grasp pose visualization
  - Coordinate frames (camera, object, grasp)
- Accuracy test: Compare estimated poses to ground truth (error &lt;5cm position, &lt;10° orientation)

**Estimated Time**: 3 hours

---

## Summary

In this lesson, you learned:

✅ 6DOF pose representation (position + quaternion rotation)
✅ Running DOPE for object pose estimation
✅ Training custom DOPE models on synthetic data
✅ Generating grasp poses from object poses
✅ Pose tracking with Kalman filtering
✅ Integrating pose estimation with robot planning

**Key Takeaways**:
- **6DOF pose** is essential for manipulation (not just 3D position)
- **DOPE** provides real-time pose estimation optimized for Isaac ROS
- **Synthetic data** from Isaac Sim enables training on custom objects
- **Grasp planning** requires object pose + gripper constraints

**Congratulations!** You've completed **Chapter 8: Perception with Isaac ROS**! You can now:
- Build maps with VSLAM
- Detect objects with YOLO
- Estimate depth and generate 3D point clouds
- Estimate 6DOF poses for manipulation

---

## Next Steps

In **Chapter 9: Autonomous Navigation (Nav2)**, you'll learn:

- Map building with SLAM and map saving
- Path planning algorithms (A*, DWA, TEB)
- Obstacle avoidance with costmap layers
- Behavior trees for navigation logic

---

## Additional Resources

- **DOPE Paper**: https://arxiv.org/abs/1809.10790
- **Isaac ROS Pose Estimation**: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/index.html
- **Quaternion Math**: https://eater.net/quaternions (interactive visualization)
- **Grasp Planning**: https://manipulation.csail.mit.edu/ (MIT robotics course)
- **YCB Object Dataset**: https://www.ycbbenchmarks.com/ (standard objects for benchmarking)

:::success Module Milestone!
You've completed 8 perception lessons (VSLAM, object detection, depth, pose)! These are the foundational AI perception skills for autonomous robots. Next up: autonomous navigation!
:::
