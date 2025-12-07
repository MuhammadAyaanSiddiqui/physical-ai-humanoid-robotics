# Object Detection with DNN Inference

## Overview

**Object detection** enables robots to identify and locate objects in camera images in real-time. Using deep neural networks (DNNs) like YOLO and Faster R-CNN, robots can detect multiple objects per frame with bounding boxes and class labels, enabling tasks like "pick up the red apple" or "navigate around people."

**What You'll Learn**:
- Understand object detection architectures (YOLO, Faster R-CNN)
- Run pre-trained models on Isaac ROS with GPU acceleration
- Train custom object detectors on synthetic data
- Integrate detection with robot control (grasp detected objects)
- Optimize inference for real-time performance (30+ FPS)

**Prerequisites**:
- Completed: VSLAM lesson (Chapter 8, previous)
- Synthetic Data Generation (Chapter 7)
- Python and basic deep learning concepts

**Estimated Time**: 3-4 hours

---

## Learning Objectives

By the end of this lesson, you will be able to:

1. Explain how YOLO and Faster R-CNN architectures work
2. Install and configure Isaac ROS DNN Inference nodes
3. Run object detection on live camera streams from Isaac Sim
4. Visualize detection results (bounding boxes, labels, confidence scores)
5. Train a custom detector on synthetic data from Chapter 7
6. Achieve real-time inference (30+ FPS) with GPU acceleration

---

## Part 1: Object Detection Fundamentals

### What is Object Detection?

**Object Detection** = **Classification** (what is it?) + **Localization** (where is it?)

**Input**: RGB image (e.g., 1280x720)
**Output**: List of detected objects, each with:
- **Bounding box**: `(x_min, y_min, x_max, y_max)` pixel coordinates
- **Class label**: e.g., "person", "box", "robot"
- **Confidence score**: 0-1 (model certainty)

**Example Output**:
```
[
  {"class": "apple", "bbox": [320, 180, 420, 280], "confidence": 0.95},
  {"class": "mug", "bbox": [500, 200, 600, 350], "confidence": 0.87}
]
```

### Popular Detection Architectures

| Architecture | Speed (FPS) | Accuracy (mAP) | Best For |
|--------------|-------------|----------------|----------|
| **YOLO v8** | 50-120 FPS | 50-55% | Real-time robotics, mobile platforms |
| **Faster R-CNN** | 5-15 FPS | 60-70% | High accuracy when speed is not critical |
| **SSD** | 20-40 FPS | 45-50% | Balanced speed/accuracy |
| **EfficientDet** | 15-30 FPS | 55-60% | Efficient for edge devices (Jetson) |

**For this course**: We focus on **YOLO v8** (fastest, good enough accuracy for robotics)

### How YOLO Works (Simplified)

**YOLO (You Only Look Once)** is a single-stage detector:

```
Input Image (640x640)
    ↓
Backbone Network (CSPDarknet)  ← Extract features
    ↓
Neck (PANet)  ← Fuse multi-scale features
    ↓
Head (Detection Layers)  ← Predict boxes + classes
    ↓
[Bounding Boxes + Class Probabilities]
    ↓
Non-Maximum Suppression (NMS)  ← Remove duplicate detections
    ↓
Final Detections
```

**Key Insight**: YOLO divides the image into a grid and predicts boxes directly, making it much faster than two-stage detectors (Faster R-CNN).

---

## Part 2: Isaac ROS DNN Inference Setup

Isaac ROS provides GPU-accelerated inference using NVIDIA TensorRT.

### Step 1: Install Isaac ROS DNN Inference

```bash
# Navigate to workspace
cd ~/ros2_ws/src

# Clone Isaac ROS DNN Inference
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git

# Clone Isaac ROS Object Detection
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select isaac_ros_dnn_inference isaac_ros_yolov8

# Source
source install/setup.bash
```

### Step 2: Download Pre-trained YOLO v8 Model

```bash
# Create model directory
mkdir -p ~/models/yolov8

# Download YOLOv8 nano (smallest, fastest)
cd ~/models/yolov8
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# Convert to TensorRT engine (GPU optimized)
python3 -c "
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.export(format='engine', device=0, half=True)  # FP16 precision
"
```

**Model Variants**:
- `yolov8n.pt` - Nano (fastest, 6MB)
- `yolov8s.pt` - Small (22MB)
- `yolov8m.pt` - Medium (50MB)
- `yolov8l.pt` - Large (100MB)

For **real-time robotics**, use Nano or Small.

---

## Part 3: Running Object Detection in Isaac Sim

### Step 1: Setup Scene with Objects

1. Launch Isaac Sim
2. Load warehouse scene: `NVIDIA/Assets/Isaac/Environments/Simple_Warehouse/warehouse.usd`
3. Add Carter robot with camera (from VSLAM lesson)
4. Place various objects in view:
   - Boxes (class: "box")
   - Person model (class: "person")
   - Chair (class: "chair")

### Step 2: Enable ROS 2 Camera Bridge

1. **Isaac Utils** > **ROS 2 Bridge**
2. Enable topic: `/front_camera/image_raw` (RGB camera)
3. Enable topic: `/front_camera/camera_info` (calibration)

### Step 3: Launch Isaac ROS Object Detection

```bash
# Terminal 1: Launch detection node
ros2 launch isaac_ros_yolov8 isaac_ros_yolov8.launch.py \
  model_file_path:=~/models/yolov8/yolov8n.engine \
  input_image_topic:=/front_camera/image_raw \
  output_topic:=/detections
```

**Expected Output**:
```
[isaac_ros_yolov8]: Loading model from ~/models/yolov8/yolov8n.engine
[isaac_ros_yolov8]: TensorRT engine loaded successfully
[isaac_ros_yolov8]: Inference node ready, waiting for images...
[isaac_ros_yolov8]: Processing images at 78.3 FPS
```

### Step 4: Visualize Detections in RViz

```bash
# Terminal 2: Launch RViz
rviz2
```

**Add visualizations**:

1. **Add** > **Image**
   - Topic: `/front_camera/image_raw` (raw camera feed)

2. **Add** > **Detection2DArray** (custom RViz plugin for bounding boxes)
   - Topic: `/detections`

You should see bounding boxes drawn on the camera image with class labels.

### Step 5: Inspect Detection Messages

```bash
# Terminal 3: Echo detections
ros2 topic echo /detections
```

**Output Example**:
```yaml
detections:
  - results:
      - id: 0
        score: 0.94
        class_id: 39  # "box" in COCO dataset
    bbox:
      center:
        x: 370.5
        y: 230.0
      size_x: 141.0
      size_y: 100.0
```

---

## Part 4: Understanding Detection Output

### Detection Message Structure

Isaac ROS uses `vision_msgs/Detection2DArray`:

```
Detection2DArray
├── header (timestamp, frame_id)
└── detections[] (list of detected objects)
    ├── results[]
    │   ├── id (class ID, e.g., 0=person, 39=box)
    │   └── score (confidence, 0-1)
    └── bbox (bounding box)
        ├── center (x, y in pixels)
        └── size (width, height in pixels)
```

### Filtering by Confidence Threshold

Ignore low-confidence detections:

```python
import rclpy
from vision_msgs.msg import Detection2DArray

class DetectionFilter:
    def __init__(self):
        self.node = rclpy.create_node('detection_filter')
        self.sub = self.node.create_subscription(
            Detection2DArray,
            '/detections',
            self.callback,
            10
        )
        self.pub = self.node.create_publisher(
            Detection2DArray,
            '/detections_filtered',
            10
        )

    def callback(self, msg):
        filtered = Detection2DArray()
        filtered.header = msg.header

        for det in msg.detections:
            if det.results[0].score > 0.7:  # 70% confidence threshold
                filtered.detections.append(det)

        self.pub.publish(filtered)
        print(f"Filtered: {len(msg.detections)} → {len(filtered.detections)} detections")

# Usage
rclpy.init()
filter_node = DetectionFilter()
rclpy.spin(filter_node.node)
```

---

## Part 5: Training a Custom Object Detector

Pre-trained models (like YOLO v8 on COCO dataset) detect 80 common classes. For custom objects (e.g., specific tools, parts, robots), train your own detector.

### Step 1: Prepare Synthetic Dataset

Use the dataset generated in Chapter 7 (Synthetic Data Generation):

```
~/synthetic_data/tabletop_dataset/
├── images/
│   ├── rgb_0000.png
│   ├── rgb_0001.png
│   └── ...
├── labels/
│   ├── 0000.txt  # YOLO format labels
│   ├── 0001.txt
│   └── ...
└── dataset.yaml
```

**YOLO Label Format** (per line in `.txt` file):
```
<class_id> <x_center> <y_center> <width> <height>
```

All values normalized to 0-1 range.

**Example** (`0000.txt`):
```
0 0.45 0.35 0.12 0.08  # Apple at center (0.45, 0.35), size 12% x 8%
1 0.67 0.42 0.10 0.15  # Mug at (0.67, 0.42)
```

### Step 2: Convert COCO Annotations to YOLO Format

If you generated data in COCO format (from Chapter 7), convert it:

```python
import json
from pathlib import Path

# Load COCO annotations
with open('annotations.json') as f:
    coco = json.load(f)

# Create class mapping
class_map = {cat['id']: cat['name'] for cat in coco['categories']}

# Convert each annotation
for ann in coco['annotations']:
    image_id = ann['image_id']
    class_id = ann['category_id']
    bbox = ann['bbox']  # [x, y, width, height] in pixels

    # Get image dimensions
    img_info = next(img for img in coco['images'] if img['id'] == image_id)
    img_w, img_h = img_info['width'], img_info['height']

    # Normalize to 0-1
    x_center = (bbox[0] + bbox[2] / 2) / img_w
    y_center = (bbox[1] + bbox[3] / 2) / img_h
    width = bbox[2] / img_w
    height = bbox[3] / img_h

    # Write YOLO label file
    label_file = f"labels/{image_id:04d}.txt"
    with open(label_file, 'a') as f:
        f.write(f"{class_id} {x_center} {y_center} {width} {height}\n")
```

### Step 3: Create Dataset Configuration

`dataset.yaml`:

```yaml
# Dataset paths
train: ~/synthetic_data/tabletop_dataset/train/images
val: ~/synthetic_data/tabletop_dataset/val/images

# Class names
nc: 6  # Number of classes
names: ['apple', 'orange', 'mug', 'plate', 'spoon', 'bowl']
```

### Step 4: Train YOLOv8

```python
from ultralytics import YOLO

# Load pre-trained model (transfer learning)
model = YOLO('yolov8n.pt')

# Train on custom dataset
results = model.train(
    data='dataset.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    device=0,  # GPU 0
    project='tabletop_detector',
    name='yolov8_custom'
)

# Export trained model to TensorRT
model.export(format='engine', device=0, half=True)
```

**Training Time**: 1-2 hours for 100 epochs on 2000 images (RTX 4070 Ti)

### Step 5: Deploy Custom Model

```bash
# Use your custom model instead of pre-trained
ros2 launch isaac_ros_yolov8 isaac_ros_yolov8.launch.py \
  model_file_path:=~/tabletop_detector/yolov8_custom/weights/best.engine \
  input_image_topic:=/front_camera/image_raw \
  output_topic:=/custom_detections
```

---

## Part 6: Integrating Detection with Robot Actions

### Use Case: Navigate to Detected Object

**Task**: Robot detects a "box" and drives toward it.

**Python Script** (`drive_to_detected_box.py`):

```python
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
import math

class DriveToObject(Node):
    def __init__(self):
        super().__init__('drive_to_object')

        # Subscribe to detections
        self.sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        # Publish velocity commands
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.target_class = "box"  # Change to your target

    def detection_callback(self, msg):
        # Find target object
        target_detection = None
        for det in msg.detections:
            class_id = det.results[0].id
            class_name = self.get_class_name(class_id)  # Map ID to name

            if class_name == self.target_class and det.results[0].score > 0.7:
                target_detection = det
                break

        if target_detection is None:
            self.get_logger().info(f"No {self.target_class} detected")
            return

        # Calculate error from image center
        bbox_center_x = target_detection.bbox.center.x
        image_width = 1280  # Camera resolution

        error = bbox_center_x - (image_width / 2)  # Pixels from center

        # Proportional controller
        Kp = 0.002  # Tuning parameter
        angular_velocity = -Kp * error  # Negative to turn toward target

        # Drive forward while correcting angle
        cmd = Twist()
        cmd.linear.x = 0.3  # Forward speed (m/s)
        cmd.angular.z = angular_velocity  # Turn to center object

        self.pub.publish(cmd)
        self.get_logger().info(f"Driving toward {self.target_class}, error: {error:.1f}px")

    def get_class_name(self, class_id):
        # COCO class mapping (simplified)
        coco_classes = {0: "person", 39: "box", 41: "cup", ...}
        return coco_classes.get(class_id, "unknown")

# Run node
rclpy.init()
node = DriveToObject()
rclpy.spin(node)
```

**Result**: Robot turns to center the "box" in view and drives toward it.

### Use Case: Grasp Detected Object

**Task**: Robot arm reaches toward detected object.

**Requirements**:
- 3D position (from depth camera + detection)
- Inverse kinematics (calculate joint angles to reach position)

(We'll cover this in detail in Module 4: Humanoid Control)

---

## Part 7: Performance Optimization

### Metric 1: Inference FPS

**Goal**: 30+ FPS for real-time robotic control

**Check FPS**:

```bash
ros2 topic hz /detections
```

**Expected Output**:
```
average rate: 78.3
```

**If FPS is low (&lt;20)**:

1. **Use smaller model**:
   - `yolov8n` instead of `yolov8l`

2. **Reduce input resolution**:
   - Train/run at 416x416 instead of 640x640

3. **Use FP16 precision**:
   - Already enabled with `half=True` in TensorRT export

4. **Batch inference** (if processing multiple cameras):
   - Combine images into batches for GPU efficiency

### Metric 2: Detection Accuracy (mAP)

**Mean Average Precision (mAP)** measures detection quality.

**Evaluate on test set**:

```python
from ultralytics import YOLO

model = YOLO('~/models/yolov8/best.engine')
metrics = model.val(data='dataset.yaml')

print(f"mAP@0.5: {metrics.box.map50:.3f}")  # Typical: 0.70-0.90 for good models
print(f"mAP@0.5:0.95: {metrics.box.map:.3f}")  # Stricter: 0.50-0.70
```

**If accuracy is low (&lt;0.5 mAP)**:

1. **More training data**: Generate 5000+ images with domain randomization
2. **More training epochs**: Train for 200-300 epochs
3. **Data augmentation**: Flips, rotations, color jitter (YOLO does this automatically)
4. **Check label quality**: Manually review 50 random annotations for errors

---

## Part 8: Troubleshooting Common Issues

### Issue 1: No Detections Output

**Symptom**: `/detections` topic has no messages

**Solutions**:

1. **Check image topic**:
   ```bash
   ros2 topic list | grep image
   ros2 topic hz /front_camera/image_raw  # Should be 30+ Hz
   ```

2. **Verify model loaded**:
   - Check terminal output for "TensorRT engine loaded successfully"

3. **Check topic remapping**:
   - Ensure `input_image_topic` matches your camera topic

### Issue 2: Low Confidence Scores

**Symptom**: All detections have score < 0.3

**Causes**:

1. **Domain gap**: Model trained on real images, testing on simulation
   - **Solution**: Fine-tune on synthetic data (100-500 images)

2. **Wrong class labels**: Model expects different classes
   - **Solution**: Verify class IDs match between model and scene

3. **Poor image quality**: Dark, blurry, or low resolution
   - **Solution**: Improve lighting in Isaac Sim, increase camera resolution

### Issue 3: Duplicate Detections

**Symptom**: Multiple boxes for the same object

**Cause**: Non-Maximum Suppression (NMS) threshold too high

**Solution**: Lower NMS threshold in model config:

```python
model = YOLO('yolov8n.pt')
model.export(format='engine', device=0, half=True, nms_threshold=0.3)  # Default: 0.45
```

---

## Part 9: Hands-On Exercise

### Exercise: Build a "Fruit Picker" Robot

**Objective**: Detect and navigate to fruits (apples, oranges) on a tabletop.

#### Requirements

1. **Scene Setup**:
   - Tabletop with 10 fruits (5 apples, 5 oranges)
   - Carter robot with camera facing tabletop

2. **Object Detection**:
   - Train custom YOLOv8 model on synthetic fruit dataset
   - Classes: ["apple", "orange"]
   - Inference at 30+ FPS

3. **Robot Control**:
   - Implement `drive_to_fruit.py` script
   - Robot detects nearest "apple"
   - Drives toward apple until distance < 0.5 meters

4. **Validation**:
   - Detection accuracy: mAP > 0.80
   - Robot successfully navigates to 8/10 apples

#### Deliverables

- Trained model: `fruit_detector_yolov8.engine`
- Control script: `drive_to_fruit.py`
- Video recording showing:
  - Live detection (bounding boxes on fruits)
  - Robot driving toward apple
  - Successful approach (&lt;0.5m from target)

**Estimated Time**: 2-3 hours

---

## Summary

In this lesson, you learned:

✅ Object detection fundamentals (YOLO architecture)
✅ Installing and running Isaac ROS DNN Inference
✅ Real-time object detection on camera streams (30+ FPS)
✅ Training custom detectors on synthetic data
✅ Integrating detection with robot control (navigation, grasping)
✅ Performance optimization (FPS, accuracy tuning)

**Key Takeaways**:
- **YOLO** provides real-time detection suitable for robotics
- **Isaac ROS** enables GPU-accelerated inference with TensorRT
- **Synthetic data** from Chapter 7 can train production-quality detectors
- **Detection → Action** integration is straightforward with ROS 2 topics

---

## Next Steps

In the next lesson (**Depth Estimation**), you'll learn:

- Stereo depth estimation from camera pairs
- Monocular depth estimation with deep learning
- 3D point cloud generation from RGB-D data
- Combining detection + depth for 3D object localization

---

## Additional Resources

- **YOLOv8 Documentation**: https://docs.ultralytics.com/
- **Isaac ROS Object Detection**: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_object_detection/index.html
- **TensorRT Optimization Guide**: https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html
- **COCO Dataset**: https://cocodataset.org/ (80 common object classes)
- **Object Detection Papers**: https://paperswithcode.com/task/object-detection

:::tip Production Robotics
Amazon warehouse robots use object detection to identify packages, bins, and shelves for autonomous picking and sorting. The same techniques work for humanoid manipulation tasks!
:::
