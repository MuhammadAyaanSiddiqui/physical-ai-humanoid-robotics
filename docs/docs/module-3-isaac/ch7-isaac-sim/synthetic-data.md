# Synthetic Data Generation

## Overview

One of Isaac Sim's most powerful capabilities is **synthetic data generation** - the ability to automatically produce labeled training data for AI perception models. Instead of manually collecting and annotating thousands of real-world images, you can generate diverse, perfectly labeled datasets in simulation.

**What You'll Learn**:
- Extract RGB camera images, depth maps, and segmentation masks
- Programmatically place cameras for multi-view data collection
- Annotate objects automatically for object detection
- Export datasets in formats for AI training (COCO, KITTI, custom)
- Generate diverse training data at scale

**Prerequisites**:
- Completed: Installation & Photorealistic Environments
- Python programming basics
- Understanding of computer vision concepts (helpful but not required)

**Estimated Time**: 2-3 hours

---

## Learning Objectives

By the end of this lesson, you will be able to:

1. Capture RGB, depth, and semantic segmentation data from Isaac Sim cameras
2. Configure camera sensors programmatically via Python API
3. Annotate 3D bounding boxes and instance segmentation masks
4. Export datasets in standard formats (COCO JSON, KITTI)
5. Generate thousands of labeled images automatically
6. Validate synthetic data quality for AI training

---

## Part 1: Understanding Synthetic Data Types

Isaac Sim can generate multiple data modalities simultaneously from a single scene:

### Data Modality Overview

| Modality | Description | Use Case | File Format |
|----------|-------------|----------|-------------|
| **RGB** | Standard color camera images | Object recognition, scene understanding | .png, .jpg |
| **Depth** | Distance from camera to each pixel | 3D reconstruction, obstacle avoidance | .npy, .exr |
| **Semantic Segmentation** | Per-pixel class labels | Scene parsing, terrain classification | .png (color-coded) |
| **Instance Segmentation** | Per-pixel object instance IDs | Counting objects, tracking instances | .png (ID map) |
| **2D Bounding Boxes** | Rectangular object bounds in image | Object detection (YOLO, Faster R-CNN) | JSON, XML |
| **3D Bounding Boxes** | Oriented 3D boxes in world space | 3D object detection, LiDAR fusion | JSON |
| **Normals** | Surface normal vectors | 3D reconstruction, relighting | .exr |
| **Optical Flow** | Pixel motion between frames | Motion estimation, tracking | .exr |

:::info Why Synthetic Data?
- **Cost**: Free vs. $0.01-$0.10/image for manual annotation
- **Speed**: 1000s of images/hour vs. days for real collection
- **Diversity**: Unlimited scene variations vs. limited real scenarios
- **Perfection**: No annotation errors vs. 5-10% human error rate
:::

---

## Part 2: Setting Up a Camera Sensor

### Step 1: Create a Scene with Objects

Before generating data, we need a scene with labeled objects. Let's use the warehouse from the previous lesson or create a simple one:

1. Launch Isaac Sim
2. **File** > **Open** > Select `warehouse_pickplace_v1.usd` (from previous lesson)
   OR create a new scene with:
   - Ground plane
   - 5-10 props (boxes, bins, robots)
   - Lighting

### Step 2: Add a Camera via GUI

1. **Create** > **Camera**
2. In **Property** panel:
   - **Name**: `DataCollectionCamera`
   - **Position**: `X: -200, Y: 200, Z: 150` (angled view of scene)
   - **Rotation**: Point toward objects (use **Look At** tool)

3. Set camera parameters:
   - **Focal Length**: `24mm` (wide angle for full scene coverage)
   - **Horizontal Aperture**: `36mm` (sensor size)
   - **Resolution**: `1280x720` (720p for training data)

### Step 3: Enable Replicator (Isaac Sim's Data Generation Tool)

1. **Isaac Utils** > **Replicator** > **Enable Replicator**
2. A new **Replicator** panel will appear (bottom of screen)

---

## Part 3: Capturing Data via Python API

For scalable data generation, we use Isaac Sim's Python API. This allows automation and batch processing.

### Step 1: Create a Data Capture Script

Create a file: `~/Documents/Isaac-Sim/scripts/capture_synthetic_data.py`

```python
import omni.replicator.core as rep
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core import SimulationContext
import numpy as np

# Initialize Isaac Sim
simulation_context = SimulationContext(physics_dt=1.0/60.0, rendering_dt=1.0/60.0)
simulation_context.initialize_physics()

# Create camera
camera = rep.create.camera(
    position=(-200, 200, 150),
    look_at=(0, 0, 50),  # Point camera at scene center
    resolution=(1280, 720),
    name="synthetic_data_camera"
)

# Define output annotators (data types to capture)
rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
semantic_annotator = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
instance_annotator = rep.AnnotatorRegistry.get_annotator("instance_segmentation_fast")
bbox_2d_annotator = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")

# Attach annotators to camera
camera.add_annotator(rgb_annotator)
camera.add_annotator(depth_annotator)
camera.add_annotator(semantic_annotator)
camera.add_annotator(instance_annotator)
camera.add_annotator(bbox_2d_annotator)

# Set output directory
output_dir = "/home/user/synthetic_data/warehouse_dataset"
rep.WriterRegistry.register_writer("BasicWriter", output_dir=output_dir)

print("Camera and annotators configured successfully!")
```

### Step 2: Run the Script in Isaac Sim

1. In Isaac Sim: **Window** > **Script Editor**
2. Paste the script above
3. Click **Run**

You should see: `Camera and annotators configured successfully!`

### Step 3: Capture a Single Frame

Add to the script:

```python
# Capture one frame of data
simulation_context.step(render=True)  # Run physics for 1 frame

# Retrieve annotator outputs
rgb_data = rgb_annotator.get_data()
depth_data = depth_annotator.get_data()
semantic_data = semantic_annotator.get_data()
instance_data = instance_annotator.get_data()
bbox_data = bbox_2d_annotator.get_data()

# Save RGB image
import cv2
cv2.imwrite(f"{output_dir}/rgb_0000.png", rgb_data)
print(f"Saved RGB image to {output_dir}/rgb_0000.png")

# Save depth map (as numpy array)
np.save(f"{output_dir}/depth_0000.npy", depth_data)
print(f"Saved depth map")

# Semantic segmentation is saved automatically by Replicator
```

Run the updated script. Check `/home/user/synthetic_data/warehouse_dataset` for output files.

---

## Part 4: Understanding Depth and Segmentation Data

### Depth Maps

Depth data represents the distance from the camera to each pixel in **meters**.

**Visualizing Depth**:

```python
import numpy as np
import matplotlib.pyplot as plt

# Load depth map
depth = np.load("/home/user/synthetic_data/warehouse_dataset/depth_0000.npy")

# Visualize (closer = darker, farther = lighter)
plt.imshow(depth, cmap='gray')
plt.colorbar(label='Distance (meters)')
plt.title('Depth Map')
plt.show()
```

**Use Cases**:
- Obstacle avoidance (identify objects &lt;2m from robot)
- 3D reconstruction (convert depth to point cloud)
- Grasp planning (estimate object distance)

### Semantic Segmentation

Each pixel is labeled with its object class (e.g., "box", "robot", "floor").

**Visualizing Semantic Segmentation**:

Segmentation masks are saved as color-coded PNG images where each color represents a class:

```python
from PIL import Image

# Load semantic segmentation
seg_img = Image.open("/home/user/synthetic_data/warehouse_dataset/semantic_0000.png")
seg_img.show()

# Color mapping (example):
# Red (255, 0, 0) = Robot
# Green (0, 255, 0) = Box
# Blue (0, 0, 255) = Floor
```

**Use Cases**:
- Terrain classification (drivable vs. obstacle)
- Scene understanding (identify all objects in view)
- Training segmentation models (U-Net, DeepLab)

### Instance Segmentation

Similar to semantic, but each individual object gets a unique ID (e.g., "box_001", "box_002").

```python
# Load instance segmentation
instance_img = np.array(Image.open("/home/user/synthetic_data/warehouse_dataset/instance_0000.png"))

# Each unique pixel value is an instance ID
unique_instances = np.unique(instance_img)
print(f"Number of instances detected: {len(unique_instances)}")
```

**Use Cases**:
- Counting objects (e.g., "How many boxes on shelf?")
- Tracking (follow specific object across frames)
- Robotic grasping (identify individual target objects)

---

## Part 5: Annotating Objects for Detection

For object detection models (YOLO, Faster R-CNN), we need 2D bounding boxes with class labels.

### Step 1: Define Semantic Classes

Isaac Sim automatically assigns semantic labels based on USD prim names. To ensure correct labeling:

1. Select an object (e.g., a box) in the **Stage** panel
2. In **Property** panel, find **Semantics**
3. Add semantic label:
   - **Type**: `class`
   - **Data**: `box` (or your class name)

Repeat for all objects in your scene (e.g., `robot`, `bin`, `pallet`).

### Step 2: Extract 2D Bounding Boxes

```python
# Get bounding box data from annotator
bbox_data = bbox_2d_annotator.get_data()

# bbox_data structure:
# {
#   'data': [
#     {
#       'semantic_id': 'box',
#       'x_min': 320, 'y_min': 180,
#       'x_max': 480, 'y_max': 360
#     },
#     ...
#   ]
# }

# Convert to COCO format (for YOLO training)
coco_annotations = []
for i, bbox in enumerate(bbox_data['data']):
    coco_annotations.append({
        "id": i,
        "image_id": 0,  # Frame number
        "category_id": class_to_id[bbox['semantic_id']],  # Map 'box' -> 1, 'robot' -> 2, etc.
        "bbox": [bbox['x_min'], bbox['y_min'],
                 bbox['x_max'] - bbox['x_min'],  # width
                 bbox['y_max'] - bbox['y_min']],  # height
        "area": (bbox['x_max'] - bbox['x_min']) * (bbox['y_max'] - bbox['y_min']),
        "iscrowd": 0
    })

# Save to JSON
import json
with open(f"{output_dir}/annotations.json", 'w') as f:
    json.dump({"annotations": coco_annotations}, f, indent=2)
```

### Step 3: Visualize Bounding Boxes on Image

```python
import cv2

# Load RGB image
img = cv2.imread(f"{output_dir}/rgb_0000.png")

# Draw bounding boxes
for bbox in bbox_data['data']:
    cv2.rectangle(img,
                  (bbox['x_min'], bbox['y_min']),
                  (bbox['x_max'], bbox['y_max']),
                  (0, 255, 0), 2)  # Green box
    cv2.putText(img, bbox['semantic_id'],
                (bbox['x_min'], bbox['y_min'] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Save annotated image
cv2.imwrite(f"{output_dir}/rgb_0000_annotated.png", img)
```

---

## Part 6: Generating Large-Scale Datasets

To train robust AI models, we need thousands of diverse images. Isaac Sim's Replicator enables this through **randomization**.

### Step 1: Randomize Camera Viewpoints

```python
import random

# Generate 1000 images with random camera positions
num_frames = 1000

for i in range(num_frames):
    # Randomize camera position (orbit around scene)
    radius = random.uniform(150, 300)  # Distance from center
    angle = random.uniform(0, 360)  # Degrees around Z-axis
    height = random.uniform(50, 200)  # Elevation

    # Calculate position
    x = radius * np.cos(np.radians(angle))
    y = radius * np.sin(np.radians(angle))
    z = height

    # Update camera
    camera.set_position((x, y, z))
    camera.set_look_at((0, 0, 50))  # Always point at scene center

    # Step simulation
    simulation_context.step(render=True)

    # Capture data
    rgb_data = rgb_annotator.get_data()
    cv2.imwrite(f"{output_dir}/rgb_{i:04d}.png", rgb_data)

    if i % 100 == 0:
        print(f"Generated {i}/{num_frames} images")

print(f"Dataset generation complete! {num_frames} images saved.")
```

### Step 2: Randomize Object Poses

```python
# Randomize object positions for each frame
objects_to_randomize = [
    "/World/Warehouse/Box_001",
    "/World/Warehouse/Box_002",
    "/World/Warehouse/Bin_001"
]

for i in range(num_frames):
    # Randomize each object's position
    for obj_path in objects_to_randomize:
        obj_prim = stage_utils.get_prim_at_path(obj_path)

        # Random position within bounds
        x = random.uniform(-200, 200)
        y = random.uniform(-200, 200)
        z = random.uniform(10, 150)  # Above ground

        # Apply transform
        obj_prim.GetAttribute('xformOp:translate').Set((x, y, z))

    # Capture data...
```

### Step 3: Randomize Lighting (covered in next lesson)

We'll explore lighting randomization in detail in the **Domain Randomization** lesson.

---

## Part 7: Export Formats for AI Training

Isaac Sim Replicator supports multiple export formats:

### COCO Format (Object Detection)

**Structure**:
```json
{
  "images": [
    {"id": 0, "file_name": "rgb_0000.png", "width": 1280, "height": 720}
  ],
  "annotations": [
    {
      "id": 0,
      "image_id": 0,
      "category_id": 1,
      "bbox": [320, 180, 160, 180],
      "area": 28800,
      "iscrowd": 0
    }
  ],
  "categories": [
    {"id": 1, "name": "box"},
    {"id": 2, "name": "robot"}
  ]
}
```

**Use**: YOLOv8, Detectron2, MMDetection training

### KITTI Format (3D Object Detection)

**Structure** (per-frame `.txt` file):
```
Box 0.00 0 -1.57 320 180 480 360 1.5 1.0 2.0 2.5 1.2 10.3 -1.57
Robot 0.00 0 -0.78 500 200 650 400 0.8 0.8 1.5 1.0 0.5 8.2 -0.78
```

**Fields**: Class, truncation, occlusion, alpha, bbox_2d (x1, y1, x2, y2), dimensions_3d (h, w, l), location_3d (x, y, z), rotation_y

**Use**: 3D object detection, LiDAR-camera fusion

### Custom Format

```python
# Define your own export structure
custom_data = {
    "frame_id": i,
    "timestamp": simulation_context.current_time,
    "camera_pose": {
        "position": camera.get_position(),
        "rotation": camera.get_rotation()
    },
    "rgb_path": f"rgb_{i:04d}.png",
    "depth_path": f"depth_{i:04d}.npy",
    "annotations": bbox_data['data']
}

with open(f"{output_dir}/frame_{i:04d}.json", 'w') as f:
    json.dump(custom_data, f, indent=2)
```

---

## Part 8: Validating Synthetic Data Quality

Before using synthetic data for training, validate it matches your requirements:

### Quality Checklist

- [ ] **Class distribution**: Each class has >500 examples
- [ ] **Viewpoint diversity**: Objects seen from 360° angles
- [ ] **Occlusion**: Some objects partially hidden (realistic scenarios)
- [ ] **Lighting variation**: Multiple light conditions (covered in domain randomization)
- [ ] **Scale variation**: Objects at different distances from camera
- [ ] **Annotation accuracy**: Bounding boxes tightly fit objects (manual spot-check)

### Visualization Tools

```python
import matplotlib.pyplot as plt

# Plot class distribution
class_counts = {}
for bbox in all_annotations:
    cls = bbox['semantic_id']
    class_counts[cls] = class_counts.get(cls, 0) + 1

plt.bar(class_counts.keys(), class_counts.values())
plt.xlabel('Class')
plt.ylabel('Count')
plt.title('Class Distribution in Synthetic Dataset')
plt.show()
```

### Spot-Check Annotations

Manually review 10-20 random images with overlaid bounding boxes to ensure:
- No missing objects
- Correct class labels
- Tight bounding boxes (not too loose or too tight)

---

## Part 9: Hands-On Exercise

### Exercise: Generate a Tabletop Object Detection Dataset

**Objective**: Create a dataset for training a model to detect kitchen objects on a countertop.

#### Requirements

1. **Scene Setup**:
   - Tabletop from previous lesson
   - 6 object classes: Apple, orange, mug, plate, spoon, bowl
   - 3 instances of each class (18 objects total)

2. **Data Generation**:
   - 500 RGB images (1280x720 resolution)
   - 500 depth maps
   - 500 semantic segmentation masks
   - 500 bounding box annotation files (COCO JSON format)

3. **Randomization**:
   - Camera: Random viewpoints (orbit radius 80-150cm, height 50-120cm)
   - Objects: Random positions on tabletop (no overlap)

4. **Validation**:
   - Each class has 80-120 instances across 500 images
   - Manual review of 20 random images shows correct annotations

#### Deliverables

- Python script: `generate_tabletop_dataset.py`
- Dataset folder with:
  - `rgb/` (500 images)
  - `depth/` (500 .npy files)
  - `semantic/` (500 segmentation masks)
  - `annotations.json` (COCO format)
- Validation report: Class distribution bar chart + 20 annotated image samples

**Estimated Time**: 1-2 hours

---

## Summary

In this lesson, you learned:

✅ Types of synthetic data (RGB, depth, segmentation, bounding boxes)
✅ How to configure cameras and annotators in Isaac Sim
✅ Programmatic data capture via Python API
✅ Automatic object annotation for detection
✅ Large-scale dataset generation with randomization
✅ Export formats for AI training (COCO, KITTI, custom)
✅ Validation techniques for synthetic datasets

**Key Takeaways**:
- **Synthetic data** dramatically reduces manual annotation costs
- **Replicator** enables thousands of images per hour
- **Randomization** is critical for dataset diversity
- **Validation** ensures synthetic data quality matches real-world needs

---

## Next Steps

In the next lesson (**Domain Randomization**), you'll learn:

- Advanced randomization techniques (textures, lighting, physics)
- Sim-to-real transfer strategies
- Closing the reality gap with diverse training data
- Balancing realism vs. diversity in synthetic datasets

---

## Additional Resources

- **Replicator Documentation**: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html
- **COCO Dataset Format**: https://cocodataset.org/#format-data
- **KITTI Dataset Format**: https://www.cvlibs.net/datasets/kitti/eval_object.php
- **Synthetic Data Best Practices**: https://arxiv.org/abs/2109.09930 (academic paper)
- **Isaac Sim Python API Reference**: https://docs.omniverse.nvidia.com/isaacsim/latest/api.html

:::tip Real-World Impact
Many production AI models (e.g., Tesla Autopilot, Amazon warehouse robots) use synthetic data to augment real data, dramatically improving model robustness!
:::
