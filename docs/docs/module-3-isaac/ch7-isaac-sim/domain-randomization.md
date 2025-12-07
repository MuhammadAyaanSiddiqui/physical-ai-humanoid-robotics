# Domain Randomization

## Overview

**Domain randomization** is a powerful technique that bridges the gap between simulation and reality. By training AI models on highly varied synthetic data (randomized textures, lighting, physics, object poses), robots learn to generalize to the unpredictable real world. This enables **sim-to-real transfer** - deploying policies trained purely in simulation directly on physical robots.

**What You'll Learn**:
- Randomize textures, materials, and lighting in Isaac Sim
- Vary physics parameters (friction, mass, damping)
- Randomize object poses, scales, and quantities
- Implement systematic randomization strategies
- Validate sim-to-real transfer effectiveness

**Prerequisites**:
- Completed: Synthetic Data Generation (previous lesson)
- Python programming
- Understanding of AI training concepts (transfer learning, generalization)

**Estimated Time**: 2-3 hours

---

## Learning Objectives

By the end of this lesson, you will be able to:

1. Understand the "reality gap" and why domain randomization helps
2. Randomize visual properties (colors, textures, lighting)
3. Randomize physical properties (mass, friction, restitution)
4. Randomize scene composition (object count, poses, scales)
5. Implement complete randomization pipelines for data generation
6. Measure sim-to-real transfer success

---

## Part 1: The Reality Gap & Why Randomization Works

### What is the Reality Gap?

The **reality gap** is the difference between simulated and real-world environments:

| Aspect | Simulation | Real World | Gap |
|--------|----------|------------|-----|
| **Lighting** | Perfect HDR environments | Unpredictable (clouds, shadows, reflections) | Large |
| **Physics** | Idealized contact models | Messy (slipping, bouncing, deformation) | Medium |
| **Sensors** | Noise-free data | Sensor noise, motion blur, occlusion | Large |
| **Textures** | Clean 3D models | Scratches, dirt, wear, reflections | Medium |

**Problem**: A robot trained in a pristine simulation fails when encountering real-world variability.

### How Domain Randomization Solves This

Instead of making simulation perfectly realistic (impossible), we make it **unrealistically diverse**:

**Strategy**: Train on 1000s of randomized scenarios → Model learns invariant features (e.g., "an apple is round, regardless of color or lighting")

**Example**:

- **Without randomization**: Train on perfect red apples in white light → Fails on yellow apples in sunlight
- **With randomization**: Train on random fruit colors (red, green, yellow, purple!) in random lighting → Recognizes all real apples

:::tip The Randomization Paradox
Making simulation **less** realistic (but more diverse) actually **improves** real-world performance. This counterintuitive insight drives modern sim-to-real robotics!
:::

---

## Part 2: Visual Randomization (Textures & Materials)

### Randomizing Object Colors

Let's start simple: randomize the color of boxes in a warehouse scene.

**Python Script** (`randomize_colors.py`):

```python
import omni.replicator.core as rep
import random

# Get all box objects in scene
boxes = rep.get.prims(path_pattern="/World/Warehouse/Box_*")

# Randomize colors for each frame
with rep.trigger.on_frame(num_frames=100):
    with boxes:
        # Random RGB color for each box
        rep.randomizer.color(
            colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1))  # Full color spectrum
        )

# Run data generation
rep.orchestrator.run()
```

**Result**: Each frame, boxes have completely random colors (red, blue, green, purple, etc.)

### Randomizing Textures

For more realism, randomize entire textures:

```python
# Prepare texture library
texture_paths = [
    "omniverse://localhost/NVIDIA/Materials/Base/Wood/Oak.mdl",
    "omniverse://localhost/NVIDIA/Materials/Base/Metal/Aluminum.mdl",
    "omniverse://localhost/NVIDIA/Materials/Base/Stone/Concrete.mdl",
    "omniverse://localhost/NVIDIA/Materials/Base/Plastic/Glossy_Plastic.mdl"
]

# Randomize materials
with rep.trigger.on_frame(num_frames=100):
    with boxes:
        rep.randomizer.materials(
            materials=rep.distribution.choice(texture_paths)
        )
```

**Result**: Boxes randomly become wood, metal, concrete, or plastic each frame

### Randomizing Floor Textures

```python
ground_plane = rep.get.prims(path_pattern="/World/GroundPlane")

floor_textures = [
    "concrete_001.mdl",
    "tile_checkered.mdl",
    "wood_planks.mdl",
    "asphalt.mdl"
]

with rep.trigger.on_frame(num_frames=100):
    with ground_plane:
        rep.randomizer.materials(
            materials=rep.distribution.choice(floor_textures)
        )
```

:::caution Texture Diversity Balance
Too much texture variation (e.g., psychedelic colors) can confuse models. Aim for diversity within *plausible* real-world ranges (browns, grays, natural colors).
:::

---

## Part 3: Lighting Randomization

Lighting has the **largest impact** on sim-to-real transfer. Randomize intensity, color, and direction.

### Randomizing Light Intensity

```python
# Get all lights in scene
lights = rep.get.prims(path_pattern="/World/Lights/*")

with rep.trigger.on_frame(num_frames=100):
    with lights:
        rep.randomizer.attribute(
            "intensity",
            rep.distribution.uniform(500, 10000)  # From dim to very bright
        )
```

### Randomizing Light Color (Temperature)

Simulate different times of day (warm sunset vs. cool overcast):

```python
with rep.trigger.on_frame(num_frames=100):
    with lights:
        # Color temperature (Kelvin)
        # 2500K = Warm (sunset), 6500K = Neutral, 9000K = Cool (overcast)
        color_temp = rep.distribution.uniform(2500, 9000)
        rep.randomizer.attribute("colorTemperature", color_temp)
```

### Randomizing Light Position

Move lights to simulate changing sun angle or moving clouds:

```python
with rep.trigger.on_frame(num_frames=100):
    with lights:
        # Random position in hemisphere above scene
        rep.modify.pose(
            position=rep.distribution.uniform((-500, -500, 100), (500, 500, 800))
        )
```

### Complete Lighting Randomization Example

```python
# Randomize all lighting aspects simultaneously
with rep.trigger.on_frame(num_frames=1000):
    # Dome light (ambient)
    dome_light = rep.get.prims(path_pattern="/World/DomeLight")
    with dome_light:
        rep.randomizer.attribute("intensity", rep.distribution.uniform(300, 1500))
        rep.randomizer.attribute("colorTemperature", rep.distribution.uniform(4000, 8000))

    # Directional light (sun)
    sun_light = rep.get.prims(path_pattern="/World/SunLight")
    with sun_light:
        rep.randomizer.attribute("intensity", rep.distribution.uniform(1.0, 5.0))
        rep.randomizer.attribute("angle", rep.distribution.uniform(0, 90))  # Sun elevation

    # Rect lights (ceiling panels)
    rect_lights = rep.get.prims(path_pattern="/World/CeilingLights/*")
    with rect_lights:
        # Randomly turn some lights on/off
        rep.randomizer.attribute("visibility", rep.distribution.choice([True, False]))
        rep.randomizer.attribute("intensity", rep.distribution.uniform(2000, 8000))
```

**Result**: Each frame simulates different lighting conditions (morning, midday, overcast, artificial light, etc.)

---

## Part 4: Physics Randomization

Physical properties affect how robots interact with objects. Randomize friction, mass, and restitution.

### Randomizing Object Mass

```python
import omni.physx as physx

# Get all box physics bodies
boxes = rep.get.prims(path_pattern="/World/Warehouse/Box_*")

with rep.trigger.on_frame(num_frames=100):
    with boxes:
        # Random mass (0.1 kg to 5 kg)
        rep.randomizer.attribute("physics:mass", rep.distribution.uniform(0.1, 5.0))
```

**Why This Matters**: A robot grasping a box needs to adapt to different weights. Randomizing mass forces the model to learn robust grasping strategies.

### Randomizing Friction

```python
# Randomize friction coefficient
with rep.trigger.on_frame(num_frames=100):
    with boxes:
        # Static friction (0.1 = slippery, 1.0 = grippy)
        rep.randomizer.attribute(
            "physics:staticFriction",
            rep.distribution.uniform(0.1, 1.0)
        )
        # Dynamic friction
        rep.randomizer.attribute(
            "physics:dynamicFriction",
            rep.distribution.uniform(0.05, 0.9)
        )
```

**Use Case**: Training a mobile robot to push objects. Random friction simulates different floor surfaces (tile, carpet, concrete).

### Randomizing Restitution (Bounciness)

```python
with rep.trigger.on_frame(num_frames=100):
    with boxes:
        # Restitution (0.0 = no bounce, 1.0 = perfect bounce)
        rep.randomizer.attribute(
            "physics:restitution",
            rep.distribution.uniform(0.0, 0.8)
        )
```

**Use Case**: Training a robot to catch falling objects. Random bounciness prepares for unpredictable real-world behavior.

### Randomizing Joint Damping (For Robots)

```python
# For robot joints (e.g., humanoid arms)
robot_joints = rep.get.prims(path_pattern="/World/Humanoid/*/Joint_*")

with rep.trigger.on_frame(num_frames=100):
    with robot_joints:
        # Joint damping (resistance to motion)
        rep.randomizer.attribute(
            "drive:damping",
            rep.distribution.uniform(0.1, 10.0)
        )
```

**Why**: Real robot joints have varying stiffness due to temperature, wear, etc. Randomized damping builds robustness.

---

## Part 5: Pose & Scene Composition Randomization

Randomize object positions, orientations, scales, and quantities.

### Randomizing Object Positions

```python
# Scatter boxes randomly on warehouse floor
boxes = rep.get.prims(path_pattern="/World/Warehouse/Box_*")

with rep.trigger.on_frame(num_frames=100):
    with boxes:
        # Random position within bounds
        rep.modify.pose(
            position=rep.distribution.uniform(
                (-400, -400, 10),  # Min (x, y, z)
                (400, 400, 200)     # Max (x, y, z)
            )
        )
```

### Randomizing Object Rotations

```python
with rep.trigger.on_frame(num_frames=100):
    with boxes:
        # Random rotation (Euler angles in degrees)
        rep.modify.pose(
            rotation=rep.distribution.uniform(
                (0, 0, 0),       # Min rotation
                (360, 360, 360)  # Max rotation (full 3D randomization)
            )
        )
```

### Randomizing Object Scales

```python
with rep.trigger.on_frame(num_frames=100):
    with boxes:
        # Random scale (0.5x to 2x original size)
        rep.modify.attribute(
            "xformOp:scale",
            rep.distribution.uniform((0.5, 0.5, 0.5), (2.0, 2.0, 2.0))
        )
```

**Use Case**: Training object detection to recognize objects at different distances/sizes.

### Randomizing Number of Objects

```python
# Spawn random number of boxes each frame
with rep.trigger.on_frame(num_frames=100):
    num_boxes = rep.distribution.uniform(5, 20)  # Random count

    # Delete existing boxes
    rep.utils.remove(boxes)

    # Spawn new boxes
    boxes = rep.create.from_usd(
        usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/.../box.usd",
        count=num_boxes,
        semantics=[("class", "box")]
    )

    # Randomize their positions
    with boxes:
        rep.modify.pose(
            position=rep.distribution.uniform((-400, -400, 10), (400, 400, 200))
        )
```

**Result**: Each frame has a different number of objects at random locations - very diverse data!

---

## Part 6: Advanced Randomization Strategies

### Correlated Randomization

Some variables should be randomized together (e.g., time of day affects both lighting color and intensity):

```python
# Define time-of-day correlation
def randomize_time_of_day():
    time = random.choice(["morning", "midday", "evening", "night"])

    if time == "morning":
        sun_intensity = rep.distribution.uniform(2.0, 3.5)
        sun_color_temp = rep.distribution.uniform(4000, 5500)  # Warm
        ambient_intensity = rep.distribution.uniform(800, 1200)
    elif time == "midday":
        sun_intensity = rep.distribution.uniform(3.5, 5.0)
        sun_color_temp = rep.distribution.uniform(5500, 6500)  # Neutral
        ambient_intensity = rep.distribution.uniform(1200, 1800)
    elif time == "evening":
        sun_intensity = rep.distribution.uniform(1.0, 2.0)
        sun_color_temp = rep.distribution.uniform(2500, 4000)  # Very warm (sunset)
        ambient_intensity = rep.distribution.uniform(400, 800)
    else:  # night
        sun_intensity = rep.distribution.uniform(0.0, 0.5)
        sun_color_temp = rep.distribution.uniform(6000, 8000)  # Cool (moonlight)
        ambient_intensity = rep.distribution.uniform(100, 400)

    return sun_intensity, sun_color_temp, ambient_intensity

# Apply correlated randomization
with rep.trigger.on_frame(num_frames=1000):
    intensity, color_temp, ambient = randomize_time_of_day()

    sun_light = rep.get.prims(path_pattern="/World/SunLight")
    with sun_light:
        rep.randomizer.attribute("intensity", intensity)
        rep.randomizer.attribute("colorTemperature", color_temp)

    dome_light = rep.get.prims(path_pattern="/World/DomeLight")
    with dome_light:
        rep.randomizer.attribute("intensity", ambient)
```

### Structured Randomization (Per-Class)

Different object types may need different randomization ranges:

```python
# Boxes: High color variation (packaging is diverse)
boxes = rep.get.prims(path_pattern="/World/Warehouse/Box_*", semantics=[("class", "box")])
with rep.trigger.on_frame():
    with boxes:
        rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))

# Robots: Limited color variation (mostly gray/black in real world)
robots = rep.get.prims(path_pattern="/World/Robots/*", semantics=[("class", "robot")])
with rep.trigger.on_frame():
    with robots:
        rep.randomizer.color(
            colors=rep.distribution.choice([
                (0.2, 0.2, 0.2),  # Dark gray
                (0.4, 0.4, 0.4),  # Medium gray
                (0.1, 0.1, 0.1)   # Black
            ])
        )
```

---

## Part 7: Implementing a Complete Randomization Pipeline

Let's combine all techniques into a comprehensive warehouse randomization script.

### Complete Example: `warehouse_randomization_pipeline.py`

```python
import omni.replicator.core as rep
import random

# Initialize scene
rep.new_layer()

# Load warehouse base scene
warehouse = rep.create.from_usd("omniverse://localhost/.../warehouse_base.usd")

# Define randomization parameters
NUM_FRAMES = 5000  # Generate 5000 diverse images

# 1. Object randomization
boxes = rep.get.prims(path_pattern="/World/Warehouse/Box_*")
pallets = rep.get.prims(path_pattern="/World/Warehouse/Pallet_*")
robots = rep.get.prims(path_pattern="/World/Warehouse/Robot_*")

with rep.trigger.on_frame(num_frames=NUM_FRAMES):
    # Visual randomization
    with boxes:
        rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
        rep.modify.pose(
            position=rep.distribution.uniform((-400, -400, 10), (400, 400, 150)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )

    # Physics randomization
    with boxes:
        rep.randomizer.attribute("physics:mass", rep.distribution.uniform(0.5, 3.0))
        rep.randomizer.attribute("physics:staticFriction", rep.distribution.uniform(0.2, 0.9))

    # Lighting randomization
    dome_light = rep.get.prims(path_pattern="/World/DomeLight")
    with dome_light:
        rep.randomizer.attribute("intensity", rep.distribution.uniform(500, 2000))
        rep.randomizer.attribute("colorTemperature", rep.distribution.uniform(3000, 8000))

    sun_light = rep.get.prims(path_pattern="/World/SunLight")
    with sun_light:
        rep.randomizer.attribute("intensity", rep.distribution.uniform(1.0, 4.0))
        rep.randomizer.attribute("angle", rep.distribution.uniform(15, 75))

    # Camera randomization
    camera = rep.create.camera(resolution=(1280, 720))
    with camera:
        rep.modify.pose(
            position=rep.distribution.uniform((-300, -300, 50), (300, 300, 250)),
            look_at=rep.distribution.uniform((-100, -100, 0), (100, 100, 100))
        )

# Configure data output
rgb_writer = rep.WriterRegistry.get("BasicWriter")
rgb_writer.initialize(output_dir="/data/warehouse_randomized", rgb=True, distance_to_camera=True)

# Attach annotators
camera.add_annotator(rep.AnnotatorRegistry.get_annotator("rgb"))
camera.add_annotator(rep.AnnotatorRegistry.get_annotator("distance_to_camera"))
camera.add_annotator(rep.AnnotatorRegistry.get_annotator("semantic_segmentation"))
camera.add_annotator(rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight"))

# Run pipeline
rep.orchestrator.run()

print(f"Generated {NUM_FRAMES} frames with full domain randomization!")
```

**Result**: 5000 highly diverse images ready for AI training!

---

## Part 8: Measuring Sim-to-Real Transfer Success

After training on randomized data, how do you know if it will work in the real world?

### Validation Metrics

| Metric | Simulation | Real World | Target |
|--------|-----------|------------|--------|
| **Object Detection mAP** | 0.92 | 0.78 | >0.75 |
| **Grasp Success Rate** | 95% | 82% | >80% |
| **Navigation Success** | 98% | 85% | >85% |
| **Task Completion Time** | 12s | 18s | &lt;20s |

**Goal**: Real-world performance within **10-15%** of simulation performance

### Best Practices for Sim-to-Real Transfer

1. **Start conservative**: Randomize one variable at a time, test transfer
2. **Prioritize visual randomization**: Lighting and textures have biggest impact
3. **Keep physics realistic**: Don't randomize beyond plausible real-world ranges
4. **Validate incrementally**: Test on real robot after each randomization addition
5. **Collect real-world failure cases**: Add those scenarios to simulation

---

## Part 9: Hands-On Exercise

### Exercise: Implement Comprehensive Randomization for Tabletop Scene

**Objective**: Create a domain randomization pipeline for the tabletop manipulation scene from the previous lesson.

#### Requirements

1. **Visual Randomization**:
   - Object colors: Full RGB spectrum
   - Table texture: 3 material options (wood, plastic, metal)
   - Floor texture: 2 options (tile, concrete)

2. **Lighting Randomization**:
   - Dome light intensity: 300-1500
   - Color temperature: 3000K-7000K
   - Rect light (overhead): On/off randomly, intensity 1000-5000

3. **Physics Randomization**:
   - Object mass: 0.1kg - 0.5kg
   - Friction: 0.3 - 0.8

4. **Pose Randomization**:
   - Object positions: Random on tabletop (no overlap)
   - Object rotations: Full 3D random
   - Camera position: Orbit around table (radius 80-150cm, height 50-120cm)

5. **Generate 2000 diverse images** with all randomizations active

#### Deliverables

- Python script: `tabletop_randomization.py`
- Dataset: 2000 RGB images + depth + annotations
- Validation report:
  - Show 10 example images demonstrating diversity
  - Histogram of lighting intensities
  - Histogram of object counts per frame

**Estimated Time**: 2 hours

---

## Summary

In this lesson, you learned:

✅ The reality gap and why domain randomization solves it
✅ Visual randomization (colors, textures, materials, lighting)
✅ Physics randomization (mass, friction, restitution, damping)
✅ Pose and composition randomization (positions, rotations, scales, counts)
✅ Advanced strategies (correlated randomization, structured randomization)
✅ Complete randomization pipelines for large-scale data generation
✅ Measuring sim-to-real transfer success

**Key Takeaways**:
- **Diversity > Realism**: Unrealistically diverse simulation → Better real-world performance
- **Lighting is critical**: Has the largest impact on sim-to-real transfer
- **Physics should stay plausible**: Don't randomize beyond real-world ranges
- **Validate iteratively**: Test on real robot after each randomization step

---

## Next Steps

**Congratulations!** You've completed Chapter 7: NVIDIA Isaac Sim.

In **Chapter 8: Perception with Isaac ROS**, you'll learn:

- Visual SLAM for real-time mapping and localization
- Object detection with DNN inference (YOLO, Faster R-CNN)
- Depth estimation from stereo and monocular cameras
- 6DOF pose estimation for object grasping

---

## Additional Resources

- **Domain Randomization Paper** (OpenAI): https://arxiv.org/abs/1703.06907
- **Sim-to-Real Transfer Best Practices**: https://www.nvidia.com/en-us/on-demand/session/gtcspring21-s31880/
- **Isaac Sim Replicator Examples**: https://github.com/NVIDIA-Omniverse/IsaacSim-Samples
- **Physics Randomization for Robotics**: https://arxiv.org/abs/1710.06537
- **Visual Domain Randomization**: https://arxiv.org/abs/1906.02110

:::success Major Milestone!
You can now generate unlimited diverse training data for AI perception models. This is a critical skill for modern Physical AI development!
:::
