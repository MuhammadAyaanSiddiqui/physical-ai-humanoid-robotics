# Photorealistic Environments & Scene Creation

## Overview

One of Isaac Sim's most powerful features is its ability to create **photorealistic environments** that closely mimic real-world conditions. This enables you to train AI perception models on synthetic data that transfers effectively to physical robots, reducing the need for expensive real-world data collection.

**What You'll Learn**:
- Create custom scenes and environments in Isaac Sim
- Import 3D assets (CAD models, props, textures)
- Configure lighting for realistic rendering
- Apply materials and textures for photorealism
- Optimize scenes for simulation performance

**Prerequisites**:
- Completed: Installation & Setup (previous lesson)
- Familiarity with 3D modeling concepts (helpful but not required)
- Isaac Sim running successfully on your system

**Estimated Time**: 2-3 hours

---

## Learning Objectives

By the end of this lesson, you will be able to:

1. Create a new scene from scratch in Isaac Sim
2. Import 3D models (USD, OBJ, FBX, URDF) into scenes
3. Configure HDR lighting and environment maps
4. Apply PBR (Physically-Based Rendering) materials
5. Compose realistic indoor and outdoor environments
6. Optimize scene performance for real-time simulation

---

## Part 1: Understanding USD (Universal Scene Description)

Isaac Sim uses **USD (Universal Scene Description)** as its native file format, developed by Pixar for professional 3D animation and visual effects.

### What is USD?

USD is a scene description framework that allows you to:

- **Compose scenes**: Combine multiple 3D assets into layered hierarchies
- **Non-destructive editing**: Make changes without modifying original assets
- **Efficient collaboration**: Multiple users can work on different parts of a scene
- **Scalability**: Handle massive scenes with millions of primitives

**Key USD Concepts**:

| Concept | Description | Example |
|---------|-------------|---------|
| **Stage** | The root container for all scene data | Entire warehouse environment |
| **Prim** | A single scene element (object, light, camera) | Robot, box, spotlight |
| **Layer** | A USD file containing scene modifications | Base scene, lighting pass, animation |
| **Composition** | Combining multiple layers into a final scene | Base + props + lighting layers |

:::info Why USD for Robotics?
USD enables **domain randomization** (varying scene properties automatically) and **procedural generation** (creating scenes programmatically), both critical for training robust AI models.
:::

---

## Part 2: Creating Your First Scene from Scratch

### Step 1: Start a New Scene

1. Launch Isaac Sim
2. **File** > **New**
3. You'll see an empty viewport (black screen)

### Step 2: Add a Ground Plane

A ground plane provides a floor for objects to rest on and improves visual reference.

1. **Create** > **Mesh** > **Plane**
2. In the **Property** panel (right side), set:
   - **Name**: `GroundPlane`
   - **Scale**: `X: 50, Y: 50, Z: 1` (large flat surface)
   - **Position**: `X: 0, Y: 0, Z: 0`

You should now see a large gray plane in the viewport.

### Step 3: Add a Default Light

Without lighting, the scene will be dark. Add a simple light source:

1. **Create** > **Light** > **Sphere Light**
2. In **Property** panel, set:
   - **Name**: `SkyLight`
   - **Position**: `X: 0, Y: 0, Z: 500` (high above ground)
   - **Intensity**: `30000` (bright daylight simulation)
   - **Radius**: `100` (large soft light source)

Your ground plane should now be visible and lit.

### Step 4: Add a Camera

1. **Create** > **Camera**
2. In **Property** panel, set:
   - **Name**: `MainCamera`
   - **Position**: `X: -300, Y: 300, Z: 150` (angled view)
   - **Rotation**: Point camera toward origin (use **Look At** tool)

To view from this camera:
- Select **Camera** > **Viewport Camera** > **MainCamera**

### Step 5: Save Your Scene

1. **File** > **Save As**
2. Name: `my_first_scene.usd`
3. Location: `~/Documents/Isaac-Sim/scenes/`

:::tip USD Naming Convention
Use descriptive names with underscores (e.g., `warehouse_interior.usd`, `outdoor_park.usd`) and keep scenes organized in topic folders.
:::

---

## Part 3: Importing 3D Assets

Isaac Sim supports multiple 3D file formats for asset importing:

| Format | Use Case | Pros | Cons |
|--------|----------|------|------|
| **USD (.usd, .usda)** | Native Isaac Sim format | Best performance, full feature support | Requires conversion from other formats |
| **URDF (.urdf)** | Robot descriptions from ROS 2 | Direct import from Module 1 robots | Limited material/texture support |
| **OBJ (.obj)** | Simple static meshes | Widely supported, small file size | No animation or physics properties |
| **FBX (.fbx)** | Complex models with animations | Good for characters and rigged models | Large file size, may need cleanup |
| **STL (.stl)** | CAD models | Engineering-accurate geometry | No color or texture information |

### Importing a Simple Prop (OBJ)

Let's import a table model:

1. Download a free table model from: https://free3d.com/3d-models/table (OBJ format)
2. In Isaac Sim: **File** > **Import**
3. Select your `.obj` file
4. In the import dialog:
   - **Import As**: `Mesh`
   - **Scale**: `100` (OBJ files often use centimeters, USD uses cm by default)
   - Click **Import**

The table should appear at the origin (0, 0, 0).

### Importing a Robot (URDF)

From Module 1, you created URDF robot models. Let's bring one into Isaac Sim:

1. **Isaac Utils** > **URDF Importer**
2. Browse to your URDF file (e.g., `simple_humanoid.urdf` from Module 1)
3. Set import options:
   - **Import Root Prim Name**: `MyHumanoid`
   - **Fix Base**: Unchecked (allow robot to move freely)
   - **Import Inertia Tensor**: Checked (for physics accuracy)
4. Click **Import**

Your robot will appear in the scene with physics and joint properties preserved.

:::caution URDF Mesh Paths
Ensure URDF mesh references (e.g., `<mesh filename="package://my_robot/meshes/base.stl"/>`) have correct absolute paths, or place meshes in the same directory as the URDF.
:::

### Importing from NVIDIA Omniverse Nucleus

Isaac Sim has access to a massive library of pre-built assets via NVIDIA Nucleus:

1. Open **Content Browser** (bottom panel)
2. Navigate to: `omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/`
3. Browse categories:
   - **Environments**: Pre-built warehouses, offices, outdoor scenes
   - **Props**: Shelves, bins, tools, furniture
   - **Robots**: Sample robots (Carter, Franka, UR10, etc.)

4. **Drag and drop** any asset from Content Browser into your viewport

**Popular Starter Assets**:
- `NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Simple_Warehouse/warehouse.usd` - Indoor warehouse
- `NVIDIA/Assets/Isaac/2023.1.1/Isaac/Props/Pallet/pallet.usd` - Shipping pallet
- `NVIDIA/Assets/Isaac/2023.1.1/Isaac/Robots/Franka/franka.usd` - Franka Emika arm

---

## Part 4: Lighting for Photorealism

Lighting is the most critical factor in achieving photorealistic rendering. Isaac Sim supports multiple light types and advanced techniques.

### Light Types

| Light Type | Use Case | Visual Effect |
|------------|----------|---------------|
| **Sphere Light** | Sun, large soft lights | Even illumination, soft shadows |
| **Rect Light** | Windows, panels | Directional area lighting |
| **Distant Light** | Outdoor sunlight | Parallel rays, sharp shadows |
| **Dome Light** | Sky/environment lighting | 360° ambient illumination |

### Step-by-Step: Create Realistic Indoor Lighting

**Scenario**: Warehouse with skylights and overhead panels

#### 1. Add Dome Light (Ambient Sky)

1. **Create** > **Light** > **Dome Light**
2. In **Property** panel:
   - **Intensity**: `500` (moderate ambient light)
   - **Texture**: Load an HDR environment map (optional, see below)

:::tip HDR Environment Maps
High Dynamic Range (HDR) images simulate realistic sky and environmental reflections. Download free HDRIs from https://polyhaven.com/hdris and load via **Texture** field in Dome Light properties.
:::

#### 2. Add Rect Lights (Ceiling Panels)

Simulate overhead fluorescent panels:

1. **Create** > **Light** > **Rect Light**
2. Set properties:
   - **Position**: `X: 0, Y: 0, Z: 400` (ceiling height)
   - **Rotation**: Point downward (90° on X-axis)
   - **Scale**: `X: 200, Y: 100, Z: 1` (panel size)
   - **Intensity**: `5000`
   - **Color**: Slightly blue-tinted white (`#E8F4FF`) for fluorescent look

3. **Duplicate** the light (Ctrl+D) and position copies across the ceiling

#### 3. Add Distant Light (Sunlight through Skylights)

1. **Create** > **Light** > **Distant Light**
2. Set properties:
   - **Intensity**: `3.0` (bright but not overpowering)
   - **Angle**: `45°` (mid-day sun angle)
   - **Color**: Warm white (`#FFF4E6`)
   - **Enable Shadows**: Checked

This simulates sunlight streaming through skylights.

### Testing Your Lighting

1. Press **Play** (▶) to see real-time lighting updates
2. Move camera around scene to check for:
   - No pure black areas (add fill lights if needed)
   - Realistic shadow softness
   - Proper reflections on shiny surfaces

---

## Part 5: Materials & Textures for Realism

Isaac Sim uses **MDL (Material Definition Language)** with Physically-Based Rendering (PBR) for realistic materials.

### PBR Material Properties

| Property | Description | Example Values |
|----------|-------------|----------------|
| **Base Color** | Surface color | Wood: #8B4513, Metal: #C0C0C0 |
| **Metallic** | Metal vs. dielectric | 0.0 = Plastic, 1.0 = Metal |
| **Roughness** | Surface smoothness | 0.0 = Mirror, 1.0 = Matte |
| **Normal Map** | Surface detail (bumps) | Texture file (.png) |
| **Emissive** | Self-illumination | For screens, LEDs |

### Step-by-Step: Apply Materials

#### 1. Create a Material

1. Select an object (e.g., your ground plane)
2. In **Property** panel, find **Material** section
3. Click **+ Add** > **OmniPBR**

#### 2. Configure PBR Properties

For a **concrete floor**:

- **Base Color**: Gray `#808080`
- **Metallic**: `0.0` (non-metallic)
- **Roughness**: `0.7` (slightly rough, not shiny)
- **Normal Map**: Optional - download concrete normal map from https://polyhaven.com/textures

For a **metal shelf**:

- **Base Color**: Light gray `#B0B0B0`
- **Metallic**: `1.0` (fully metallic)
- **Roughness**: `0.3` (brushed metal)

#### 3. Apply Textures

To add realistic surface details:

1. Download a free PBR texture set (Base Color, Normal, Roughness) from https://polyhaven.com/textures
2. In Material properties:
   - **Albedo Map**: Load Base Color texture
   - **Normal Map**: Load Normal texture
   - **Roughness Map**: Load Roughness texture

3. Adjust **UV Tiling** to scale texture (e.g., `10x10` for repeating floor tiles)

:::tip Quick Material Library
Isaac Sim includes pre-made materials: **Content Browser** > `omniverse://localhost/NVIDIA/Materials/` - drag and drop onto objects
:::

---

## Part 6: Building a Complete Environment

Let's combine everything to build a **warehouse pick-and-place environment**.

### Environment Requirements

- Floor area: 10m x 10m
- Shelving units: 3 rows, 5 shelves each
- Lighting: Overhead panels + skylights
- Props: Boxes, pallets, bins
- Robot: Franka arm on mobile base

### Step-by-Step Assembly

#### 1. Create Base Structure

```python
# Optionally, use Python API for procedural scene generation
# (We'll cover this in detail in later lessons)

# For now, use GUI:
# - Create > Mesh > Cube (for walls)
# - Scale to 1000cm (10m) length
# - Duplicate and position to form warehouse perimeter
```

#### 2. Add Shelving Units

1. From Content Browser, drag: `NVIDIA/Assets/Isaac/.../Props/Shelf/shelf_unit.usd`
2. Position: `X: -300, Y: 0, Z: 0`
3. Duplicate (Ctrl+D) and space evenly: `X: -300, 0, 300`

#### 3. Populate with Objects

Add various props:

- **Boxes**: `Props/Boxes/cardboard_box.usd`
- **Pallets**: `Props/Pallet/pallet.usd`
- **Bins**: `Props/Bins/industrial_bin.usd`

Randomly place them on shelves and floor.

#### 4. Add Robot

1. **Isaac Utils** > **URDF Importer**
2. Import your mobile manipulator URDF (or use Franka from Nucleus)
3. Position at: `X: 0, Y: 0, Z: 0` (center of warehouse)

#### 5. Configure Lighting

Follow Part 4 steps:

- 1x Dome Light (HDR environment)
- 4x Rect Lights (overhead panels)
- 1x Distant Light (skylight simulation)

#### 6. Final Touches

- Add floor material (concrete PBR)
- Add wall material (painted metal)
- Adjust camera to showcase scene
- Press Play to test physics

### Save and Version Control

```bash
# Save scene
File > Save As > warehouse_pickplace_v1.usd

# For large projects, consider Git LFS:
git lfs track "*.usd"
git add warehouse_pickplace_v1.usd
git commit -m "Add warehouse pick-and-place environment"
```

---

## Part 7: Scene Optimization for Performance

Photorealistic scenes can be computationally expensive. Optimize for real-time simulation:

### Optimization Techniques

| Technique | Impact | How to Implement |
|-----------|--------|------------------|
| **Level of Detail (LOD)** | High | Use lower-poly meshes for distant objects |
| **Texture Resolution** | Medium | Reduce texture size (e.g., 4K → 2K for far objects) |
| **Light Count** | High | Limit to &lt;10 dynamic lights per scene |
| **Shadow Map Resolution** | Medium | Edit > Preferences > Rendering > Shadow Map: 2048 |
| **Disable Ray Tracing** | Very High | For training, disable if photorealism not critical |

### Performance Checklist

Before finalizing your scene:

- [ ] FPS > 30 with robot and props loaded
- [ ] Physics simulation stable (no jittering objects)
- [ ] Shadows render correctly without artifacts
- [ ] Textures load fully (no pink/missing texture indicators)
- [ ] Scene file size < 500MB (for quick loading)

:::tip Performance Profiling
Use **Window** > **Profiler** to identify bottlenecks (GPU time, physics step time, rendering time)
:::

---

## Part 8: Hands-On Exercise

### Exercise: Create a Tabletop Manipulation Scene

**Objective**: Build a realistic kitchen countertop scene for robot manipulation training

#### Requirements

1. **Scene Elements**:
   - Countertop (1m x 0.5m x 0.9m height)
   - 5 objects: Apple, orange, mug, plate, spoon
   - Franka Emika robot arm
   - Realistic kitchen lighting

2. **Technical Specs**:
   - Ground plane with tile material
   - PBR materials on all objects (vary metallic/roughness)
   - 3 lights: 1 dome (ambient), 1 rect (ceiling), 1 distant (window)
   - Camera positioned to view tabletop from 45° angle

3. **Acceptance Criteria**:
   - Scene runs at 30+ FPS
   - Objects have realistic materials (apple is matte, spoon is metallic)
   - Lighting creates soft shadows
   - Robot can be added without performance degradation

#### Deliverables

- USD scene file: `tabletop_manipulation.usd`
- Screenshot showing final scene with lighting
- Performance stats (FPS, GPU memory usage from Profiler)

---

## Summary

In this lesson, you learned:

✅ USD scene structure and composition
✅ How to import 3D assets (OBJ, URDF, USD)
✅ Lighting techniques for indoor/outdoor realism
✅ PBR material application and texturing
✅ Building a complete warehouse environment
✅ Scene optimization for real-time performance

**Key Takeaways**:
- **USD** is Isaac Sim's native format for scene description
- **Lighting** is the most impactful factor for photorealism
- **PBR materials** require Base Color, Metallic, and Roughness tuning
- **Optimization** is essential for maintaining 30+ FPS in simulation

---

## Next Steps

In the next lesson (**Synthetic Data Generation**), you'll learn:

- How to extract camera images, depth maps, and segmentation masks
- Programmatic camera placement for data collection
- Annotating objects for object detection training
- Exporting datasets for AI model training

---

## Additional Resources

- **USD Tutorials**: https://graphics.pixar.com/usd/docs/index.html
- **PBR Texture Library**: https://polyhaven.com/ (free CC0 textures)
- **Isaac Sim Asset Library**: https://docs.omniverse.nvidia.com/isaacsim/latest/features/assets.html
- **Lighting Best Practices**: https://docs.omniverse.nvidia.com/isaacsim/latest/tutorials/gui_tutorials/tutorial_lighting.html
- **Material Editor Guide**: https://docs.omniverse.nvidia.com/materials/latest/index.html

:::info Next Lesson Preview
Synthetic data generation enables you to train AI models without expensive real-world data collection. You'll learn to generate thousands of labeled images automatically!
:::
