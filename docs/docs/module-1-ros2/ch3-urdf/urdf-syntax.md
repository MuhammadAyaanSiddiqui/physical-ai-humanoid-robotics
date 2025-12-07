# URDF Syntax and Robot Modeling

## Introduction

The Unified Robot Description Format (URDF) is an XML-based format for describing robot structures in ROS 2. URDF files define the kinematic and dynamic properties of robots, including links (rigid bodies), joints (connections), sensors, and visual/collision geometry. Understanding URDF is essential for simulating and controlling robots.

## Learning Objectives

By the end of this lesson, you will be able to:
- Understand URDF XML structure and syntax
- Define robot links with visual and collision geometry
- Create different types of joints (revolute, prismatic, fixed, continuous)
- Specify inertial properties and collision models
- Use materials and colors for visualization
- Validate and debug URDF files

## Prerequisites

- Basic XML knowledge
- Understanding of 3D coordinate systems
- Familiarity with robot kinematics concepts

---

## URDF File Structure

A URDF file is an XML document with a `<robot>` root element:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <!-- Visual geometry -->
    <!-- Collision geometry -->
    <!-- Inertial properties -->
  </link>

  <!-- Joints connect links -->
  <joint name="joint1" type="revolute">
    <!-- Parent and child links -->
    <!-- Joint axis and limits -->
  </joint>
</robot>
```

---

## Links - Rigid Bodies

Links represent rigid bodies in your robot. Each link can have:
- **Visual**: How the link appears (for rendering)
- **Collision**: Simplified geometry for collision detection
- **Inertial**: Mass and inertia properties (for physics simulation)

### Basic Link

```xml
<link name="base_link">
  <!-- Visual representation -->
  <visual>
    <geometry>
      <box size="0.5 0.3 0.2"/>
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <!-- Collision model -->
  <collision>
    <geometry>
      <box size="0.5 0.3 0.2"/>
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </collision>

  <!-- Inertial properties -->
  <inertial>
    <mass value="10.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0"
             iyy="1.0" iyz="0.0"
             izz="1.0"/>
  </inertial>
</link>
```

### Geometry Types

#### Box
```xml
<geometry>
  <box size="length width height"/>
  <!-- Example: <box size="1.0 0.5 0.3"/> -->
</geometry>
```

#### Cylinder
```xml
<geometry>
  <cylinder radius="0.1" length="0.5"/>
</geometry>
```

#### Sphere
```xml
<geometry>
  <sphere radius="0.15"/>
</geometry>
```

#### Mesh (Custom 3D Models)
```xml
<geometry>
  <mesh filename="package://my_robot_description/meshes/arm.stl" scale="1.0 1.0 1.0"/>
</geometry>
```

**Supported mesh formats:**
- STL (Stereolithography)
- DAE (COLLADA)
- OBJ (Wavefront)

---

## Origins and Transformations

The `<origin>` tag specifies position and orientation:

```xml
<origin xyz="x y z" rpy="roll pitch yaw"/>
```

- **xyz**: Translation in meters [x, y, z]
- **rpy**: Rotation in radians [roll, pitch, yaw]
  - Roll: Rotation around X-axis
  - Pitch: Rotation around Y-axis
  - Yaw: Rotation around Z-axis

### Examples

```xml
<!-- No offset -->
<origin xyz="0 0 0" rpy="0 0 0"/>

<!-- Offset 0.5m up in Z -->
<origin xyz="0 0 0.5" rpy="0 0 0"/>

<!-- Rotated 90 degrees around Z-axis -->
<origin xyz="0 0 0" rpy="0 0 1.5708"/>

<!-- Combined translation and rotation -->
<origin xyz="0.2 0.1 0.3" rpy="0 0.785 0"/>
```

---

## Materials and Colors

### Defining Materials

```xml
<material name="red">
  <color rgba="1 0 0 1"/>
  <!-- RGBA: Red Green Blue Alpha (0-1 range) -->
</material>

<material name="blue">
  <color rgba="0 0 1 1"/>
</material>

<material name="gray">
  <color rgba="0.5 0.5 0.5 1"/>
</material>
```

### Using Materials

```xml
<visual>
  <geometry>
    <cylinder radius="0.05" length="0.3"/>
  </geometry>
  <material name="red"/>
</visual>
```

### Material with Texture

```xml
<material name="wood">
  <texture filename="package://my_robot_description/textures/wood.png"/>
</material>
```

---

## Joints - Connecting Links

Joints define how links are connected and how they can move relative to each other.

### Joint Types

| Type | Description | Degrees of Freedom |
|------|-------------|-------------------|
| `fixed` | No movement | 0 |
| `revolute` | Rotation with limits | 1 |
| `continuous` | Unlimited rotation | 1 |
| `prismatic` | Linear sliding | 1 |
| `floating` | 6-DOF free movement | 6 |
| `planar` | 2D plane movement | 2 |

### Fixed Joint

Rigidly connects two links:

```xml
<joint name="base_to_sensor" type="fixed">
  <parent link="base_link"/>
  <child link="sensor_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>
```

### Revolute Joint (Rotation with Limits)

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>

  <!-- Joint location relative to parent -->
  <origin xyz="0 0.2 0.5" rpy="0 0 0"/>

  <!-- Rotation axis (in joint frame) -->
  <axis xyz="0 1 0"/>

  <!-- Joint limits -->
  <limit effort="100.0" velocity="2.0" lower="-1.57" upper="1.57"/>

  <!-- Joint dynamics (optional) -->
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

**Explanation:**
- `effort`: Maximum torque (N⋅m)
- `velocity`: Maximum angular velocity (rad/s)
- `lower`: Minimum angle (radians)
- `upper`: Maximum angle (radians)
- `axis`: Rotation axis in joint frame

### Continuous Joint (Unlimited Rotation)

```xml
<joint name="wheel_joint" type="continuous">
  <parent link="chassis"/>
  <child link="wheel"/>
  <origin xyz="0.3 0 -0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit effort="50.0" velocity="10.0"/>
</joint>
```

### Prismatic Joint (Linear Sliding)

```xml
<joint name="linear_actuator" type="prismatic">
  <parent link="base"/>
  <child link="slider"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit effort="500.0" velocity="1.0" lower="0.0" upper="0.5"/>
</joint>
```

---

## Inertial Properties

Critical for physics simulation:

```xml
<inertial>
  <!-- Mass in kg -->
  <mass value="5.0"/>

  <!-- Center of mass offset -->
  <origin xyz="0 0 0" rpy="0 0 0"/>

  <!-- Inertia tensor (kg⋅m²) -->
  <inertia ixx="0.1" ixy="0.0" ixz="0.0"
           iyy="0.1" iyz="0.0"
           izz="0.1"/>
</inertial>
```

### Calculating Inertia

**Box (dimensions: x, y, z):**
```
ixx = (1/12) * m * (y² + z²)
iyy = (1/12) * m * (x² + z²)
izz = (1/12) * m * (x² + y²)
```

**Cylinder (radius: r, height: h, axis: z):**
```
ixx = (1/12) * m * (3*r² + h²)
iyy = (1/12) * m * (3*r² + h²)
izz = (1/2) * m * r²
```

**Sphere (radius: r):**
```
ixx = iyy = izz = (2/5) * m * r²
```

### Example: Box Inertia

```xml
<!-- Box: 1m x 0.5m x 0.3m, mass: 10kg -->
<inertial>
  <mass value="10.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.283" ixy="0.0" ixz="0.0"
           iyy="0.900" iyz="0.0"
           izz="1.042"/>
  <!-- ixx = (1/12)*10*(0.5² + 0.3²) = 0.283 -->
  <!-- iyy = (1/12)*10*(1.0² + 0.3²) = 0.900 -->
  <!-- izz = (1/12)*10*(1.0² + 0.5²) = 1.042 -->
</inertial>
```

---

## Complete Simple Robot Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- Materials -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>

  <material name="red">
    <color rgba="0.8 0 0 1"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0"
               iyy="0.07" iyz="0.0"
               izz="0.1"/>
    </inertial>
  </link>

  <!-- Arm Link -->
  <link name="arm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.5"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.5"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <inertia ixx="0.021" ixy="0.0" ixz="0.0"
               iyy="0.021" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="50.0" velocity="2.0" lower="-1.57" upper="1.57"/>
  </joint>

</robot>
```

---

## URDF Best Practices

### 1. Use Consistent Units
- **Length**: meters
- **Mass**: kilograms
- **Angles**: radians
- **Force/Torque**: Newtons/Newton-meters

### 2. Separate Visual and Collision Geometry

```xml
<!-- High-detail visual mesh -->
<visual>
  <geometry>
    <mesh filename="package://my_robot/meshes/arm_detailed.stl"/>
  </geometry>
</visual>

<!-- Simplified collision geometry for performance -->
<collision>
  <geometry>
    <cylinder radius="0.05" length="0.3"/>
  </geometry>
</collision>
```

### 3. Validate Coordinate Frames
- Ensure joint axes are correct
- Check parent-child relationships
- Verify origin transforms

### 4. Include Inertial Properties
Even if approximate, include mass and inertia for physics simulation.

---

## Validating URDF Files

### Using check_urdf

```bash
# Install URDF tools
sudo apt install liburdfdom-tools

# Check URDF syntax
check_urdf my_robot.urdf
```

**Output (success):**
```
robot name is: my_robot
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child(ren)
    child(1):  arm
```

### Using urdf_to_graphiz

Visualize the link-joint tree:

```bash
urdf_to_graphiz my_robot.urdf
# Generates my_robot.pdf with robot structure diagram
```

### Common URDF Errors

**1. Missing parent or child:**
```xml
<!-- Wrong: missing parent -->
<joint name="joint1" type="revolute">
  <child link="arm"/>
  ...
</joint>

<!-- Correct -->
<joint name="joint1" type="revolute">
  <parent link="base_link"/>
  <child link="arm"/>
  ...
</joint>
```

**2. Invalid joint limits:**
```xml
<!-- Wrong: lower > upper -->
<limit lower="1.57" upper="-1.57" effort="100" velocity="2"/>

<!-- Correct -->
<limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
```

**3. Missing required attributes:**
```xml
<!-- Wrong: missing effort and velocity -->
<limit lower="-1.57" upper="1.57"/>

<!-- Correct -->
<limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
```

---

## Hands-On Exercise

### Exercise 1: Create a Simple Arm

Create a URDF file for a 2-DOF robotic arm:
- Base link (box: 0.2 x 0.2 x 0.1 m)
- Upper arm (cylinder: radius 0.05m, length 0.4m)
- Forearm (cylinder: radius 0.04m, length 0.3m)
- Shoulder joint (revolute, ±90°)
- Elbow joint (revolute, 0° to 135°)

### Exercise 2: Add Inertial Properties

Calculate and add proper inertial properties for each link in Exercise 1.

---

## Summary

You've learned to:
- Understand URDF XML structure
- Define links with geometry and materials
- Create different joint types
- Specify inertial properties
- Validate URDF files

**Key Takeaways:**
- URDF describes robot kinematics and dynamics
- Links are rigid bodies with visual/collision/inertial
- Joints connect links with motion constraints
- Always validate URDF syntax
- Use consistent units (meters, kg, radians)

---

## Next Steps

- [Humanoid Robot Description](./humanoid-description.md) - Build a complete humanoid URDF
- [RViz Visualization](./rviz-visualization.md) - Visualize your URDF
- [Joint Controllers](./joint-controllers.md) - Control robot joints

---

## Additional Resources

- [URDF XML Specification](http://wiki.ros.org/urdf/XML)
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter)
- [Complete URDF Example](/code/ros2-packages/simple_humanoid.urdf)
