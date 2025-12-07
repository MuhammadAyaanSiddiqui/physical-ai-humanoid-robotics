# URDF Syntax and Structure

**Module**: The Robotic Nervous System
**Chapter**: Robot Modeling (URDF)
**Estimated Time**: 2-3 hours
**Difficulty**: Intermediate

## Prerequisites

- Understanding of ROS 2 nodes and topics
- Basic knowledge of 3D coordinates and transformations
- XML/HTML syntax familiarity

## Learning Objectives

By the end of this lesson, you will be able to:

- Understand what URDF is and its role in robotics
- Create URDF files describing robot structures
- Define links (rigid bodies) and joints (connections)
- Specify visual, collision, and inertial properties
- Use coordinate frames and transformations
- Validate and debug URDF files

## What is URDF?

**URDF** (Unified Robot Description Format) is an XML-based format for describing:
- Robot **geometry** (shapes, sizes)
- Robot **kinematics** (how parts connect and move)
- Robot **dynamics** (masses, inertias)
- Visual appearance and collision properties

### Why URDF?

- **Standard Format**: Works across ROS 2 tools (RViz, Gazebo, MoveIt)
- **Visualization**: See your robot in 3D before building
- **Simulation**: Test algorithms in physics simulators
- **Motion Planning**: Plan trajectories with accurate kinematics
- **Documentation**: Visual specification of robot structure

### URDF in the ROS 2 Ecosystem

```
URDF File
    ↓
RViz (Visualization)
Gazebo (Simulation)
robot_state_publisher (TF frames)
MoveIt (Motion Planning)
```

## Basic URDF Structure

### Minimal Example

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </visual>
  </link>

</robot>
```

**Structure**:
- `<?xml version="1.0"?>`: XML header (required)
- `<robot name="...">`: Root element with robot name
- `<link>`: Rigid body component
- `<visual>`: How link appears visually

## Links - Rigid Bodies

**Links** represent rigid parts of the robot (base, wheels, arms, sensors).

### Link Anatomy

```xml
<link name="link_name">
  <visual>      <!-- How it looks -->
  <collision>   <!-- Collision shape for physics -->
  <inertial>    <!-- Mass and inertia for dynamics -->
</link>
```

### Visual Element

Defines appearance in visualization tools (RViz, Gazebo GUI):

```xml
<link name="chassis">
  <visual>
    <!-- Origin: position and orientation relative to link frame -->
    <origin xyz="0 0 0.05" rpy="0 0 0"/>

    <!-- Geometry: shape of the visual -->
    <geometry>
      <box size="0.6 0.4 0.1"/>  <!-- Length Width Height in meters -->
    </geometry>

    <!-- Material: color (optional) -->
    <material name="blue">
      <color rgba="0 0 1 1"/>  <!-- Red Green Blue Alpha -->
    </material>
  </visual>
</link>
```

**Geometry Types**:

**1. Box**:
```xml
<geometry>
  <box size="length width height"/>  <!-- meters -->
</geometry>
```

**2. Cylinder**:
```xml
<geometry>
  <cylinder radius="0.05" length="0.3"/>
</geometry>
```

**3. Sphere**:
```xml
<geometry>
  <sphere radius="0.1"/>
</geometry>
```

**4. Mesh** (from STL/DAE file):
```xml
<geometry>
  <mesh filename="package://my_robot/meshes/chassis.stl" scale="1 1 1"/>
</geometry>
```

### Collision Element

Defines shape for collision detection (usually simpler than visual):

```xml
<link name="wheel">
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
  </collision>
</link>
```

**Why separate collision?**
- Visual meshes can be complex (thousands of polygons)
- Collision shapes should be simple (spheres, boxes) for fast computation
- Collision is often approximated for performance

### Inertial Element

Defines mass distribution for physics simulation:

```xml
<link name="base">
  <inertial>
    <!-- Center of mass relative to link origin -->
    <origin xyz="0 0 0" rpy="0 0 0"/>

    <!-- Mass in kilograms -->
    <mass value="10.0"/>

    <!-- Inertia matrix (kg⋅m²) -->
    <inertia ixx="0.1" ixy="0.0" ixz="0.0"
             iyy="0.1" iyz="0.0"
             izz="0.1"/>
  </inertial>
</link>
```

**Inertia Matrix** (for common shapes):

**Box** (mass m, dimensions a×b×c):
```
ixx = m/12 * (b² + c²)
iyy = m/12 * (a² + c²)
izz = m/12 * (a² + b²)
ixy = ixz = iyz = 0
```

**Cylinder** (mass m, radius r, length h, about center):
```
ixx = iyy = m/12 * (3r² + h²)
izz = m/2 * r²
ixy = ixz = iyz = 0
```

**Sphere** (mass m, radius r):
```
ixx = iyy = izz = 2/5 * m * r²
ixy = ixz = iyz = 0
```

## Joints - Connections Between Links

**Joints** connect links and define how they move relative to each other.

### Joint Types

| Type | Description | Degrees of Freedom |
|------|-------------|-------------------|
| `fixed` | Rigidly attached, no movement | 0 |
| `revolute` | Rotates around axis (with limits) | 1 (rotation) |
| `continuous` | Rotates around axis (no limits) | 1 (rotation) |
| `prismatic` | Slides along axis | 1 (translation) |
| `floating` | Free in 3D space | 6 (3 trans + 3 rot) |
| `planar` | Moves in a plane | 3 (2 trans + 1 rot) |

### Joint Anatomy

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
</joint>
```

**Elements**:
- `parent`: Link this joint attaches to
- `child`: Link that moves relative to parent
- `origin`: Position/orientation of joint in parent's frame
- `axis`: Axis of rotation or translation (unit vector)
- `limit`: Joint limits (for revolute/prismatic)

### Fixed Joint

Permanently connects two links (no motion):

```xml
<joint name="sensor_mount" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.2 0 0.3" rpy="0 0 0"/>
</joint>
```

### Revolute Joint (Rotates with Limits)

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>

  <!-- Joint is 0.3m above torso center -->
  <origin xyz="0 0 0.3" rpy="0 0 0"/>

  <!-- Rotates around Y-axis (pitch) -->
  <axis xyz="0 1 0"/>

  <!-- Limits: -90° to +90°, max torque 50 Nm, max speed 1 rad/s -->
  <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
</joint>
```

### Continuous Joint (Rotates Infinitely)

For wheels or rotating sensors:

```xml
<joint name="wheel_joint" type="continuous">
  <parent link="chassis"/>
  <child link="wheel"/>
  <origin xyz="0.2 0.15 0" rpy="-1.57 0 0"/>  <!-- Wheel rotates around X -->
  <axis xyz="1 0 0"/>
</joint>
```

**Note**: No `limit` element for continuous joints.

### Prismatic Joint (Slides)

For linear actuators, elevators:

```xml
<joint name="lift_joint" type="prismatic">
  <parent link="base"/>
  <child link="platform"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Moves up/down -->
  <limit lower="0" upper="0.5" effort="100" velocity="0.2"/>
</joint>
```

## Coordinate Frames and Transformations

### Link Frames

Each link has its own coordinate frame:
- **Origin**: Typically at the center or a mechanical reference point
- **X-axis**: Usually forward
- **Y-axis**: Usually left
- **Z-axis**: Usually up

### Joint Origin

The `<origin>` in a joint specifies:
- Where the **child link frame** is relative to the **parent link frame**
- Position: `xyz` in meters
- Orientation: `rpy` (roll, pitch, yaw) in radians

**Example**:
```xml
<joint name="arm_joint" type="revolute">
  <parent link="base"/>
  <child link="arm"/>
  <!-- Child link frame is 0.5m forward, 0.2m up from parent -->
  <origin xyz="0.5 0 0.2" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

### Roll, Pitch, Yaw (rpy)

**RPY** (intrinsic rotations):
1. **Roll**: Rotation around X-axis
2. **Pitch**: Rotation around Y-axis (after roll)
3. **Yaw**: Rotation around Z-axis (after pitch)

**Example**:
```xml
<!-- Rotate 90° around Y-axis (pitch up) -->
<origin xyz="0 0 0" rpy="0 1.57 0"/>

<!-- Rotate 45° around Z-axis (yaw left) -->
<origin xyz="0 0 0" rpy="0 0 0.785"/>
```

**Common Rotations**:
- Wheel horizontal: `rpy="1.57 0 0"` (90° roll)
- Camera facing forward: `rpy="0 0 0"` (identity)
- Camera facing down: `rpy="0 1.57 0"` (90° pitch)

## Complete Example: Simple Robot

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- ========== BASE LINK ========== -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.6 0.4 0.1"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.15" iyz="0.0"
               izz="0.2"/>
    </inertial>
  </link>

  <!-- ========== WHEEL LINKS ========== -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.002"/>
    </inertial>
  </link>

  <link name="wheel_right">
    <!-- Same as wheel_left -->
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- ========== WHEEL JOINTS ========== -->
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <!-- Left wheel is 0.2m to the left, at same height as base -->
    <origin xyz="0 0.2 0" rpy="-1.57 0 0"/>
    <axis xyz="1 0 0"/>
  </joint>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <!-- Right wheel is 0.2m to the right -->
    <origin xyz="0 -0.2 0" rpy="-1.57 0 0"/>
    <axis xyz="1 0 0"/>
  </joint>

  <!-- ========== SENSOR LINK ========== -->
  <link name="lidar">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.08"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar"/>
    <!-- LiDAR is 0.2m above base center -->
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

</robot>
```

## Validation and Debugging

### Check URDF Syntax

```bash
check_urdf my_robot.urdf
```

**Output** (if valid):
```
robot name is: simple_robot
---------- Successfully Parsed XML ---------------
root Link: base_link has 3 child(ren)
    child(1):  wheel_left
    child(2):  wheel_right
    child(3):  lidar
```

### Visualize Robot Tree

```bash
urdf_to_graphiz my_robot.urdf
```

Creates a PDF showing link-joint relationships.

### View in RViz

```bash
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```

Opens RViz with robot model and joint sliders.

## Common URDF Patterns

### Pattern 1: Macro for Repeated Elements

Use Xacro (XML macros) to avoid repetition:

**Xacro Example**:
```xml
<xacro:macro name="wheel" params="prefix xyz">
  <link name="${prefix}_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="${prefix}_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="${prefix}_wheel"/>
    <origin xyz="${xyz}" rpy="-1.57 0 0"/>
    <axis xyz="1 0 0"/>
  </joint>
</xacro:macro>

<!-- Use macro -->
<xacro:wheel prefix="left" xyz="0 0.2 0"/>
<xacro:wheel prefix="right" xyz="0 -0.2 0"/>
```

### Pattern 2: Separate Visual and Collision

```xml
<link name="complex_part">
  <!-- High-detail visual mesh -->
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/part.stl"/>
    </geometry>
  </visual>

  <!-- Simple collision approximation -->
  <collision>
    <geometry>
      <box size="0.1 0.1 0.2"/>  <!-- Much faster than mesh collision -->
    </geometry>
  </collision>
</link>
```

### Pattern 3: Dummy Links for Sensors

Sensors need their own frames:

```xml
<link name="camera_link">
  <!-- Small visual marker -->
  <visual>
    <geometry>
      <box size="0.01 0.05 0.02"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.1" rpy="0 0 0"/>
</joint>
```

## Best Practices

### 1. Use Meaningful Names

```xml
<!-- Good -->
<link name="left_wheel"/>
<joint name="left_wheel_joint"/>

<!-- Bad -->
<link name="link1"/>
<joint name="joint1"/>
```

### 2. Set Proper Inertias

```xml
<!-- Good: Realistic values -->
<inertial>
  <mass value="2.5"/>
  <inertia ixx="0.01" ... />
</inertial>

<!-- Bad: Zeros or unrealistic values -->
<inertial>
  <mass value="0.0"/>  <!-- Causes simulation issues! -->
</inertial>
```

### 3. Keep Visual Simple for Large Models

```xml
<!-- Good: Simple visual, detailed mesh separate -->
<visual>
  <geometry>
    <box size="0.1 0.1 0.1"/>  <!-- Fast to render -->
  </geometry>
</visual>

<!-- Only use detailed mesh if necessary -->
<visual>
  <geometry>
    <mesh filename="package://my_robot/meshes/detailed.dae"/>
  </geometry>
</visual>
```

### 4. Use Package Paths

```xml
<!-- Good: Portable across systems -->
<mesh filename="package://my_robot_description/meshes/base.stl"/>

<!-- Bad: Absolute path (breaks on other computers) -->
<mesh filename="/home/user/catkin_ws/src/my_robot/meshes/base.stl"/>
```

## Troubleshooting

### Issue 1: URDF Fails to Parse

**Error**: `Error parsing XML`

**Check**:
- XML syntax (matching tags, quotes)
- Required elements present
- Valid numbers (no text in numeric fields)

### Issue 2: Robot Explodes in Gazebo

**Cause**: Bad inertias or collision shapes overlapping

**Fix**:
- Set realistic inertia values
- Ensure collision shapes don't intersect at rest
- Check joint limits

### Issue 3: Links Not Connected

**Error**: `link [X] is not connected to the root`

**Fix**:
- Every link must connect to root via joints
- Check parent/child names match link names exactly

## Key Takeaways

- **URDF** is XML-based format for robot description
- **Links** are rigid bodies with visual, collision, inertial properties
- **Joints** connect links and define motion
- **Coordinate frames** use xyz (position) and rpy (orientation)
- Use `check_urdf` to validate syntax
- Keep collision simple, visual can be detailed

## What's Next?

Now that you understand URDF syntax:

- **Next Lesson**: [Humanoid Robot Description](./humanoid-description.md) - Build a complete humanoid model
- **Related**: [RViz Visualization](./rviz-visualization.md)

## Further Reading

- [Official URDF Specification](http://wiki.ros.org/urdf/XML)
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Xacro Documentation](http://wiki.ros.org/xacro)

---

**Checkpoint**: You can now create and understand URDF robot descriptions!
