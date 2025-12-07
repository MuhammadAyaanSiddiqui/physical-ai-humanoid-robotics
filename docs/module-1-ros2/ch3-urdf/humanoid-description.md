# Building a Humanoid Robot URDF Model

**Module**: The Robotic Nervous System
**Chapter**: Robot Modeling (URDF)
**Estimated Time**: 3-4 hours
**Difficulty**: Intermediate

## Prerequisites

- Understanding of URDF syntax (links, joints, coordinate frames)
- Familiarity with 3D coordinates and rotations
- ROS 2 Humble installed

## Learning Objectives

By the end of this lesson, you will be able to:

- Design a hierarchical humanoid robot structure
- Create a complete URDF model with 10+ joints
- Understand humanoid kinematic chains (legs, torso, arms, head)
- Set appropriate joint limits for human-like motion
- Build modular URDF using best practices
- Test and visualize your humanoid in RViz

## Humanoid Robot Anatomy

A basic humanoid consists of:

**Kinematic Chain**:
```
                    head
                     |
        left_arm -- torso -- right_arm
                     |
              left_leg  right_leg
```

**Joint Count**:
- **Torso**: 1 joint (waist rotation)
- **Legs**: 6 joints (3 per leg: hip, knee, ankle)
- **Arms**: 4 joints (2 per arm: shoulder, elbow)
- **Head**: 1 joint (neck)
- **Total**: 12 joints (simplified humanoid)

For this lesson, we'll create a **10-joint humanoid** (excluding some degrees of freedom for simplicity).

## Design Specifications

### Link Dimensions (meters)

| Link | Length | Width | Height |
|------|--------|-------|--------|
| Torso | 0.20 | 0.30 | 0.40 |
| Upper Leg | 0.10 | 0.10 | 0.35 |
| Lower Leg | 0.08 | 0.08 | 0.30 |
| Upper Arm | 0.08 | 0.08 | 0.25 |
| Lower Arm | 0.06 | 0.06 | 0.20 |
| Head | 0.15 | 0.15 | 0.20 |

### Joint Limits (radians)

| Joint | Type | Axis | Min | Max |
|-------|------|------|-----|-----|
| Hip Pitch | Revolute | Y | -1.57 | 1.57 |
| Knee | Revolute | Y | 0.0 | 2.36 |
| Ankle | Revolute | Y | -0.78 | 0.78 |
| Shoulder Pitch | Revolute | Y | -1.57 | 1.57 |
| Elbow | Revolute | Y | 0.0 | 2.36 |
| Neck | Revolute | Z | -0.78 | 0.78 |

## Step-by-Step Construction

### Step 1: Create Base Structure

Create `simple_humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- ========================================
       BASE LINK (Root - where robot stands)
       ======================================== -->
  <link name="base_link">
    <!-- Empty link, used as reference point on ground -->
  </link>

  <!-- ========================================
       TORSO
       ======================================== -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.20 0.30 0.40"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0.5 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.20 0.30 0.40"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <inertia ixx="0.27" ixy="0.0" ixz="0.0"
               iyy="0.22" iyz="0.0"
               izz="0.13"/>
    </inertial>
  </link>

  <!-- Connect torso to base (fixed, torso is 0.7m above ground) -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.7" rpy="0 0 0"/>
  </joint>

</robot>
```

**Key Points**:
- `base_link`: Ground reference (empty link)
- `torso`: Main body, 0.7m above ground (hip height)
- Fixed joint: Torso doesn't move relative to ground (for now)

### Step 2: Add Legs

Add left and right legs with hip, knee, and ankle joints:

```xml
  <!-- ========================================
       LEFT LEG
       ======================================== -->

  <!-- Upper Leg (Thigh) -->
  <link name="left_upper_leg">
    <visual>
      <origin xyz="0 0 -0.175" rpy="0 0 0"/>  <!-- Center of geometry -->
      <geometry>
        <box size="0.10 0.10 0.35"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.175" rpy="0 0 0"/>
      <geometry>
        <box size="0.10 0.10 0.35"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.175" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0"
               iyy="0.02" iyz="0.0"
               izz="0.003"/>
    </inertial>
  </link>

  <!-- Hip Joint (connects torso to upper leg) -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <!-- Hip is at bottom left of torso -->
    <origin xyz="0 0.10 -0.20" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Pitch rotation (forward/backward) -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="5.0"/>
  </joint>

  <!-- Lower Leg (Shin) -->
  <link name="left_lower_leg">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.30"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.30"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0"
               iyy="0.008" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <!-- Knee Joint -->
  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <!-- Knee is at bottom of upper leg -->
    <origin xyz="0 0 -0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.36" effort="50" velocity="5.0"/>
  </joint>

  <!-- Foot -->
  <link name="left_foot">
    <visual>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.20 0.10 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.20 0.10 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.002" iyz="0.0"
               izz="0.002"/>
    </inertial>
  </link>

  <!-- Ankle Joint -->
  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.30" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="30" velocity="5.0"/>
  </joint>
```

**Pattern**: Hip → Upper Leg → Knee → Lower Leg → Ankle → Foot

**Repeat for Right Leg**: Change `left` to `right` and Y-offset from `0.10` to `-0.10`

### Step 3: Add Arms

```xml
  <!-- ========================================
       LEFT ARM
       ======================================== -->

  <!-- Upper Arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.25"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.25"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0"
               iyy="0.005" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <!-- Shoulder Joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <!-- Shoulder at top left of torso -->
    <origin xyz="0 0.18 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="5.0"/>
  </joint>

  <!-- Lower Arm (Forearm) -->
  <link name="left_lower_arm">
    <visual>
      <origin xyz="0 0 -0.10" rpy="0 0 0"/>
      <geometry>
        <box size="0.06 0.06 0.20"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.10" rpy="0 0 0"/>
      <geometry>
        <box size="0.06 0.06 0.20"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.10" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0"
               iyy="0.002" iyz="0.0"
               izz="0.0003"/>
    </inertial>
  </link>

  <!-- Elbow Joint -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.36" effort="30" velocity="5.0"/>
  </joint>
```

**Repeat for Right Arm**: Change `left` to `right` and Y-offset from `0.18` to `-0.18`

### Step 4: Add Head

```xml
  <!-- ========================================
       HEAD
       ======================================== -->

  <link name="head">
    <visual>
      <origin xyz="0 0 0.10" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.15 0.20"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.10" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.15 0.20"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.10" rpy="0 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0"
               iyy="0.008" iyz="0.0"
               izz="0.006"/>
    </inertial>
  </link>

  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.20" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Yaw rotation (left/right) -->
    <limit lower="-0.78" upper="0.78" effort="20" velocity="3.0"/>
  </joint>

</robot>
```

## Complete 10-Joint Humanoid Summary

**Joint List**:
1. `left_hip_joint` - Left leg hip (pitch)
2. `left_knee_joint` - Left leg knee (pitch)
3. `left_ankle_joint` - Left leg ankle (pitch)
4. `right_hip_joint` - Right leg hip (pitch)
5. `right_knee_joint` - Right leg knee (pitch)
6. `right_ankle_joint` - Right leg ankle (pitch)
7. `left_shoulder_joint` - Left arm shoulder (pitch)
8. `left_elbow_joint` - Left arm elbow (pitch)
9. `right_shoulder_joint` - Right arm shoulder (pitch)
10. `right_elbow_joint` - Right arm elbow (pitch)
11. `neck_joint` - Head rotation (yaw)

**Total**: 11 revolute joints + 1 fixed joint (base_to_torso)

## Kinematic Chains

### Leg Chain (Left)
```
torso
  └─ left_hip_joint
      └─ left_upper_leg
          └─ left_knee_joint
              └─ left_lower_leg
                  └─ left_ankle_joint
                      └─ left_foot
```

### Arm Chain (Left)
```
torso
  └─ left_shoulder_joint
      └─ left_upper_arm
          └─ left_elbow_joint
              └─ left_lower_arm
```

## Testing Your Humanoid

### Step 1: Validate URDF

```bash
check_urdf simple_humanoid.urdf
```

**Expected Output**:
```
robot name is: simple_humanoid
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child(ren)
    child(1):  torso
        child(1):  left_upper_leg
            child(1):  left_lower_leg
                child(1):  left_foot
        child(2):  right_upper_leg
            ...
        child(3):  left_upper_arm
            ...
        child(4):  right_upper_arm
            ...
        child(5):  head
```

### Step 2: Visualize in RViz

Create launch file `display_humanoid.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = 'simple_humanoid.urdf'

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
```

**Launch**:
```bash
ros2 launch display_humanoid.launch.py
```

You'll see:
- RViz with the humanoid model
- Joint State Publisher GUI with sliders for each joint

## Advanced Features

### Adding Visual Complexity with Meshes

Replace simple boxes with STL/DAE meshes:

```xml
<link name="torso">
  <visual>
    <geometry>
      <mesh filename="package://my_humanoid/meshes/torso.stl" scale="1 1 1"/>
    </geometry>
  </visual>
  <!-- Keep simple collision -->
  <collision>
    <geometry>
      <box size="0.20 0.30 0.40"/>
    </geometry>
  </collision>
</link>
```

### Adding Sensors

```xml
<!-- Camera on head -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.08 0 0.15" rpy="0 0 0"/>
</joint>
```

### Adding More Degrees of Freedom

Expand to realistic humanoid (25+ DOF):

```xml
<!-- Hip with 3 DOF (roll, pitch, yaw) -->
<joint name="left_hip_roll" type="revolute">
  <parent link="torso"/>
  <child link="left_hip_roll_link"/>
  <axis xyz="1 0 0"/>  <!-- Roll -->
  <limit lower="-0.5" upper="0.5" effort="50" velocity="3.0"/>
</joint>

<joint name="left_hip_pitch" type="revolute">
  <parent link="left_hip_roll_link"/>
  <child link="left_hip_pitch_link"/>
  <axis xyz="0 1 0"/>  <!-- Pitch -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="5.0"/>
</joint>

<joint name="left_hip_yaw" type="revolute">
  <parent link="left_hip_pitch_link"/>
  <child link="left_upper_leg"/>
  <axis xyz="0 0 1"/>  <!-- Yaw -->
  <limit lower="-0.5" upper="0.5" effort="50" velocity="3.0"/>
</joint>
```

## Best Practices

### 1. Symmetry

Keep left and right sides symmetric:
```xml
<!-- Use same dimensions for left and right -->
<!-- Only change xyz origin positions -->
```

### 2. Realistic Masses

Total humanoid mass should be reasonable:
- Lightweight humanoid: 15-30 kg
- Full-size humanoid: 50-80 kg

**Mass Distribution**:
- Torso: ~40% of total
- Legs: ~30% of total
- Arms: ~20% of total
- Head: ~10% of total

### 3. Joint Limits

Match human range of motion:
- Hip: -90° to +90° (≈ -1.57 to 1.57 rad)
- Knee: 0° to +135° (0 to 2.36 rad)
- Elbow: 0° to +135° (0 to 2.36 rad)

### 4. Link Origins

Place visual/collision origins at geometric center:
```xml
<!-- If box is 0.4m tall, center is 0.2m below joint -->
<origin xyz="0 0 -0.20" rpy="0 0 0"/>
```

## Common Issues and Fixes

### Issue 1: Robot Falls Apart in Simulation

**Cause**: Joints overlap or bad inertias

**Fix**:
- Ensure joint origins place child links correctly
- Set realistic inertia values
- Check collision shapes don't intersect

### Issue 2: Joints Move Backwards

**Cause**: Axis direction inverted

**Fix**:
```xml
<!-- If joint moves wrong direction, flip axis -->
<axis xyz="0 -1 0"/>  <!-- Instead of "0 1 0" -->
```

### Issue 3: Robot Explodes in Gazebo

**Cause**: Zero or very small masses/inertias

**Fix**:
```xml
<!-- Set minimum mass -->
<mass value="0.1"/>  <!-- At least 0.1 kg -->

<!-- Set realistic inertias (not zero) -->
<inertia ixx="0.001" ... />
```

## Practice Exercises

### Exercise 1: Add Hands

Add simple hand links to the end of each arm with fixed joints.

### Exercise 2: Add Hip Roll

Modify the hip to have 2 DOF (pitch and roll) by adding an intermediate link.

### Exercise 3: Balance Pose

Using joint_state_publisher_gui, create a "balanced standing" pose and note the joint angles.

### Exercise 4: Calculate Total Mass

Sum up all link masses and verify the total is reasonable for a humanoid.

## Key Takeaways

- Humanoid robots have **hierarchical kinematic chains**
- Each limb follows pattern: **proximal joint → link → distal joint → link**
- **Joint limits** should match human range of motion
- **Symmetry** simplifies design (left/right arms and legs)
- **Inertial properties** are critical for simulation
- Always **validate** with `check_urdf` before testing

## What's Next?

Now that you have a humanoid URDF:

- **Next Lesson**: [RViz Visualization](./rviz-visualization.md) - Visualize and interact with your model
- **Related**: [Joint Controllers](./joint-controllers.md) - Control joint positions

## Further Reading

- [Example Humanoid URDFs](https://github.com/ros/robot_model_tutorials)
- [Boston Dynamics Atlas URDF](https://github.com/RobotLocomotion/drake/tree/master/examples/atlas)
- [NASA Valkyrie URDF](https://github.com/NASA-JSC-Robotics/val_description)

---

**Checkpoint**: You can now create a complete humanoid robot URDF model!
