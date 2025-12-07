# Humanoid Robot URDF Description

## Introduction

Building a humanoid robot model in URDF requires careful planning of the kinematic chain, joint configurations, and coordinate frames. This lesson guides you through creating a complete humanoid robot description with head, torso, arms, and legs - suitable for simulation and control.

## Learning Objectives

By the end of this lesson, you will be able to:
- Design a humanoid robot kinematic structure
- Create a multi-DOF humanoid URDF model
- Configure realistic joint limits for humanoid motion
- Organize complex URDF files with macros
- Add sensors to humanoid robots
- Test and validate humanoid models

## Prerequisites

- Understanding of URDF syntax
- Knowledge of robot kinematics
- Familiarity with 3D coordinate systems

---

## Humanoid Robot Structure

A typical humanoid robot consists of:

```
                head
                 |
            neck_joint
                 |
               torso
              /  |  \
  left_shoulder | right_shoulder
            /   |   \
    left_arm  spine  right_arm
         |           |
  left_elbow    right_elbow
         |           |
   left_hand     right_hand

        torso (continued)
         /  \
   left_hip  right_hip
      /        \
 left_thigh  right_thigh
     |          |
 left_knee   right_knee
     |          |
 left_shin   right_shin
     |          |
 left_ankle  right_ankle
     |          |
 left_foot   right_foot
```

### Joint Count

A basic humanoid typically has:
- **Head**: 2 DOF (pan, tilt)
- **Arms**: 6 DOF per arm (shoulder: 3, elbow: 1, wrist: 2)
- **Legs**: 6 DOF per leg (hip: 3, knee: 1, ankle: 2)
- **Total**: 2 + 12 + 12 = **26 DOF minimum**

For this lesson, we'll create a simplified 10-DOF humanoid.

---

## Simple 10-DOF Humanoid URDF

### Link Definitions

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- ============================================ -->
  <!-- MATERIALS -->
  <!-- ============================================ -->

  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>

  <material name="red">
    <color rgba="0.8 0 0 1"/>
  </material>

  <material name="green">
    <color rgba="0 0.8 0 1"/>
  </material>

  <!-- ============================================ -->
  <!-- TORSO -->
  <!-- ============================================ -->

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <inertia ixx="0.233" ixy="0.0" ixz="0.0"
               iyy="0.317" iyz="0.0"
               izz="0.133"/>
    </inertial>
  </link>

  <!-- ============================================ -->
  <!-- HEAD -->
  <!-- ============================================ -->

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0"
               iyy="0.008" iyz="0.0"
               izz="0.008"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="10.0" velocity="2.0" lower="-1.57" upper="1.57"/>
  </joint>

  <!-- ============================================ -->
  <!-- LEFT ARM -->
  <!-- ============================================ -->

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0"
               iyy="0.008" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0.15 0.35" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="30.0" velocity="2.0" lower="-1.57" upper="1.57"/>
  </joint>

  <link name="left_forearm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0"
               iyy="0.003" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="20.0" velocity="2.0" lower="0.0" upper="2.36"/>
  </joint>

  <!-- ============================================ -->
  <!-- RIGHT ARM (Mirror of left) -->
  <!-- ============================================ -->

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0"
               iyy="0.008" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 -0.15 0.35" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="30.0" velocity="2.0" lower="-1.57" upper="1.57"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0"
               iyy="0.003" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="20.0" velocity="2.0" lower="0.0" upper="2.36"/>
  </joint>

  <!-- ============================================ -->
  <!-- LEFT LEG -->
  <!-- ============================================ -->

  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.023" ixy="0.0" ixz="0.0"
               iyy="0.023" iyz="0.0"
               izz="0.004"/>
    </inertial>
  </link>

  <joint name="left_hip" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh"/>
    <origin xyz="0 0.08 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="50.0" velocity="2.0" lower="-1.57" upper="0.785"/>
  </joint>

  <link name="left_shin">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0"
               iyy="0.015" iyz="0.0"
               izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_knee" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="50.0" velocity="2.0" lower="0.0" upper="2.36"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <inertia ixx="0.0004" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_ankle" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="30.0" velocity="2.0" lower="-0.785" upper="0.785"/>
  </joint>

  <!-- ============================================ -->
  <!-- RIGHT LEG (Mirror of left) -->
  <!-- ============================================ -->

  <link name="right_thigh">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.023" ixy="0.0" ixz="0.0"
               iyy="0.023" iyz="0.0"
               izz="0.004"/>
    </inertial>
  </link>

  <joint name="right_hip" type="revolute">
    <parent link="base_link"/>
    <child link="right_thigh"/>
    <origin xyz="0 -0.08 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="50.0" velocity="2.0" lower="-1.57" upper="0.785"/>
  </joint>

  <link name="right_shin">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0"
               iyy="0.015" iyz="0.0"
               izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_knee" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="50.0" velocity="2.0" lower="0.0" upper="2.36"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <inertia ixx="0.0004" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_ankle" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="30.0" velocity="2.0" lower="-0.785" upper="0.785"/>
  </joint>

</robot>
```

This URDF defines a 10-DOF humanoid with:
- 1 neck joint (head pan)
- 2 shoulders
- 2 elbows
- 2 hips
- 2 knees
- 2 ankles

---

## Realistic Joint Limits

| Joint | Lower (rad) | Upper (rad) | Lower (deg) | Upper (deg) | Notes |
|-------|-------------|-------------|-------------|-------------|-------|
| Neck Pan | -1.57 | 1.57 | -90° | 90° | Head rotation |
| Shoulder | -1.57 | 1.57 | -90° | 90° | Forward/backward swing |
| Elbow | 0.0 | 2.36 | 0° | 135° | Flexion only |
| Hip | -1.57 | 0.785 | -90° | 45° | Swing and lift |
| Knee | 0.0 | 2.36 | 0° | 135° | Flexion only |
| Ankle | -0.785 | 0.785 | -45° | 45° | Tilt |

---

## Testing the Humanoid URDF

### 1. Validate Syntax

```bash
check_urdf simple_humanoid.urdf
```

Expected output:
```
robot name is: simple_humanoid
---------- Successfully Parsed XML ---------------
root Link: base_link has 5 child(ren)
    child(1):  head
    child(2):  left_upper_arm
        child(1):  left_forearm
    child(3):  right_upper_arm
        child(1):  right_forearm
    child(4):  left_thigh
        child(1):  left_shin
            child(1):  left_foot
    child(5):  right_thigh
        child(1):  right_shin
            child(1):  right_foot
```

### 2. Visualize Structure

```bash
urdf_to_graphiz simple_humanoid.urdf
```

This generates a PDF showing the kinematic tree.

### 3. View in RViz

```bash
# Launch RViz with URDF display
ros2 launch urdf_tutorial display.launch.py model:=simple_humanoid.urdf
```

---

## Adding Sensors

### Camera on Head

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.08 0.03"/>
    </geometry>
    <material name="white"/>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.08 0.03"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0" ixz="0"
             iyy="0.0001" iyz="0"
             izz="0.0001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.08 0 0.1" rpy="0 0 0"/>
</joint>
```

### IMU on Torso

```xml
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
    <material name="red"/>
  </visual>
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.00001" ixy="0" ixz="0"
             iyy="0.00001" iyz="0"
             izz="0.00001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
</joint>
```

---

## Best Practices for Humanoid URDFs

### 1. Consistent Naming Convention

```
{side}_{segment}_{type}

Examples:
- left_shoulder_joint
- right_knee_joint
- head_camera_link
```

### 2. Proper Z-Up Convention

Robot should stand upright along Z-axis:
- X: forward
- Y: left
- Z: up

### 3. Symmetric Left/Right

Ensure left and right sides are mirrored properly:
```xml
<!-- Left arm: +Y offset -->
<origin xyz="0 0.15 0.35" rpy="0 0 0"/>

<!-- Right arm: -Y offset -->
<origin xyz="0 -0.15 0.35" rpy="0 0 0"/>
```

### 4. Realistic Mass Distribution

Total robot mass should be reasonable:
- Humanoid (1m tall): ~20-30 kg
- Head: ~10% of total
- Torso: ~40% of total
- Arms: ~20% of total
- Legs: ~30% of total

---

## Summary

You've learned to:
- Design humanoid kinematic structures
- Create complete multi-DOF URDF models
- Set realistic joint limits
- Add sensors to robots
- Validate humanoid URDFs

**Key Takeaways:**
- Plan kinematic tree before coding
- Use consistent naming and frames
- Include realistic masses and inertias
- Test with check_urdf and RViz
- Start simple, add complexity incrementally

---

## Next Steps

- [RViz Visualization](./rviz-visualization.md) - Visualize and test your humanoid
- [Joint Controllers](./joint-controllers.md) - Control humanoid motion
- [Complete URDF Example](/code/ros2-packages/simple_humanoid.urdf)

---

## Additional Resources

- [Humanoid Robotics Resources](https://github.com/topics/humanoid-robot)
- [URDF Examples Repository](https://github.com/ros/urdf_tutorial)
- [Inertia Calculator Tool](http://wiki.ros.org/inertial_parameters_calculator)
