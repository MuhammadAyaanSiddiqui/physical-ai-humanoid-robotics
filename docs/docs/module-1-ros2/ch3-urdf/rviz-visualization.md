# Visualizing Robots in RViz

## Introduction

RViz (ROS Visualization) is the primary 3D visualization tool for ROS 2. It allows you to see your robot model, sensor data, transforms, and much more in real-time. This lesson covers how to visualize URDF models, configure displays, and debug robot visualization issues.

## Learning Objectives

- Launch and configure RViz for robot visualization
- Display URDF models with RobotModel display
- Use joint_state_publisher for interactive control
- Visualize TF frames and coordinate systems
- Debug common visualization issues
- Save and load RViz configurations

## Prerequisites

- Understanding of URDF syntax
- ROS 2 Humble installed
- Basic understanding of ROS 2 topics and nodes

---

## Launching RViz with URDF

### Method 1: Using urdf_tutorial Package

```bash
# Install urdf_tutorial
sudo apt install ros-humble-urdf-tutorial

# Launch with your URDF
ros2 launch urdf_tutorial display.launch.py model:=/path/to/your_robot.urdf
```

This launch file automatically:
- Starts RViz
- Publishes robot state
- Configures RobotModel display
- Starts joint_state_publisher_gui

### Method 2: Manual Launch

**Terminal 1: Publish Robot Description**
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat my_robot.urdf)"
```

**Terminal 2: Publish Joint States**
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

**Terminal 3: Launch RViz**
```bash
rviz2
```

---

## RViz Interface Overview

### Main Components

1. **3D View**: Main visualization window
2. **Displays Panel**: Add/remove visualization elements
3. **Tool Properties**: Configuration for selected displays
4. **Views**: Camera controls and perspectives
5. **Time**: Visualization timestamp control

### Essential Displays

| Display Type | Purpose |
|--------------|---------|
| RobotModel | Show robot URDF with current joint states |
| TF | Visualize coordinate frame transforms |
| Grid | Ground reference plane |
| Axes | Show X-Y-Z axes at origin |
| Camera | View camera image streams |
| LaserScan | Visualize LiDAR data |
| PointCloud2 | 3D point clouds |
| Marker | Custom shapes and annotations |

---

## Configuring RobotModel Display

### Adding RobotModel

1. Click **Add** in Displays panel
2. Select **RobotModel**
3. Click **OK**

### Configuration Options

**Description Topic:**
```
/robot_description
```

**TF Prefix:** (usually empty)

**Visual Enabled:** Check to show visual geometry

**Collision Enabled:** Check to show collision geometry

**Alpha:** Transparency (0.0 = invisible, 1.0 = opaque)

---

## Joint State Publisher GUI

The joint_state_publisher_gui provides sliders to move robot joints interactively.

### Features

- Slider for each non-fixed joint
- Real-time joint angle display
- Respects joint limits from URDF
- Publishes to `/joint_states` topic

### Usage

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

**Tips:**
- Center slider = mid-range of joint limits
- Move sliders to see robot motion
- Watch RViz update in real-time

---

## Hands-On Exercise

### Exercise 1: Visualize Your Humanoid

1. Launch RViz with your humanoid URDF
2. Add RobotModel and TF displays
3. Use joint_state_publisher_gui to move joints
4. Save configuration as `humanoid.rviz`

---

## Summary

You learned to:
- Launch RViz with URDF models
- Configure RobotModel display
- Use joint_state_publisher_gui
- Visualize TF frames
- Debug visualization issues
- Save/load RViz configurations

---

## Next Steps

- [Joint Controllers](./joint-controllers.md) - Control robot joints programmatically
- [Workspace Setup](../ch4-packages/workspace-setup.md) - Create ROS 2 packages

---

## Additional Resources

- [RViz User Guide](https://github.com/ros2/rviz/blob/ros2/docs/user_guide.md)
- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
