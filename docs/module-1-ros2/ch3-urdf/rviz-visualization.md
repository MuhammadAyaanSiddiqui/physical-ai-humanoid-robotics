# Visualizing Robots in RViz

**Module**: The Robotic Nervous System
**Chapter**: Robot Modeling (URDF)
**Estimated Time**: 2-3 hours
**Difficulty**: Beginner to Intermediate

## Prerequisites

- Understanding of URDF files (links and joints)
- ROS 2 Humble with RViz2 installed
- Basic knowledge of coordinate frames

## Learning Objectives

By the end of this lesson, you will be able to:

- Launch and configure RViz2 for robot visualization
- Use robot_state_publisher to broadcast robot state
- Control joint positions with joint_state_publisher_gui
- Visualize TF frames and coordinate transformations
- Debug robot models using RViz tools
- Save and load RViz configurations

## What is RViz?

**RViz** (ROS Visualization) is a 3D visualization tool for ROS 2 that displays:
- Robot models (URDF)
- Sensor data (LaserScan, PointCloud, Images, Camera)
- TF coordinate frames
- Paths and trajectories
- Interactive markers

### RViz2 vs RViz (ROS 1)

RViz2 is the ROS 2 version with:
- Improved performance
- Better plugin architecture
- Qt5-based interface
- Native ROS 2 integration

## Required ROS 2 Nodes

### 1. robot_state_publisher

Publishes robot state based on URDF and joint states:

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat my_robot.urdf)"
```

**What it does**:
- Reads URDF from `robot_description` parameter
- Subscribes to `/joint_states` topic
- Publishes TF transforms for all links
- Broadcasts robot model to `/robot_description` topic

### 2. joint_state_publisher

Publishes joint positions (for testing):

```bash
ros2 run joint_state_publisher joint_state_publisher
```

**What it does**:
- Publishes `/joint_states` messages
- Sets all non-fixed joints to 0 (default position)

### 3. joint_state_publisher_gui

Interactive joint control:

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

**What it does**:
- Provides sliders for each joint
- Publishes joint positions to `/joint_states`
- Allows manual pose configuration

## Quick Start: Visualizing a URDF

### Step 1: Prepare Your URDF

Use the simple humanoid from the previous lesson or any URDF file:

```bash
# Save as simple_robot.urdf
cat > simple_robot.urdf << 'EOF'
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>

  <link name="link1">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>
EOF
```

### Step 2: Launch RViz with Robot Model

**Terminal 1** - Start robot_state_publisher:
```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat simple_robot.urdf)"
```

**Terminal 2** - Start joint_state_publisher_gui:
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

**Terminal 3** - Start RViz:
```bash
ros2 run rviz2 rviz2
```

### Step 3: Configure RViz

1. **Set Fixed Frame**:
   - In left panel, change `Fixed Frame` to `base_link`

2. **Add Robot Model**:
   - Click `Add` button (bottom left)
   - Select `RobotModel`
   - Click `OK`

3. **Result**: You should see your robot!

4. **Move Joint**: Use slider in Joint State Publisher GUI window

## RViz Interface Overview

```
┌─────────────────────────────────────────────────────┐
│ Toolbar                                             │
├──────────┬──────────────────────────────────────────┤
│          │                                          │
│ Displays │         3D View                          │
│  Panel   │                                          │
│          │         (Your Robot Here)                │
│  - Robot │                                          │
│  - TF    │                                          │
│  - Grid  │                                          │
│          │                                          │
├──────────┴──────────────────────────────────────────┤
│ Views Panel                Time Panel               │
└─────────────────────────────────────────────────────┘
```

### Main Components

**1. Displays Panel** (Left):
- Add/remove visualization elements
- Configure display properties
- Enable/disable individual displays

**2. 3D View** (Center):
- Main visualization area
- Interactive camera controls
- Right-click to rotate, scroll to zoom

**3. Toolbar** (Top):
- Interact, Move Camera, Select, etc.
- Time control
- Screenshot tools

**4. Views Panel** (Bottom):
- Camera position and orientation
- Orbit, FPS, Top-down views

**5. Time Panel** (Bottom Right):
- Current ROS time
- Wall clock time

## Essential Display Types

### RobotModel

Displays robot URDF:

**Configuration**:
- **Robot Description**: Topic with URDF (`robot_description`)
- **TF Prefix**: Namespace for TF frames (usually empty)
- **Visual Enabled**: Show visual meshes
- **Collision Enabled**: Show collision shapes (default: off)
- **Alpha**: Transparency (0.0 = invisible, 1.0 = opaque)

**Properties**:
```
RobotModel
├─ Robot Description: robot_description
├─ Visual Enabled: ✓
├─ Collision Enabled: ☐
├─ Alpha: 1.0
└─ Links
    ├─ base_link
    │   ├─ Show Axes: ☐
    │   └─ Show Trail: ☐
    └─ link1
        └─ ...
```

### TF (Transform Frames)

Displays coordinate frames:

**Configuration**:
- **Show Names**: Display frame names
- **Show Axes**: Display XYZ axes (Red=X, Green=Y, Blue=Z)
- **Show Arrows**: Display parent-child relationships
- **Marker Scale**: Size of frame markers
- **Update Interval**: How often to refresh (seconds)

**Add TF Display**:
1. Click `Add`
2. Select `TF`
3. Enable `Show Names` and `Show Axes`

**Result**: See all link frames with XYZ axes

### Grid

Ground plane reference:

**Configuration**:
- **Reference Frame**: Which frame to attach grid to
- **Plane Cell Count**: Number of grid cells
- **Cell Size**: Size of each cell (meters)
- **Color**: Grid line color

### Axes

Shows XYZ axes at origin:

**Configuration**:
- **Length**: Axis length (meters)
- **Radius**: Axis thickness

## Controlling the Camera

### Orbit View (Default)

- **Rotate**: Left-click + drag
- **Pan**: Middle-click + drag (or Shift + left-click + drag)
- **Zoom**: Scroll wheel

### FPS View

Navigate like first-person game:

1. Select `FPS` in Views panel
2. **Move**: W/A/S/D keys
3. **Look**: Left-click + drag
4. **Up/Down**: Q/E keys

### Top-Down View

Fixed orthographic view:

1. Select `TopDownOrtho` in Views panel
2. **Pan**: Middle-click + drag
3. **Zoom**: Scroll wheel

### Reset View

Click `Views` → `Zero` to reset camera to origin

## Visualizing Joint States

### Using joint_state_publisher_gui

**Launch**:
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

**Interface**:
- Slider for each non-fixed joint
- Real-time update in RViz
- Shows joint limits (red regions = out of bounds)

**Example** - Move robot arm:
1. Launch robot_state_publisher, joint_state_publisher_gui, RViz
2. In GUI, move `shoulder_joint` slider
3. Watch arm move in RViz

### Manual Joint States (Command Line)

Publish joint states manually:

```bash
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
  "{name: ['joint1'], position: [1.57]}"
```

This sets `joint1` to 1.57 radians (90 degrees).

## Saving and Loading Configurations

### Save Configuration

After configuring displays, camera, etc.:

1. `File` → `Save Config As`
2. Save as `my_robot.rviz`

**Config File** (YAML):
```yaml
Panels:
  - Class: rviz_common/Displays
    Property Tree:
      - Name: Grid
        Class: rviz_default_plugins/Grid
      - Name: RobotModel
        Class: rviz_default_plugins/RobotModel
        Robot Description: robot_description
        Visual Enabled: true
Visualization Manager:
  Fixed Frame: base_link
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3.0
```

### Load Configuration

```bash
ros2 run rviz2 rviz2 -d my_robot.rviz
```

## Launch File for Complete Setup

Create `display.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    # Get URDF file path
    urdf_file = LaunchConfiguration('model')

    # Declare launch argument for URDF file
    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value='simple_robot.urdf',
        description='Path to robot URDF file'
    )

    # Read URDF file
    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # joint_state_publisher_gui node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # RViz node with config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', 'config/robot.rviz'],
        output='screen'
    )

    return LaunchDescription([
        declare_model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
```

**Usage**:
```bash
ros2 launch display.launch.py model:=my_humanoid.urdf
```

## Debugging Robot Models

### Issue 1: Robot Not Visible

**Symptoms**: RViz opens but no robot

**Check**:
1. **robot_state_publisher running?**
   ```bash
   ros2 node list | grep robot_state_publisher
   ```

2. **robot_description topic published?**
   ```bash
   ros2 topic echo /robot_description
   ```

3. **Fixed Frame set correctly?**
   - Should match a link in your URDF (usually `base_link`)

4. **RobotModel display added?**
   - Check Displays panel

### Issue 2: TF Errors

**Symptoms**: Red text in RViz about TF frames

**Common Errors**:

**"No transform from [X] to [Y]"**:
- joint_state_publisher not running
- joint_states not being published
- Link not connected in URDF

**Check TF tree**:
```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf showing TF tree
```

**Monitor TF**:
```bash
ros2 run tf2_ros tf2_echo base_link link1
# Shows transform from base_link to link1
```

### Issue 3: Robot in Wrong Position

**Symptoms**: Robot floating, upside down, etc.

**Cause**: Fixed Frame mismatch

**Fix**:
- Set `Fixed Frame` to root link (usually `base_link`)
- Check URDF root link with `check_urdf`

### Issue 4: Joints Not Moving

**Symptoms**: Sliders move but robot static

**Check**:
1. **joint_state_publisher_gui publishing?**
   ```bash
   ros2 topic echo /joint_states
   ```

2. **Joint names match URDF?**
   - Names in joint_states must exactly match URDF

3. **robot_state_publisher receiving joint_states?**
   ```bash
   ros2 node info /robot_state_publisher
   # Check Subscribers section
   ```

## Advanced Visualization

### Showing Collision Meshes

Toggle collision geometry:

1. In RobotModel display
2. Expand `Links`
3. For each link, enable `Show Collision`

**Use Case**: Debug collision shapes vs. visual appearance

### Showing Link Frames

Show coordinate frame for each link:

1. Expand `Links` in RobotModel
2. For each link, enable `Show Axes`

**Result**: XYZ axes at each link origin

### Semi-Transparent Robot

Make robot transparent to see internal structure:

1. In RobotModel display
2. Set `Alpha` to `0.5` (50% transparent)

### Comparing Visual vs Collision

**Method 1** - Two RobotModel displays:
1. Add two RobotModel displays
2. First: Visual Enabled, Collision Disabled
3. Second: Visual Disabled, Collision Enabled, set Color

**Method 2** - Toggle:
1. Single RobotModel
2. Enable both Visual and Collision
3. Set Visual Alpha to 0.5

## Interactive Markers (Advanced)

Add interactive controls to move robot:

```bash
ros2 run interactive_marker_twist_server interactive_marker_twist_server
```

Adds 6-DOF marker in RViz for controlling robot pose.

## RViz Command Line Options

### Useful Flags

```bash
# Load specific config
ros2 run rviz2 rviz2 -d my_config.rviz

# Set fixed frame
ros2 run rviz2 rviz2 -f base_link

# Fullscreen mode
ros2 run rviz2 rviz2 --fullscreen

# Hide displays panel
ros2 run rviz2 rviz2 --hide-left-panel
```

## Tips and Tricks

### 1. Quick Frame Reference

**Remember**: **RGB = XYZ**
- **Red** axis = **X** (forward)
- **Green** axis = **Y** (left)
- **Blue** axis = **Z** (up)

### 2. Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `G` | Toggle grid |
| `F` | Focus on selected item |
| `R` | Reset view |
| `T` | Top-down view |
| `O` | Orthographic projection |

### 3. Performance Optimization

For complex robots:
- Disable unnecessary displays
- Reduce TF `Update Interval` to 0.1s
- Use simplified collision meshes
- Lower Grid cell count

### 4. Screenshots

`File` → `Screenshot` (or toolbar camera icon)

Saves high-quality image of 3D view.

## Practice Exercises

### Exercise 1: Visualize Your Humanoid

1. Take the humanoid URDF from the previous lesson
2. Create a launch file to display it in RViz
3. Save an RViz config file
4. Create a "T-pose" using joint sliders

### Exercise 2: TF Tree Visualization

1. Run view_frames to generate TF tree
2. Compare the tree to your URDF structure
3. Identify parent-child relationships

### Exercise 3: Debug Broken URDF

Given a URDF with a disconnected link:
1. Try to visualize it in RViz
2. Identify the TF error
3. Fix the URDF and verify

## Key Takeaways

- **RViz2** is the primary tool for visualizing ROS 2 robots
- **robot_state_publisher** converts URDF + joint_states to TF
- **joint_state_publisher_gui** provides interactive joint control
- **TF display** shows coordinate frames and relationships
- **Save configs** to avoid reconfiguring RViz every time
- **Launch files** automate the complete visualization setup

## What's Next?

Now that you can visualize robots:

- **Next Lesson**: [Joint Controllers](./joint-controllers.md) - Control joint positions programmatically
- **Related**: [Workspace Setup](../ch4-packages/workspace-setup.md)

## Further Reading

- [RViz2 User Guide](https://github.com/ros2/rviz/blob/ros2/README.md)
- [robot_state_publisher Documentation](https://github.com/ros/robot_state_publisher)
- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)

---

**Checkpoint**: You can now visualize and debug robot models in RViz2!
