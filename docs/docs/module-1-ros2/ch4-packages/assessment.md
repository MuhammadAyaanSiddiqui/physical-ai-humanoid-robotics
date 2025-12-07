# Module 1 Assessment: Build a Robot Control System

**Module**: The Robotic Nervous System
**Type**: Capstone Project
**Estimated Time**: 6-10 hours
**Difficulty**: Intermediate

## Overview

This assessment tests your mastery of **ROS 2 fundamentals, Python integration (rclpy), URDF modeling, and package development**. You will build a complete multi-node robot control system from scratch.

## Learning Objectives Assessed

By completing this project, you demonstrate ability to:

- ✅ Create a complete ROS 2 workspace with multiple packages
- ✅ Design and implement URDF robot models
- ✅ Write Python nodes with publishers, subscribers, and timers
- ✅ Use parameters for runtime configuration
- ✅ Create launch files for system startup
- ✅ Visualize robots in RViz2
- ✅ Debug ROS 2 systems
- ✅ Follow ROS 2 best practices

## Project Requirements

### Overview

Build a **simulated mobile robot** with the following components:

1. **URDF Model**: A differential-drive robot with sensors
2. **Control Node**: Publishes velocity commands based on keyboard input or algorithms
3. **Odometry Node**: Tracks robot position and publishes odometry
4. **Sensor Node**: Simulates sensor data (laser scan or camera)
5. **Visualization**: RViz2 configuration showing robot, odometry, and sensor data
6. **Launch System**: Single launch file to start entire system

## Detailed Specifications

### 1. URDF Robot Model (25 points)

Create `robot_description` package with URDF file.

**Robot Requirements**:
- Differential-drive mobile base (2 wheels + 1 caster)
- Chassis: Box shape (0.6m × 0.4m × 0.2m)
- Wheels: 2 revolute joints, cylindrical shape (radius: 0.1m, width: 0.05m)
- Caster wheel: Fixed joint, spherical shape (radius: 0.05m)
- Sensor mount: Link for laser scanner or camera
- TF tree: `base_link` → `left_wheel_link`, `right_wheel_link`, `caster_link`, `sensor_link`

**Materials**:
- Chassis: Blue (RGBA: 0 0 0.8 1)
- Wheels: Black (RGBA: 0.1 0.1 0.1 1)
- Sensor: Red (RGBA: 0.8 0 0 1)

**Deliverable**:
```
robot_description/
├── package.xml
├── setup.py
├── urdf/
│   └── mobile_robot.urdf
├── rviz/
│   └── robot.rviz
└── launch/
    └── display.launch.py
```

**Validation**:
```bash
check_urdf urdf/mobile_robot.urdf
ros2 launch robot_description display.launch.py
```

### 2. Robot Control Node (20 points)

Create `robot_control` package with velocity controller.

**Node Name**: `velocity_controller`

**Publishers**:
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/joint_states` (sensor_msgs/JointState): Wheel joint positions

**Parameters**:
- `max_linear_vel` (double, default: 0.5): Maximum forward velocity (m/s)
- `max_angular_vel` (double, default: 1.0): Maximum turn rate (rad/s)
- `update_rate` (double, default: 10.0): Control loop frequency (Hz)

**Behavior**:

**Option A - Keyboard Control**:
- Use `pynput` or similar to read keyboard input
- W/S: forward/backward
- A/D: rotate left/right
- Space: stop

**Option B - Autonomous Pattern**:
- Drive in a square pattern (1m sides)
- Or drive in a circle (0.5m radius)
- Publish joint states based on commanded velocity

**Deliverable**:
```python
# robot_control/velocity_controller.py
class VelocityController(Node):
    def __init__(self):
        # Declare parameters
        # Create publishers
        # Create timer for control loop

    def control_loop(self):
        # Read input (keyboard or pattern)
        # Publish cmd_vel
        # Update and publish joint_states
```

### 3. Odometry Node (20 points)

Create odometry tracking node.

**Node Name**: `odometry_publisher`

**Subscribers**:
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands

**Publishers**:
- `/odom` (nav_msgs/Odometry): Robot pose estimate
- `/tf` (TF transform): `odom` → `base_link`

**Parameters**:
- `wheel_base` (double, default: 0.4): Distance between wheels (m)
- `wheel_radius` (double, default: 0.1): Wheel radius (m)
- `publish_rate` (double, default: 20.0): Odometry update rate (Hz)

**Behavior**:
- Subscribe to `/cmd_vel`
- Integrate velocity to estimate pose (x, y, theta)
- Publish odometry message with pose and twist
- Broadcast TF transform from `odom` to `base_link`

**Equations**:
```python
# Dead reckoning odometry
dt = 1.0 / publish_rate
dx = linear_vel * cos(theta) * dt
dy = linear_vel * sin(theta) * dt
dtheta = angular_vel * dt

x += dx
y += dy
theta += dtheta
```

**Deliverable**:
```python
# robot_control/odometry_publisher.py
class OdometryPublisher(Node):
    def __init__(self):
        # Initialize pose (x, y, theta)
        # Create subscriber for cmd_vel
        # Create publisher for odom
        # Create TF broadcaster

    def cmd_vel_callback(self, msg):
        # Store latest velocity

    def publish_odometry(self):
        # Integrate velocity
        # Create Odometry message
        # Broadcast TF
```

### 4. Sensor Simulation Node (15 points)

Create simulated sensor node.

**Node Name**: `sensor_simulator`

**Publishers**:
- `/scan` (sensor_msgs/LaserScan): Simulated laser scan
  OR
- `/camera/image_raw` (sensor_msgs/Image): Simulated camera image

**Parameters**:
- `sensor_rate` (double, default: 10.0): Sensor update rate (Hz)
- `num_ranges` (int, default: 360): Number of laser rays (if laser)
- `max_range` (double, default: 10.0): Maximum detection range (m)

**Behavior**:

**Option A - Laser Scanner**:
- Simulate 360-degree laser scan
- Generate synthetic obstacles (e.g., walls at fixed distances)
- Vary ranges slightly for realism

**Option B - Camera**:
- Generate solid color or simple pattern image
- 640×480 resolution
- RGB8 encoding

**Deliverable**:
```python
# robot_control/sensor_simulator.py
class SensorSimulator(Node):
    def __init__(self):
        # Create publisher
        # Create timer

    def publish_sensor_data(self):
        # Generate synthetic data
        # Publish message
```

### 5. Launch System (10 points)

Create comprehensive launch file.

**Launch File**: `robot_control/launch/robot.launch.py`

**Launched Nodes**:
1. robot_state_publisher (with URDF)
2. velocity_controller
3. odometry_publisher
4. sensor_simulator
5. RViz2 (with config file)

**Launch Arguments**:
- `use_sim_time` (default: false)
- `max_vel` (default: 0.5): Pass to velocity_controller
- `enable_sensor` (default: true): Conditionally launch sensor node

**Deliverable**:
```python
# robot_control/launch/robot.launch.py
def generate_launch_description():
    # Declare arguments
    # Load URDF
    # Configure nodes
    # Return LaunchDescription with all nodes
```

### 6. RViz Configuration (5 points)

Create RViz config file showing:

- Robot model (from URDF)
- TF frames (show axes)
- Odometry (Path display)
- Sensor data (LaserScan or Image)
- Grid (ground plane)

**Deliverable**:
- `robot_control/rviz/robot.rviz`

### 7. Documentation (5 points)

Create README files:

**robot_description/README.md**:
- Description of robot design
- How to visualize: `ros2 launch robot_description display.launch.py`
- URDF structure explanation

**robot_control/README.md**:
- System architecture (node graph)
- How to run: `ros2 launch robot_control robot.launch.py`
- Parameters and how to change them
- Known limitations

## Grading Rubric

### URDF Model (25 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| Structure | 10 | All required links and joints present |
| Visual | 5 | Correct geometry and materials |
| TF Tree | 5 | Valid transform tree, passes check_urdf |
| Launch File | 5 | Display launch file works correctly |

### Control Node (20 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| Publishers | 8 | Publishes cmd_vel and joint_states correctly |
| Parameters | 4 | All required parameters declared and used |
| Behavior | 6 | Control logic works as specified |
| Code Quality | 2 | Clean code, proper logging |

### Odometry Node (20 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| Subscriber | 4 | Subscribes to cmd_vel correctly |
| Publishers | 8 | Publishes odom message with correct data |
| TF Broadcast | 6 | Broadcasts odom→base_link transform |
| Integration | 2 | Pose estimation is reasonable |

### Sensor Node (15 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| Publisher | 7 | Publishes sensor data in correct format |
| Parameters | 3 | Parameters declared and used |
| Data Quality | 5 | Synthetic data is reasonable |

### Launch System (10 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| Completeness | 5 | Launches all required nodes |
| Arguments | 3 | Launch arguments work correctly |
| Conditional | 2 | Conditional launching works |

### RViz Configuration (5 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| Displays | 3 | All required displays present |
| Usability | 2 | Config is well-organized and useful |

### Documentation (5 points)

| Criteria | Points | Description |
|----------|--------|-------------|
| Completeness | 3 | README files explain all key aspects |
| Clarity | 2 | Instructions are clear and accurate |

**Total: 100 points**

**Grading Scale**:
- 90-100: Excellent (A)
- 80-89: Good (B)
- 70-79: Satisfactory (C)
- 60-69: Needs Improvement (D)
- < 60: Incomplete (F)

## Getting Started

### Step 1: Set Up Workspace

```bash
mkdir -p ~/robot_assessment_ws/src
cd ~/robot_assessment_ws/src
```

### Step 2: Create Packages

```bash
# URDF package
ros2 pkg create --build-type ament_python robot_description

# Control package
ros2 pkg create --build-type ament_python robot_control \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs tf2_ros
```

### Step 3: Create Directory Structure

```bash
# robot_description
cd robot_description
mkdir -p urdf rviz launch

# robot_control
cd ../robot_control
mkdir -p launch rviz config
```

### Step 4: Start with URDF

Create basic robot model:
```bash
cd robot_description/urdf
nano mobile_robot.urdf
# Start with base_link, add components incrementally
```

Test frequently:
```bash
check_urdf mobile_robot.urdf
```

### Step 5: Build and Test Incrementally

```bash
cd ~/robot_assessment_ws
colcon build --symlink-install
source install/setup.bash
```

## Testing Your System

### Test 1: URDF Validation

```bash
# Check URDF syntax
check_urdf robot_description/urdf/mobile_robot.urdf

# Visualize in RViz
ros2 launch robot_description display.launch.py
```

**Expected**: Robot appears in RViz, joints move with sliders.

### Test 2: Control Node

```bash
# Terminal 1: Start control node
ros2 run robot_control velocity_controller

# Terminal 2: Check topics
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /joint_states

# Terminal 3: Check publish rate
ros2 topic hz /cmd_vel
```

**Expected**: cmd_vel and joint_states published at correct rates.

### Test 3: Odometry Node

```bash
# Terminal 1: Start odometry
ros2 run robot_control odometry_publisher

# Terminal 2: Publish test velocity
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Terminal 3: Monitor odometry
ros2 topic echo /odom
```

**Expected**: Odometry x increases over time.

### Test 4: Complete System

```bash
ros2 launch robot_control robot.launch.py
```

**Expected**:
- All nodes start
- Robot visible in RViz
- Sensor data appears
- Odometry path shows robot motion

### Test 5: Parameters

```bash
# Launch with custom parameters
ros2 launch robot_control robot.launch.py max_vel:=1.0 enable_sensor:=false

# Check parameter was set
ros2 param get /velocity_controller max_linear_vel
```

**Expected**: Parameter value matches launch argument.

## Common Issues and Solutions

### Issue 1: "Package not found"

**Solution**: Build and source workspace
```bash
cd ~/robot_assessment_ws
colcon build --packages-select robot_description robot_control
source install/setup.bash
```

### Issue 2: Robot not visible in RViz

**Check**:
1. robot_state_publisher running?
2. Fixed Frame set to `base_link`?
3. RobotModel display added?
4. URDF loaded correctly? `ros2 topic echo /robot_description`

### Issue 3: Odometry not updating

**Check**:
1. Subscribing to `/cmd_vel`?
2. Publishing velocity commands?
3. TF broadcaster configured?
4. Check `ros2 run tf2_ros tf2_echo odom base_link`

### Issue 4: Nodes crash on startup

**Debug**:
```bash
# Check logs
ros2 run robot_control my_node --ros-args --log-level debug

# Check for Python errors
cat ~/.ros/log/latest/my_node-*.log
```

## Submission Requirements

Submit a **zip file** or **Git repository** containing:

```
robot_assessment_ws/
└── src/
    ├── robot_description/
    │   ├── package.xml
    │   ├── setup.py
    │   ├── urdf/
    │   │   └── mobile_robot.urdf
    │   ├── rviz/
    │   │   └── robot.rviz
    │   ├── launch/
    │   │   └── display.launch.py
    │   └── README.md
    └── robot_control/
        ├── package.xml
        ├── setup.py
        ├── robot_control/
        │   ├── __init__.py
        │   ├── velocity_controller.py
        │   ├── odometry_publisher.py
        │   └── sensor_simulator.py
        ├── launch/
        │   └── robot.launch.py
        ├── rviz/
        │   └── robot.rviz
        ├── config/
        │   └── params.yaml (optional)
        └── README.md
```

**Include**:
- All source code
- URDF files
- Launch files
- RViz configs
- README files
- **DO NOT** include `build/`, `install/`, `log/` directories

## Bonus Challenges (+10 points each)

### Bonus 1: Advanced Visualization

- Add robot mesh models instead of primitive shapes
- Animate wheel rotation in RViz
- Show velocity vectors

### Bonus 2: Configuration System

- Load all parameters from YAML file
- Support multiple robot configurations (slow, fast, etc.)
- Validate parameter ranges

### Bonus 3: Safety Features

- Add velocity limiting (gradual acceleration/deceleration)
- Implement obstacle avoidance (stop if laser detects object < 0.5m)
- Add emergency stop service

### Bonus 4: Testing

- Write Python unit tests for odometry integration
- Create integration tests using `launch_testing`
- Achieve > 80% code coverage

## Example Timeline

**Hour 1-2**: URDF model and visualization
- Create robot_description package
- Design and test URDF
- Create display launch file

**Hour 3-4**: Control node
- Create robot_control package
- Implement velocity_controller
- Test with keyboard or pattern

**Hour 5-6**: Odometry node
- Implement odometry_publisher
- Test pose estimation
- Debug TF issues

**Hour 7-8**: Sensor and integration
- Implement sensor_simulator
- Create main launch file
- Configure RViz

**Hour 9-10**: Polish and documentation
- Test complete system
- Write README files
- Fix bugs
- Test on fresh workspace

## Tips for Success

1. **Start Simple**: Get basic version working first, add features incrementally
2. **Test Frequently**: Test each component before moving to next
3. **Use Examples**: Refer to course materials and ROS 2 documentation
4. **Log Everything**: Add informative log messages for debugging
5. **Check Topics**: Use `ros2 topic echo` and `ros2 topic hz` to verify behavior
6. **Visualize**: Use rqt_graph and RViz to understand system
7. **Ask for Help**: If stuck > 30 minutes, seek assistance
8. **Version Control**: Use Git to track changes and enable rollback

## Learning Resources

- **Course Materials**: Review all Module 1 lessons
- **ROS 2 Docs**: https://docs.ros.org/en/humble/
- **URDF Tutorial**: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html
- **rclpy API**: https://docs.ros2.org/latest/api/rclpy/
- **Example Code**: Check course `static/code/` examples

## After Completion

Once you complete this assessment, you will have:

✅ **Practical ROS 2 Skills**: Built a complete robot system
✅ **Portfolio Project**: Demonstrable project for resume/portfolio
✅ **Foundation**: Ready for advanced topics (navigation, manipulation, perception)
✅ **Confidence**: Ability to create your own ROS 2 projects

## Self-Assessment Questions

Before submission, ask yourself:

- [ ] Can I launch the entire system with one command?
- [ ] Does the robot appear correctly in RViz?
- [ ] Do all nodes publish/subscribe to correct topics?
- [ ] Are parameters configurable via launch arguments?
- [ ] Does odometry track robot motion reasonably?
- [ ] Does sensor data appear in RViz?
- [ ] Are README files clear and complete?
- [ ] Does the system work on a fresh workspace?
- [ ] Have I tested all features?
- [ ] Is my code clean and well-documented?

## Next Steps

After completing this assessment:

1. **Review Feedback**: Understand any mistakes and learn from them
2. **Iterate**: Improve based on feedback and bonus challenges
3. **Expand**: Add more features (mapping, navigation, etc.)
4. **Share**: Publish to GitHub, write blog post about learnings
5. **Apply**: Use these skills in personal robotics projects

---

**Good luck! This assessment demonstrates your mastery of ROS 2 fundamentals. Take your time, test thoroughly, and build something you're proud of!**

## Questions or Issues?

If you encounter problems:
- Review course materials
- Check ROS 2 documentation
- Use debugging techniques from Chapter 4
- Consult with instructor/peers
- Post specific questions with error messages

**Remember**: The goal is learning, not just completing. Take time to understand each component!
