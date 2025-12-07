# ROS 2 Code Examples

This directory contains complete, runnable ROS 2 examples demonstrating fundamental concepts covered in Module 1: The Robotic Nervous System.

## Contents

- [minimal_publisher.py](#minimal-publisher) - Basic publisher example
- [minimal_subscriber.py](#minimal-subscriber) - Basic subscriber example
- [service_example.py](#service-example) - Service server and client
- [simple_humanoid.urdf](#simple-humanoid-urdf) - 10-joint humanoid robot model
- [my_robot_package/](#my-robot-package) - Complete ROS 2 package structure

---

## Prerequisites

Before running these examples, ensure you have:

1. **ROS 2 Humble installed**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Required dependencies**:
   ```bash
   sudo apt install ros-humble-rclpy ros-humble-std-msgs \
                    ros-humble-geometry-msgs ros-humble-sensor-msgs \
                    ros-humble-example-interfaces
   ```

3. **Python 3** (Python 3.10+ recommended)

---

## Minimal Publisher

**File**: `minimal_publisher.py`

### Description

Demonstrates creating a ROS 2 publisher that sends String messages at 1 Hz.

### Key Concepts

- Creating a ROS 2 node
- Publishing messages to a topic
- Using timers for periodic operations
- ROS 2 logging

### How to Run

**Option 1: Standalone Script**

```bash
# Make executable
chmod +x minimal_publisher.py

# Run directly
python3 minimal_publisher.py
```

**Option 2: In a ROS 2 Package**

1. Copy to your package:
   ```bash
   cp minimal_publisher.py ~/robot_ws/src/my_pkg/my_pkg/
   ```

2. Add to `setup.py` entry_points:
   ```python
   'minimal_publisher = my_pkg.minimal_publisher:main',
   ```

3. Build and run:
   ```bash
   cd ~/robot_ws
   colcon build --packages-select my_pkg
   source install/setup.bash
   ros2 run my_pkg minimal_publisher
   ```

### Expected Output

```
[INFO] [minimal_publisher]: Minimal Publisher node has started!
[INFO] [minimal_publisher]: Publishing to topic: /chatter
[INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 0"
[INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 1"
[INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 2"
...
```

### Testing

**Terminal 1** - Run publisher:
```bash
python3 minimal_publisher.py
```

**Terminal 2** - Echo topic:
```bash
ros2 topic echo /chatter
```

**Expected**:
```
data: 'Hello ROS 2: 0'
---
data: 'Hello ROS 2: 1'
---
```

**Terminal 3** - Check publish rate:
```bash
ros2 topic hz /chatter
```

**Expected**: `average rate: 1.000`

---

## Minimal Subscriber

**File**: `minimal_subscriber.py`

### Description

Demonstrates creating a ROS 2 subscriber that receives String messages.

### Key Concepts

- Creating a ROS 2 subscriber
- Processing messages in callbacks
- Logging received data

### How to Run

**Standalone**:
```bash
python3 minimal_subscriber.py
```

**In package**: Same process as minimal_publisher.py

### Testing

**Terminal 1** - Run subscriber:
```bash
python3 minimal_subscriber.py
```

**Terminal 2** - Publish test messages:
```bash
ros2 topic pub /chatter std_msgs/String "data: 'Test message'"
```

**Or run with publisher**:
```bash
# Terminal 1
python3 minimal_publisher.py

# Terminal 2
python3 minimal_subscriber.py
```

### Expected Output

```
[INFO] [minimal_subscriber]: Minimal Subscriber node has started!
[INFO] [minimal_subscriber]: Listening to topic: /chatter
[INFO] [minimal_subscriber]: Waiting for messages...
[INFO] [minimal_subscriber]: I heard: "Hello ROS 2: 0" (message #1)
[INFO] [minimal_subscriber]: I heard: "Hello ROS 2: 1" (message #2)
...
```

---

## Service Example

**File**: `service_example.py`

### Description

Demonstrates ROS 2 service server and client for adding two integers.

### Key Concepts

- Creating service servers
- Creating service clients
- Request-response communication
- Synchronous service calls

### How to Run

**Terminal 1** - Start server:
```bash
python3 service_example.py --ros-args -p mode:=server
```

**Terminal 2** - Start client:
```bash
python3 service_example.py --ros-args -p mode:=client
```

**Or call from command line**:
```bash
# Terminal 1: Server
python3 service_example.py --ros-args -p mode:=server

# Terminal 2: Manual call
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"
```

### Expected Output

**Server**:
```
Starting service server...
[INFO] [add_two_ints_server]: AddTwoInts service server started
[INFO] [add_two_ints_server]: Service name: /add_two_ints
[INFO] [add_two_ints_server]: Waiting for requests...
[INFO] [add_two_ints_server]: Request: 5 + 3 = 8
[INFO] [add_two_ints_server]: Request: 10 + 20 = 30
```

**Client**:
```
Starting service client...
[INFO] [add_two_ints_client]: Waiting for service /add_two_ints...
[INFO] [add_two_ints_client]: Service found! Client ready.
[INFO] [add_two_ints_client]: Sending request: 5 + 3
[INFO] [add_two_ints_client]: Result: 5 + 3 = 8
[INFO] [add_two_ints_client]: Sending request: 10 + 20
[INFO] [add_two_ints_client]: Result: 10 + 20 = 30
...
```

### Testing

List services:
```bash
ros2 service list
```

Get service type:
```bash
ros2 service type /add_two_ints
```

Call service:
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 100, b: 200}"
```

---

## Simple Humanoid URDF

**File**: `simple_humanoid.urdf`

### Description

A complete 10-joint humanoid robot model with:
- Torso (base_link)
- Head (1 joint: neck)
- Legs (6 joints: left/right hip, knee, ankle)
- Arms (4 joints: left/right shoulder, elbow)

### Key Concepts

- URDF link and joint definitions
- Robot kinematics
- Visual and collision geometry
- Inertial properties
- Joint limits and dynamics

### How to Validate

**Check URDF syntax**:
```bash
check_urdf simple_humanoid.urdf
```

**Expected output**:
```
robot name is: simple_humanoid
---------- Successfully Parsed XML ---------------
root Link: base_link has 4 child(ren)
    child(1):  head_link
    child(2):  left_hip_link
        child(1):  left_knee_link
            child(1):  left_ankle_link
    child(3):  right_hip_link
        child(1):  right_knee_link
            child(1):  right_ankle_link
    child(4):  left_shoulder_link
        child(1):  left_elbow_link
    child(5):  right_shoulder_link
        child(1):  right_elbow_link
```

### How to Visualize

**Terminal 1** - robot_state_publisher:
```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat simple_humanoid.urdf)"
```

**Terminal 2** - joint_state_publisher_gui:
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

**Terminal 3** - RViz:
```bash
ros2 run rviz2 rviz2
```

**RViz Configuration**:
1. Set Fixed Frame to `base_link`
2. Add → RobotModel
3. Add → TF (enable Show Names and Show Axes)
4. Use sliders in joint_state_publisher_gui to move joints

### Expected Behavior

- Robot appears in RViz
- All 10 joints controllable via sliders
- Robot maintains valid poses within joint limits

### Joint Details

| Joint | Type | Axis | Range (rad) | Purpose |
|-------|------|------|-------------|---------|
| neck_joint | Revolute | Z | ±1.57 | Head rotation |
| left_hip_joint | Revolute | Y | ±1.57 | Left leg swing |
| left_knee_joint | Revolute | Y | 0 to 2.36 | Left knee bend |
| left_ankle_joint | Revolute | Y | ±0.79 | Left foot pitch |
| right_hip_joint | Revolute | Y | ±1.57 | Right leg swing |
| right_knee_joint | Revolute | Y | 0 to 2.36 | Right knee bend |
| right_ankle_joint | Revolute | Y | ±0.79 | Right foot pitch |
| left_shoulder_joint | Revolute | Y | ±3.14 | Left arm swing |
| left_elbow_joint | Revolute | Y | 0 to 2.79 | Left elbow bend |
| right_shoulder_joint | Revolute | Y | ±3.14 | Right arm swing |
| right_elbow_joint | Revolute | Y | 0 to 2.79 | Right elbow bend |

---

## My Robot Package

**Directory**: `my_robot_package/`

### Description

A complete ROS 2 Python package demonstrating best practices for package structure, including nodes, launch files, and configuration.

### Package Structure

```
my_robot_package/
├── package.xml              # Package metadata and dependencies
├── setup.py                 # Python package setup
├── setup.cfg                # Install configuration
├── resource/
│   └── my_robot_package     # Package marker file
├── my_robot_package/        # Python module
│   ├── __init__.py          # Module initializer
│   └── example_node.py      # Example node with pub/sub/params
├── launch/
│   └── example.launch.py    # Launch file
└── config/
    └── params.yaml          # Parameter configuration
```

### How to Build

1. **Copy to workspace**:
   ```bash
   cp -r my_robot_package ~/robot_ws/src/
   ```

2. **Build package**:
   ```bash
   cd ~/robot_ws
   colcon build --packages-select my_robot_package
   source install/setup.bash
   ```

3. **Verify installation**:
   ```bash
   ros2 pkg list | grep my_robot_package
   ```

### How to Run

**Option 1: Run node directly**:
```bash
ros2 run my_robot_package example_node
```

**Option 2: Use launch file**:
```bash
ros2 launch my_robot_package example.launch.py
```

**Option 3: With custom parameters**:
```bash
ros2 launch my_robot_package example.launch.py publish_rate:=5.0 message_prefix:="Custom"
```

**Option 4: Load parameters from YAML**:
```bash
ros2 run my_robot_package example_node --ros-args --params-file \
  $(ros2 pkg prefix my_robot_package)/share/my_robot_package/config/params.yaml
```

### Expected Output

```
[INFO] [example_node]: Example node started!
[INFO] [example_node]: Publish rate: 1.0 Hz
[INFO] [example_node]: Message prefix: Hello
[INFO] [example_node]: Publishing: "Hello 0"
[INFO] [example_node]: Publishing: "Hello 1"
...
```

### Testing

**Check topics**:
```bash
ros2 topic list
# Should show /output_topic and /cmd_vel
```

**Echo output**:
```bash
ros2 topic echo /output_topic
```

**Send velocity command**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.5}}"
```

**Check parameters**:
```bash
ros2 param list /example_node
ros2 param get /example_node publish_rate
```

**Change parameter at runtime**:
```bash
ros2 param set /example_node publish_rate 5.0
```

---

## Common Issues and Solutions

### Issue 1: "No module named 'rclpy'"

**Solution**:
```bash
source /opt/ros/humble/setup.bash
```

### Issue 2: "Package 'example_interfaces' not found"

**Solution**:
```bash
sudo apt install ros-humble-example-interfaces
```

### Issue 3: Python script not executable

**Solution**:
```bash
chmod +x *.py
```

### Issue 4: check_urdf command not found

**Solution**:
```bash
sudo apt install liburdfdom-tools
```

### Issue 5: RViz shows no robot

**Check**:
1. Is robot_state_publisher running?
2. Is Fixed Frame set to `base_link`?
3. Is RobotModel display added?
4. Check: `ros2 topic echo /robot_description`

---

## Additional Resources

- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **rclpy API**: https://docs.ros2.org/latest/api/rclpy/
- **URDF Tutorial**: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html
- **Course Materials**: See `docs/module-1-ros2/` for detailed lessons

---

## Next Steps

After mastering these examples:

1. **Modify the examples**: Change topic names, message types, parameters
2. **Combine examples**: Create multi-node systems
3. **Add features**: Implement services, actions, custom messages
4. **Build projects**: Apply concepts to Module 1 assessment project

---

**Questions?** Refer to the debugging guide in `docs/module-1-ros2/ch4-packages/debugging.md`
