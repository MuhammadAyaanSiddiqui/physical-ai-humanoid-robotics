# Joint State Publishing and Control

**Module**: The Robotic Nervous System
**Chapter**: Robot Modeling (URDF)
**Estimated Time**: 2-3 hours
**Difficulty**: Intermediate

## Prerequisites

- Understanding of URDF and ROS 2 topics
- Familiarity with RViz visualization
- Python or C++ programming basics

## Learning Objectives

By the end of this lesson, you will be able to:

- Understand the `/joint_states` topic and message format
- Use joint_state_publisher for static poses
- Create custom joint controllers in Python
- Publish joint states programmatically
- Understand the relationship between joint_states and TF
- Implement basic joint trajectory control

## The /joint_states Topic

The `/joint_states` topic is the standard way to communicate joint positions in ROS 2.

### Message Type: sensor_msgs/msg/JointState

```bash
ros2 interface show sensor_msgs/msg/JointState
```

**Output**:
```
std_msgs/Header header
string[] name          # Joint names (must match URDF)
float64[] position     # Joint positions (radians or meters)
float64[] velocity     # Joint velocities (rad/s or m/s) - optional
float64[] effort       # Joint efforts/torques (Nm or N) - optional
```

### Example Message

```python
from sensor_msgs.msg import JointState

msg = JointState()
msg.header.stamp = self.get_clock().now().to_msg()
msg.name = ['joint1', 'joint2', 'joint3']
msg.position = [0.0, 1.57, -0.78]  # radians
msg.velocity = [0.0, 0.0, 0.0]
msg.effort = []  # Optional, leave empty if not measuring
```

**Key Points**:
- `name`, `position`, `velocity`, and `effort` arrays must have same length
- Joint names must exactly match URDF joint names
- Order doesn't matter (matched by name)
- Missing joints default to 0

## joint_state_publisher

**joint_state_publisher** publishes static joint positions.

### Basic Usage

```bash
ros2 run joint_state_publisher joint_state_publisher
```

**What it does**:
- Reads URDF from `/robot_description` topic
- Finds all non-fixed joints
- Publishes joint positions to `/joint_states`
- Default: all joints at 0

### With Initial Positions

Pass parameters to set initial joint positions:

```bash
ros2 run joint_state_publisher joint_state_publisher \
  --ros-args -p joint1:=1.57 -p joint2:=0.78
```

Or via launch file:

```python
Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    parameters=[{
        'joint1': 1.57,
        'joint2': 0.78
    }]
)
```

### Source Joints from File

Create `joint_states.yaml`:

```yaml
/**:
  ros__parameters:
    joint1: 1.57
    joint2: -0.78
    joint3: 0.0
```

Load:

```bash
ros2 run joint_state_publisher joint_state_publisher \
  --ros-args --params-file joint_states.yaml
```

## joint_state_publisher_gui

Interactive GUI with sliders:

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

**Features**:
- Slider for each joint
- Shows joint limits (red = out of range)
- Real-time publishing to `/joint_states`
- Randomize button
- Center button (reset to zero)

**Use Cases**:
- Manual posing of robots
- Testing URDF joint limits
- Creating reference poses
- Debugging visualization

## Creating a Custom Joint Publisher

### Simple Publisher Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Timer (publish at 50 Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)

        # Joint configuration
        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.positions = [0.0, 0.0, 0.0]

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        msg.velocity = []
        msg.effort = []

        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Usage**:
```bash
python3 joint_state_publisher.py
```

### Sinusoidal Motion

Create smooth oscillating motion:

```python
class SinusoidalJointPublisher(Node):
    def __init__(self):
        super().__init__('sinusoidal_joint_publisher')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.joint_names = ['joint1', 'joint2']
        self.time = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Sinusoidal motion with different frequencies
        msg.position = [
            math.sin(self.time),        # joint1: 1 Hz
            math.sin(2 * self.time)     # joint2: 2 Hz
        ]

        self.joint_pub.publish(msg)

        self.time += 0.02  # Increment time
```

**Result**: Joints oscillate smoothly.

## Trajectory Control

Move joints from position A to position B over time.

### Linear Interpolation

```python
class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.joint_names = ['joint1', 'joint2']

        # Start and goal positions
        self.start_pos = [0.0, 0.0]
        self.goal_pos = [1.57, -0.78]

        # Trajectory parameters
        self.duration = 2.0  # seconds
        self.elapsed = 0.0
        self.dt = 0.02

    def timer_callback(self):
        # Linear interpolation parameter (0 to 1)
        alpha = min(self.elapsed / self.duration, 1.0)

        # Interpolate between start and goal
        positions = [
            self.start_pos[i] + alpha * (self.goal_pos[i] - self.start_pos[i])
            for i in range(len(self.joint_names))
        ]

        # Publish
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions

        self.joint_pub.publish(msg)

        # Update time
        self.elapsed += self.dt

        if self.elapsed >= self.duration:
            self.get_logger().info('Trajectory complete')
```

### Velocity-Based Control

```python
class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.joint_names = ['joint1']
        self.position = [0.0]
        self.velocity = [0.5]  # rad/s
        self.dt = 0.02

    def timer_callback(self):
        # Integrate velocity to get position
        self.position[0] += self.velocity[0] * self.dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.position
        msg.velocity = self.velocity

        self.joint_pub.publish(msg)
```

## Combining with Subscribers

React to commands and update joint states:

```python
from std_msgs.msg import Float64

class CommandedJointController(Node):
    def __init__(self):
        super().__init__('commanded_joint_controller')

        # Publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Subscriber for commands
        self.cmd_sub = self.create_subscription(
            Float64,
            'joint1_command',
            self.command_callback,
            10
        )

        # State
        self.joint_names = ['joint1', 'joint2']
        self.positions = [0.0, 0.0]

        # Publish current state at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_state)

    def command_callback(self, msg):
        """Update joint1 position based on command"""
        self.positions[0] = msg.data
        self.get_logger().info(f'Received command: {msg.data:.2f}')

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions

        self.joint_pub.publish(msg)
```

**Usage**:
```bash
# Terminal 1: Run controller
python3 commanded_joint_controller.py

# Terminal 2: Send command
ros2 topic pub /joint1_command std_msgs/msg/Float64 "data: 1.57"
```

## Joint State Aggregation

If multiple nodes publish joint states (e.g., different limbs), aggregate them:

```python
class JointStateAggregator(Node):
    def __init__(self):
        super().__init__('joint_state_aggregator')

        # Subscribe to multiple joint state topics
        self.create_subscription(
            JointState, 'left_arm/joint_states', self.left_arm_callback, 10
        )
        self.create_subscription(
            JointState, 'right_arm/joint_states', self.right_arm_callback, 10
        )

        # Publisher for combined state
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Storage
        self.left_arm_state = None
        self.right_arm_state = None

        # Publish combined state at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_combined)

    def left_arm_callback(self, msg):
        self.left_arm_state = msg

    def right_arm_callback(self, msg):
        self.right_arm_state = msg

    def publish_combined(self):
        if self.left_arm_state is None or self.right_arm_state is None:
            return

        combined = JointState()
        combined.header.stamp = self.get_clock().now().to_msg()

        # Combine names and positions
        combined.name = (self.left_arm_state.name +
                        self.right_arm_state.name)
        combined.position = (list(self.left_arm_state.position) +
                            list(self.right_arm_state.position))

        self.joint_pub.publish(combined)
```

## Debugging Joint States

### Monitor Joint States

```bash
ros2 topic echo /joint_states
```

**Output**:
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: ''
name:
- joint1
- joint2
position:
- 0.0
- 1.57
velocity: []
effort: []
---
```

### Check Publishing Rate

```bash
ros2 topic hz /joint_states
```

**Output**:
```
average rate: 50.023
    min: 0.019s max: 0.021s std dev: 0.00012s window: 50
```

**Recommendation**: 10-100 Hz for most robots

### Verify Joint Names

```bash
ros2 topic echo /joint_states --field name
```

**Output**:
```
['joint1', 'joint2', 'joint3']
```

Compare to URDF joint names:

```bash
grep "<joint name=" my_robot.urdf
```

## TF Integration

`robot_state_publisher` converts joint_states to TF:

```
joint_states (topic)
        ↓
robot_state_publisher (node)
        ↓
TF transforms (tf_static + tf)
        ↓
RViz (visualization)
```

### Check TF Transforms

```bash
ros2 run tf2_ros tf2_echo base_link link1
```

**Output**:
```
At time 1234567890.123
- Translation: [0.000, 0.000, 0.100]
- Rotation: in Quaternion [0.000, 0.000, 0.707, 0.707]
```

Rotation quaternion corresponds to joint position.

## Complete Example: Humanoid Arm Waving

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class ArmWaveController(Node):
    def __init__(self):
        super().__init__('arm_wave_controller')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)

        # Humanoid arm joints
        self.joint_names = [
            'left_shoulder_joint',
            'left_elbow_joint',
            'right_shoulder_joint',
            'right_elbow_joint'
        ]

        self.time = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Left arm: wave up and down
        left_shoulder = 0.5 * math.sin(2 * self.time)  # -0.5 to +0.5 rad
        left_elbow = 1.0 + 0.3 * math.sin(2 * self.time)  # Bent elbow

        # Right arm: rest position
        right_shoulder = 0.0
        right_elbow = 0.0

        msg.position = [
            left_shoulder,
            left_elbow,
            right_shoulder,
            right_elbow
        ]

        self.joint_pub.publish(msg)

        self.time += 0.02

def main(args=None):
    rclpy.init(args=args)
    node = ArmWaveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Result**: Left arm waves while right arm stays still.

## Best Practices

### 1. Always Set Timestamp

```python
# Good
msg.header.stamp = self.get_clock().now().to_msg()

# Bad
msg.header.stamp.sec = 0  # Will cause TF issues
```

### 2. Publish at Consistent Rate

```python
# Good: Fixed timer
self.create_timer(0.02, self.timer_callback)  # 50 Hz

# Bad: Variable rate
time.sleep(random.uniform(0.01, 0.05))  # Inconsistent
```

### 3. Match URDF Joint Names Exactly

```python
# Good: Matches URDF
msg.name = ['left_shoulder_joint', 'left_elbow_joint']

# Bad: Typo or mismatch
msg.name = ['left_shoulder', 'elbow_left']  # Won't work!
```

### 4. Include All Joints

```python
# Good: All joints present
msg.name = ['joint1', 'joint2', 'joint3']
msg.position = [0.0, 1.57, -0.78]

# Okay: Missing joints default to 0
msg.name = ['joint1', 'joint3']
msg.position = [0.0, -0.78]
# joint2 will be 0
```

### 5. Handle Velocity and Effort Properly

```python
# Good: Include if you have the data
msg.velocity = [0.1, 0.2, 0.0]
msg.effort = [5.0, 3.2, 1.1]

# Good: Omit if you don't have the data
msg.velocity = []
msg.effort = []

# Bad: Wrong length
msg.name = ['j1', 'j2']
msg.position = [0.0, 1.0]
msg.velocity = [0.1]  # ERROR: Length mismatch!
```

## Common Issues

### Issue 1: Robot Not Moving in RViz

**Symptoms**: Joint sliders move but robot static

**Check**:
1. robot_state_publisher running?
2. Joint names match URDF?
3. Timestamp set?

**Fix**:
```bash
# Verify joint names
ros2 topic echo /joint_states --field name

# Compare with URDF
grep "joint name=" my_robot.urdf
```

### Issue 2: Jerky Motion

**Symptoms**: Robot jumps between positions

**Cause**: Publishing rate too slow or inconsistent

**Fix**:
```python
# Increase publishing rate
self.create_timer(0.01, self.callback)  # 100 Hz instead of 10 Hz
```

### Issue 3: TF Warnings

**Symptoms**: "Transform from X to Y is NaN"

**Cause**: Invalid joint position (NaN or inf)

**Fix**:
```python
# Validate positions
if any(math.isnan(p) or math.isinf(p) for p in positions):
    self.get_logger().error('Invalid joint position!')
    return
```

## Practice Exercises

### Exercise 1: Pendulum Motion

Create a controller that makes a single joint swing like a pendulum using physics:
- Use sine wave for position
- Calculate velocity as derivative
- Publish both position and velocity

### Exercise 2: Sequence Controller

Create a node that moves through a sequence of poses:
1. Neutral pose (2 seconds)
2. Arms up (2 seconds)
3. Arms forward (2 seconds)
4. Return to neutral (2 seconds)
5. Repeat

### Exercise 3: Remote Control

Create a service that accepts a target joint configuration and smoothly moves to it over a specified duration.

## Key Takeaways

- `/joint_states` topic is the standard for communicating joint positions
- **joint_state_publisher** provides static joint positions
- **joint_state_publisher_gui** enables interactive control
- Custom controllers publish to `/joint_states` at 10-100 Hz
- **robot_state_publisher** converts joint_states to TF transforms
- Joint names must exactly match URDF
- Always set header timestamp

## What's Next?

You've completed Chapter 3: Robot Modeling! Next:

- **Next Chapter**: [Workspace Setup](../ch4-packages/workspace-setup.md) - Create ROS 2 packages
- **Related**: [Launch Files](../ch4-packages/launch-files.md)

## Further Reading

- [joint_state_publisher Documentation](https://github.com/ros/joint_state_publisher)
- [sensor_msgs/JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)
- [robot_state_publisher](https://github.com/ros/robot_state_publisher/tree/ros2)

---

**Checkpoint**: You can now publish and control robot joint states!
