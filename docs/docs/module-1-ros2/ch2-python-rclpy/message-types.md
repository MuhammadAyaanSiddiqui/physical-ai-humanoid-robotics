# ROS 2 Message Types

## Introduction

ROS 2 uses messages to communicate data between nodes. Understanding different message types and how to use them is crucial for building robotic systems. This lesson covers standard message types, how to work with complex messages, and when to create custom messages.

## Learning Objectives

By the end of this lesson, you will be able to:
- Use standard ROS 2 message types (`std_msgs`, `geometry_msgs`, `sensor_msgs`)
- Work with complex nested message structures
- Understand message field types and constraints
- Create and use custom message definitions
- Choose appropriate message types for different scenarios

## Prerequisites

- Understanding of ROS 2 nodes and topics
- Completed "First ROS 2 Node" lesson
- Basic Python knowledge

---

## Standard Message Packages

ROS 2 provides several standard message packages:

| Package | Purpose | Common Messages |
|---------|---------|-----------------|
| `std_msgs` | Basic data types | `String`, `Int32`, `Float64`, `Bool` |
| `geometry_msgs` | Geometric data | `Point`, `Pose`, `Twist`, `Transform` |
| `sensor_msgs` | Sensor data | `Image`, `LaserScan`, `Imu`, `JointState` |
| `nav_msgs` | Navigation | `Odometry`, `Path`, `MapMetaData` |
| `std_srvs` | Standard services | `Empty`, `SetBool`, `Trigger` |

---

## Working with std_msgs

### String Messages

```python
from std_msgs.msg import String

# Create and populate
msg = String()
msg.data = "Hello, ROS 2!"

# Access data
text = msg.data
```

### Numeric Messages

```python
from std_msgs.msg import Int32, Float64, Bool

# Integer
int_msg = Int32()
int_msg.data = 42

# Float
float_msg = Float64()
float_msg.data = 3.14159

# Boolean
bool_msg = Bool()
bool_msg.data = True
```

### Array Messages

```python
from std_msgs.msg import Float32MultiArray, Int32MultiArray

# Float array
array_msg = Float32MultiArray()
array_msg.data = [1.0, 2.5, 3.7, 4.2]

# Int array
int_array_msg = Int32MultiArray()
int_array_msg.data = [10, 20, 30, 40, 50]
```

### Publisher Example with Different Types

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float64


class MultiTypePublisher(Node):
    def __init__(self):
        super().__init__('multi_type_publisher')

        self.string_pub = self.create_publisher(String, 'text_data', 10)
        self.int_pub = self.create_publisher(Int32, 'count_data', 10)
        self.float_pub = self.create_publisher(Float64, 'sensor_data', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        # Publish string
        str_msg = String()
        str_msg.data = f'Message {self.count}'
        self.string_pub.publish(str_msg)

        # Publish integer
        int_msg = Int32()
        int_msg.data = self.count
        self.int_pub.publish(int_msg)

        # Publish float
        float_msg = Float64()
        float_msg.data = self.count * 1.5
        self.float_pub.publish(float_msg)

        self.count += 1
```

---

## Working with geometry_msgs

### Point - 3D Position

```python
from geometry_msgs.msg import Point

# Create a 3D point
point = Point()
point.x = 1.0
point.y = 2.0
point.z = 3.0
```

**Message Definition:**
```
float64 x
float64 y
float64 z
```

### Quaternion - 3D Orientation

```python
from geometry_msgs.msg import Quaternion

# Create orientation (quaternion)
quat = Quaternion()
quat.x = 0.0
quat.y = 0.0
quat.z = 0.0
quat.w = 1.0  # Identity quaternion (no rotation)
```

### Pose - Position + Orientation

```python
from geometry_msgs.msg import Pose, Point, Quaternion

# Create a pose
pose = Pose()
pose.position = Point(x=1.0, y=2.0, z=0.0)
pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
```

**Message Definition:**
```
Point position
Quaternion orientation
```

### Twist - Linear and Angular Velocity

```python
from geometry_msgs.msg import Twist, Vector3

# Create velocity command
cmd_vel = Twist()

# Linear velocity (m/s)
cmd_vel.linear.x = 0.5   # Forward
cmd_vel.linear.y = 0.0   # Sideways (for holonomic robots)
cmd_vel.linear.z = 0.0   # Up/down

# Angular velocity (rad/s)
cmd_vel.angular.x = 0.0  # Roll
cmd_vel.angular.y = 0.0  # Pitch
cmd_vel.angular.z = 0.3  # Yaw (turning)
```

**Robot Control Example:**

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        msg = Twist()

        # Move forward at 0.5 m/s
        msg.linear.x = 0.5

        # Turn at 0.3 rad/s
        msg.angular.z = 0.3

        self.publisher.publish(msg)
        self.get_logger().info('Sent velocity command')

    def stop(self):
        # Send zero velocity to stop
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
```

### Transform - Position, Rotation, and Metadata

```python
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform, Quaternion, Vector3

# Create a transform
transform = TransformStamped()

# Header with timestamp and frame IDs
transform.header.stamp = self.get_clock().now().to_msg()
transform.header.frame_id = 'world'
transform.child_frame_id = 'robot'

# Translation
transform.transform.translation = Vector3(x=1.0, y=0.5, z=0.0)

# Rotation
transform.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
```

---

## Working with sensor_msgs

### JointState - Robot Joint Information

```python
from sensor_msgs.msg import JointState

# Create joint state message
joint_state = JointState()

# Header with timestamp
joint_state.header.stamp = self.get_clock().now().to_msg()

# Joint names
joint_state.name = ['joint1', 'joint2', 'joint3']

# Joint positions (radians or meters)
joint_state.position = [0.0, 1.57, -0.5]

# Joint velocities (optional)
joint_state.velocity = [0.1, 0.0, -0.2]

# Joint efforts/torques (optional)
joint_state.effort = [0.5, 1.2, 0.8]
```

**Humanoid Joint Example:**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class HumanoidJointPublisher(Node):
    def __init__(self):
        super().__init__('humanoid_joint_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joints)
        self.angle = 0.0

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Define humanoid joints
        msg.name = [
            'head_pan', 'head_tilt',
            'left_shoulder', 'left_elbow',
            'right_shoulder', 'right_elbow',
            'left_hip', 'left_knee',
            'right_hip', 'right_knee'
        ]

        # Simulate movement (sinusoidal)
        self.angle += 0.01
        bend = math.sin(self.angle)

        msg.position = [
            0.0,           # head_pan
            bend * 0.3,    # head_tilt
            bend * 0.5,    # left_shoulder
            abs(bend) * 1.0,  # left_elbow
            -bend * 0.5,   # right_shoulder
            abs(bend) * 1.0,  # right_elbow
            bend * 0.3,    # left_hip
            abs(bend) * 0.5,  # left_knee
            -bend * 0.3,   # right_hip
            abs(bend) * 0.5   # right_knee
        ]

        self.publisher.publish(msg)
```

### Imu - Inertial Measurement Unit

```python
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

# Create IMU message
imu_msg = Imu()

# Header
imu_msg.header.stamp = self.get_clock().now().to_msg()
imu_msg.header.frame_id = 'imu_link'

# Orientation (quaternion)
imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

# Angular velocity (rad/s)
imu_msg.angular_velocity = Vector3(x=0.01, y=-0.02, z=0.0)

# Linear acceleration (m/s²)
imu_msg.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)

# Covariance matrices (optional, 9 elements each)
imu_msg.orientation_covariance = [0.0] * 9
imu_msg.angular_velocity_covariance = [0.0] * 9
imu_msg.linear_acceleration_covariance = [0.0] * 9
```

### LaserScan - LiDAR Data

```python
from sensor_msgs.msg import LaserScan
import math

# Create laser scan
scan = LaserScan()

# Header
scan.header.stamp = self.get_clock().now().to_msg()
scan.header.frame_id = 'laser_frame'

# Scan parameters
scan.angle_min = -math.pi / 2  # -90 degrees
scan.angle_max = math.pi / 2   # +90 degrees
scan.angle_increment = math.pi / 180  # 1 degree
scan.time_increment = 0.0
scan.scan_time = 0.1

# Range limits
scan.range_min = 0.1  # meters
scan.range_max = 30.0  # meters

# Range data (distances)
num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
scan.ranges = [1.0] * num_readings  # Example: all readings at 1m

# Intensity data (optional)
scan.intensities = [100.0] * num_readings
```

---

## Understanding Headers

Many messages include a `Header` for metadata:

```python
from std_msgs.msg import Header

header = Header()
header.stamp = self.get_clock().now().to_msg()  # Timestamp
header.frame_id = 'base_link'  # Coordinate frame
```

**Why Headers Matter:**
- **Timestamps**: Know when data was captured
- **Frame IDs**: Understand coordinate systems (critical for TF transforms)
- **Synchronization**: Match data from multiple sensors

**Example with Header:**

```python
from geometry_msgs.msg import PoseStamped

pose_stamped = PoseStamped()
pose_stamped.header.stamp = self.get_clock().now().to_msg()
pose_stamped.header.frame_id = 'map'
pose_stamped.pose.position.x = 1.0
pose_stamped.pose.position.y = 2.0
pose_stamped.pose.position.z = 0.0
```

---

## Viewing Message Definitions

Use the CLI to explore message structures:

```bash
# Show message definition
ros2 interface show geometry_msgs/msg/Twist

# Output:
# Vector3  linear
#   float64 x
#   float64 y
#   float64 z
# Vector3  angular
#   float64 x
#   float64 y
#   float64 z

# List all messages in a package
ros2 interface package geometry_msgs

# Show service definition
ros2 interface show std_srvs/srv/SetBool
```

---

## Creating Custom Messages

When standard messages don't fit your needs, create custom ones.

### Step 1: Create Package Structure

```bash
cd ~/ros2_ws/src
ros2 pkg create my_custom_msgs --build-type ament_cmake
cd my_custom_msgs
mkdir msg
```

### Step 2: Define Your Message

Create `msg/HumanoidStatus.msg`:

```
# HumanoidStatus.msg - Custom message for humanoid robot status

# Header for timestamp and frame
std_msgs/Header header

# Robot identification
string robot_id
string robot_name

# Battery information
float32 battery_voltage
float32 battery_percentage
bool is_charging

# Motion status
bool is_walking
bool is_balancing
geometry_msgs/Vector3 center_of_mass

# Joint temperatures (Celsius)
float32[] joint_temperatures

# Error codes
int32 error_code
string error_message
```

### Step 3: Update package.xml

Add dependencies:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
```

### Step 4: Update CMakeLists.txt

```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HumanoidStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
```

### Step 5: Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_custom_msgs
source install/setup.bash
```

### Step 6: Use Your Custom Message

```python
from my_custom_msgs.msg import HumanoidStatus
from geometry_msgs.msg import Vector3

# Create custom message
status = HumanoidStatus()
status.header.stamp = self.get_clock().now().to_msg()
status.robot_id = 'HUM-001'
status.robot_name = 'Atlas'

status.battery_voltage = 24.5
status.battery_percentage = 85.0
status.is_charging = False

status.is_walking = True
status.is_balancing = True
status.center_of_mass = Vector3(x=0.0, y=0.0, z=0.95)

status.joint_temperatures = [35.2, 36.1, 34.8, 37.5, 35.9]

status.error_code = 0
status.error_message = 'OK'
```

---

## Message Field Types

Common field types in ROS 2 messages:

| Type | Size | Range/Description |
|------|------|-------------------|
| `bool` | 1 byte | True/False |
| `int8` | 1 byte | -128 to 127 |
| `uint8` | 1 byte | 0 to 255 |
| `int16` | 2 bytes | -32,768 to 32,767 |
| `uint16` | 2 bytes | 0 to 65,535 |
| `int32` | 4 bytes | -2³¹ to 2³¹-1 |
| `uint32` | 4 bytes | 0 to 2³²-1 |
| `int64` | 8 bytes | -2⁶³ to 2⁶³-1 |
| `uint64` | 8 bytes | 0 to 2⁶⁴-1 |
| `float32` | 4 bytes | Single precision float |
| `float64` | 8 bytes | Double precision float |
| `string` | Variable | Text data (UTF-8) |

**Array Types:**
- Fixed size: `int32[5]` - exactly 5 elements
- Variable size: `int32[]` - any number of elements

---

## Best Practices

### 1. Choose Appropriate Types

```python
# Good: Use standard messages when possible
from geometry_msgs.msg import Twist  # Standard velocity

# Avoid: Creating custom message for common data
# Don't create custom velocity message
```

### 2. Include Headers When Needed

```python
# Good: Include header for timestamped/framed data
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = 'base_link'

# Avoid: Missing timestamps when they matter
```

### 3. Use Descriptive Field Names

```python
# Good
string robot_name
float32 battery_percentage

# Avoid
string name
float32 val
```

### 4. Document Custom Messages

```
# Temperature sensor reading
# Units: Celsius
# Range: -40 to 125

float32 temperature
bool is_valid
string sensor_id
```

---

## Hands-On Exercise

### Exercise 1: Robot Status Publisher

Create a node that publishes robot status using multiple message types:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist


class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')

        # TODO: Create publishers for:
        # 1. Battery level (Float32) on /battery
        # 2. Is moving (Bool) on /is_moving
        # 3. Velocity (Twist) on /cmd_vel

        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        # TODO: Publish status data
        pass
```

### Exercise 2: Multi-Sensor Subscriber

Create a node that subscribes to different sensor types and logs the data:

```python
# Subscribe to:
# - /temperature (Float32)
# - /imu (Imu)
# - /joint_states (JointState)
```

---

## Summary

You've learned to:
- Use standard ROS 2 message types
- Work with complex geometric and sensor messages
- Create custom message definitions
- Choose appropriate message types
- Include headers for timestamped data

**Key Takeaways:**
- Prefer standard messages when possible
- Use headers for timestamped/framed data
- Create custom messages for unique requirements
- Explore message definitions with `ros2 interface show`

---

## Next Steps

- [Timers and Callbacks](./timers-callbacks.md) - Advanced callback patterns
- [Parameters](./parameters.md) - Configure nodes with parameters
- [Service Example](/code/ros2-packages/service_example.py) - Request/response communication

---

## Additional Resources

- [ROS 2 Common Interfaces](https://github.com/ros2/common_interfaces)
- [Creating Custom Messages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [Message Type Reference](https://docs.ros2.org/latest/api/)
