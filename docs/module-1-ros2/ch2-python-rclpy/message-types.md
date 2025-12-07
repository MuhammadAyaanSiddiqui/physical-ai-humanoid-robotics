# ROS 2 Message Types

**Module**: The Robotic Nervous System
**Chapter**: Python Integration (rclpy)
**Estimated Time**: 2-2.5 hours
**Difficulty**: Beginner to Intermediate

## Prerequisites

- Understanding of ROS 2 nodes and topics
- Ability to create basic publisher/subscriber nodes
- Python 3.10+ and rclpy installed

## Learning Objectives

By the end of this lesson, you will be able to:

- Understand the structure of ROS 2 messages
- Use standard message types (std_msgs, geometry_msgs, sensor_msgs)
- Inspect message definitions using CLI tools
- Publish and subscribe to complex messages
- Create custom message types for your applications
- Choose appropriate message types for different scenarios

## What are ROS 2 Messages?

**Messages** are the data structures used for communication between ROS 2 nodes. They define:
- The **fields** (data members) and their **types**
- The **structure** (nested messages, arrays)
- The **serialization format** (how data is encoded for transmission)

### Message Definition Files

Messages are defined in `.msg` files using a simple syntax:

```
# my_package/msg/Temperature.msg
float64 value       # Temperature in Celsius
string location     # Where the reading was taken
time stamp          # When the reading was taken
```

These `.msg` files are compiled into Python/C++ classes during package build.

## Standard Message Packages

ROS 2 provides several standard message packages:

| Package | Purpose | Examples |
|---------|---------|----------|
| `std_msgs` | Basic data types | String, Int32, Float64, Bool |
| `geometry_msgs` | Geometric primitives | Point, Pose, Twist, Transform |
| `sensor_msgs` | Sensor data | Image, LaserScan, Imu, PointCloud2 |
| `nav_msgs` | Navigation data | Odometry, Path, OccupancyGrid |
| `std_srvs` | Basic services | Empty, SetBool, Trigger |
| `rcl_interfaces` | ROS internals | ParameterEvent, Log |

## std_msgs - Basic Data Types

### Primitive Types

**String**:
```python
from std_msgs.msg import String

msg = String()
msg.data = "Hello ROS 2"
```

**Int32**:
```python
from std_msgs.msg import Int32

msg = Int32()
msg.data = 42
```

**Float64**:
```python
from std_msgs.msg import Float64

msg = Float64()
msg.data = 3.14159
```

**Bool**:
```python
from std_msgs.msg import Bool

msg = Bool()
msg.data = True
```

### When to Use std_msgs

**Use std_msgs for**:
- ✅ Simple debugging and testing
- ✅ Quick prototypes
- ✅ Single-value data

**Avoid std_msgs for production**:
- ❌ No semantic meaning (what does "Int32" represent?)
- ❌ No metadata (timestamps, frame IDs)
- ❌ Better to use or create meaningful message types

**Example - Bad**:
```python
# What does this represent? Speed? Count? Temperature?
msg = Int32()
msg.data = 100
```

**Example - Good**:
```python
# Clear semantic meaning
from my_msgs.msg import Temperature

msg = Temperature()
msg.value = 25.5
msg.unit = "celsius"
msg.sensor_id = "kitchen_sensor_01"
```

## geometry_msgs - Geometric Primitives

### Point (3D Coordinates)

**Definition**:
```
float64 x
float64 y
float64 z
```

**Usage**:
```python
from geometry_msgs.msg import Point

point = Point()
point.x = 1.5
point.y = 2.0
point.z = 0.0
```

### Pose (Position + Orientation)

**Definition**:
```
Point position
Quaternion orientation
```

**Usage**:
```python
from geometry_msgs.msg import Pose

pose = Pose()
pose.position.x = 1.0
pose.position.y = 2.0
pose.position.z = 0.0

# Quaternion (orientation)
# For 2D: no rotation around x, y
pose.orientation.x = 0.0
pose.orientation.y = 0.0
pose.orientation.z = 0.0
pose.orientation.w = 1.0  # Identity quaternion (no rotation)
```

**Quaternions**:
- Represent 3D rotations
- 4 values: x, y, z, w
- Identity (no rotation): (0, 0, 0, 1)
- For 2D rotation around Z-axis:
  - w = cos(θ/2)
  - z = sin(θ/2)
  - x = y = 0

**Example - 90° rotation around Z**:
```python
import math

angle = math.pi / 2  # 90 degrees
pose.orientation.z = math.sin(angle / 2)  # 0.707
pose.orientation.w = math.cos(angle / 2)  # 0.707
```

### Twist (Linear + Angular Velocity)

**Definition**:
```
Vector3 linear   # meters/second
Vector3 angular  # radians/second
```

**Usage**:
```python
from geometry_msgs.msg import Twist

cmd_vel = Twist()
cmd_vel.linear.x = 0.5    # Move forward at 0.5 m/s
cmd_vel.linear.y = 0.0
cmd_vel.linear.z = 0.0
cmd_vel.angular.x = 0.0
cmd_vel.angular.y = 0.0
cmd_vel.angular.z = 0.3   # Rotate counterclockwise at 0.3 rad/s
```

**Common Use**: Robot velocity commands (`/cmd_vel` topic)

**Example - Move in a Circle**:
```python
# Constant forward speed + constant rotation = circular motion
cmd_vel.linear.x = 1.0    # 1 m/s forward
cmd_vel.angular.z = 0.5   # 0.5 rad/s rotation
# Result: Circle with radius = linear/angular = 2 meters
```

## sensor_msgs - Sensor Data

### Image (Camera Data)

**Definition** (simplified):
```
std_msgs/Header header
uint32 height          # rows
uint32 width           # columns
string encoding        # "rgb8", "bgr8", "mono8", etc.
uint8 is_bigendian
uint32 step            # row length in bytes
uint8[] data           # pixel data
```

**Usage**:
```python
from sensor_msgs.msg import Image

img_msg = Image()
img_msg.header.stamp = self.get_clock().now().to_msg()
img_msg.header.frame_id = "camera_frame"
img_msg.height = 480
img_msg.width = 640
img_msg.encoding = "rgb8"
img_msg.step = img_msg.width * 3  # 3 bytes per pixel (RGB)
img_msg.data = [...]  # Flattened pixel data
```

**With OpenCV**:
```python
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()

# OpenCV image to ROS message
cv_image = cv2.imread("image.jpg")
img_msg = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

# ROS message to OpenCV image
cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
```

### LaserScan (LiDAR Data)

**Definition** (simplified):
```
std_msgs/Header header
float32 angle_min       # Start angle (radians)
float32 angle_max       # End angle (radians)
float32 angle_increment # Angular distance between measurements
float32 range_min       # Minimum range value (meters)
float32 range_max       # Maximum range value (meters)
float32[] ranges        # Range measurements (meters)
float32[] intensities   # Intensity measurements
```

**Usage**:
```python
from sensor_msgs.msg import LaserScan

scan = LaserScan()
scan.header.stamp = self.get_clock().now().to_msg()
scan.header.frame_id = "laser_frame"
scan.angle_min = -1.57  # -90 degrees
scan.angle_max = 1.57   # +90 degrees
scan.angle_increment = 0.01  # ~0.57 degrees
scan.range_min = 0.1
scan.range_max = 10.0
scan.ranges = [3.2, 3.1, 3.0, ...]  # Distance to obstacles
```

**Reading LaserScan Data**:
```python
def scan_callback(self, msg):
    # Find closest obstacle
    min_distance = min(msg.ranges)
    min_index = msg.ranges.index(min_distance)

    # Calculate angle of closest obstacle
    angle = msg.angle_min + min_index * msg.angle_increment

    self.get_logger().info(f'Closest obstacle: {min_distance:.2f}m at {angle:.2f} rad')
```

### Imu (Inertial Measurement Unit)

**Definition** (simplified):
```
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```

**Usage**:
```python
from sensor_msgs.msg import Imu

imu_msg = Imu()
imu_msg.header.stamp = self.get_clock().now().to_msg()
imu_msg.header.frame_id = "imu_link"

# Orientation (quaternion)
imu_msg.orientation.x = 0.0
imu_msg.orientation.y = 0.0
imu_msg.orientation.z = 0.0
imu_msg.orientation.w = 1.0

# Angular velocity (rad/s)
imu_msg.angular_velocity.x = 0.0
imu_msg.angular_velocity.y = 0.0
imu_msg.angular_velocity.z = 0.1

# Linear acceleration (m/s²)
imu_msg.linear_acceleration.x = 0.0
imu_msg.linear_acceleration.y = 0.0
imu_msg.linear_acceleration.z = 9.81  # Gravity
```

## Header - Timestamps and Frame IDs

Most sensor messages include a **Header**:

```
builtin_interfaces/Time stamp
string frame_id
```

**Purpose**:
- `stamp`: When the data was captured
- `frame_id`: Which coordinate frame the data is in

**Usage**:
```python
from std_msgs.msg import Header

header = Header()
header.stamp = self.get_clock().now().to_msg()
header.frame_id = "base_link"
```

**Why Headers Matter**:
- Synchronize data from multiple sensors
- Transform between coordinate frames (TF2)
- Detect stale data (check timestamp freshness)

## nav_msgs - Navigation Messages

### Odometry (Robot Position and Velocity)

**Definition** (simplified):
```
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

**Usage**:
```python
from nav_msgs.msg import Odometry

odom = Odometry()
odom.header.stamp = self.get_clock().now().to_msg()
odom.header.frame_id = "odom"
odom.child_frame_id = "base_link"

# Position
odom.pose.pose.position.x = 1.0
odom.pose.pose.position.y = 2.0
odom.pose.pose.orientation.w = 1.0

# Velocity
odom.twist.twist.linear.x = 0.5
odom.twist.twist.angular.z = 0.1
```

## Inspecting Message Definitions

### Show Message Structure

```bash
ros2 interface show geometry_msgs/msg/Twist
```

**Output**:
```
# This expresses velocity in free space broken into its linear and angular parts.

Vector3 linear
        float64 x
        float64 y
        float64 z
Vector3 angular
        float64 x
        float64 y
        float64 z
```

### List All Messages in a Package

```bash
ros2 interface package sensor_msgs
```

**Output**:
```
sensor_msgs/msg/Image
sensor_msgs/msg/LaserScan
sensor_msgs/msg/Imu
sensor_msgs/msg/PointCloud2
...
```

### Find Packages with Specific Message

```bash
ros2 interface list | grep Twist
```

**Output**:
```
geometry_msgs/msg/Twist
geometry_msgs/msg/TwistStamped
geometry_msgs/msg/TwistWithCovariance
...
```

## Creating Custom Messages

### Step 1: Create Package with msg Directory

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_robot_msgs
cd my_robot_msgs
mkdir msg
```

### Step 2: Define Message

Create `msg/BatteryStatus.msg`:

```
# Battery status information
float32 voltage           # Battery voltage in volts
float32 current           # Current draw in amps
float32 percentage        # Battery level 0-100
bool is_charging          # True if charging
string health             # "good", "fair", "poor"
builtin_interfaces/Time last_update
```

### Step 3: Update package.xml

Add these lines to `package.xml`:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### Step 4: Update CMakeLists.txt

Add:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryStatus.msg"
)
```

### Step 5: Build Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_msgs
source install/setup.bash
```

### Step 6: Use Custom Message

```python
from my_robot_msgs.msg import BatteryStatus

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher_ = self.create_publisher(BatteryStatus, 'battery_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = BatteryStatus()
        msg.voltage = 12.6
        msg.current = 2.5
        msg.percentage = 85.0
        msg.is_charging = False
        msg.health = "good"
        msg.last_update = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)
```

## Nested Messages

Messages can contain other messages:

**Example - PoseStamped**:
```
std_msgs/Header header
geometry_msgs/Pose pose
```

**Usage**:
```python
from geometry_msgs.msg import PoseStamped

pose_stamped = PoseStamped()
pose_stamped.header.stamp = self.get_clock().now().to_msg()
pose_stamped.header.frame_id = "map"
pose_stamped.pose.position.x = 1.0
pose_stamped.pose.position.y = 2.0
pose_stamped.pose.orientation.w = 1.0
```

## Arrays in Messages

### Fixed-Size Arrays

**Definition**:
```
float64[3] position  # Exactly 3 elements
```

**Usage**:
```python
msg.position = [1.0, 2.0, 3.0]
```

### Variable-Size Arrays

**Definition**:
```
float32[] ranges  # Any number of elements
```

**Usage**:
```python
msg.ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
# Or
msg.ranges = list(range(100))  # 100 elements
```

## Message Constants

Messages can define constants:

**Example**:
```
# Status constants
uint8 STATUS_OK=0
uint8 STATUS_WARNING=1
uint8 STATUS_ERROR=2

uint8 status
string message
```

**Usage**:
```python
from my_msgs.msg import SystemStatus

msg = SystemStatus()
msg.status = SystemStatus.STATUS_WARNING
msg.message = "Low battery"
```

## Best Practices

### 1. Use Stamped Messages

Prefer messages with headers (timestamps and frame IDs):

```python
# Good: PoseStamped (has header)
from geometry_msgs.msg import PoseStamped

# Okay but less useful: Pose (no header)
from geometry_msgs.msg import Pose
```

### 2. Choose Appropriate Types

```python
# Good: Semantic meaning
from geometry_msgs.msg import Twist  # Velocity command

# Bad: Generic type
from std_msgs.msg import Float64MultiArray  # What does this represent?
```

### 3. Document Custom Messages

```
# msg/RobotCommand.msg

# Command type
uint8 CMD_STOP=0
uint8 CMD_MOVE=1
uint8 CMD_ROTATE=2

uint8 command_type       # Type of command (use constants above)
float32 linear_velocity  # Forward speed in m/s
float32 angular_velocity # Rotation rate in rad/s
float32 duration         # How long to execute command (seconds)
```

### 4. Use Covariance When Available

For sensor data with uncertainty:

```python
# PoseWithCovariance includes uncertainty estimate
odom.pose.covariance[0] = 0.01  # x position variance
odom.pose.covariance[7] = 0.01  # y position variance
```

## Common Message Patterns

### Pattern 1: Sensor Data Pipeline

```python
# Sensor Node
sensor_msg = LaserScan()
sensor_msg.header.stamp = self.get_clock().now().to_msg()
sensor_pub.publish(sensor_msg)

# Processing Node
def sensor_callback(self, msg):
    # Use msg.header.stamp for synchronization
    age = (self.get_clock().now().nanoseconds -
           msg.header.stamp.sec * 1e9 - msg.header.stamp.nanosec) / 1e9
    if age > 1.0:
        self.get_logger().warn('Stale sensor data!')
```

### Pattern 2: Command-Status Pattern

```python
# Command publisher
cmd = Twist()
cmd.linear.x = 1.0
cmd_pub.publish(cmd)

# Status publisher (separate topic)
status = String()
status.data = "executing"
status_pub.publish(status)
```

## Practice Exercises

### Exercise 1: Turtle Control

Write a node that publishes `Twist` messages to control turtlesim in a square pattern.

**Hints**:
- Use `/turtle1/cmd_vel` topic
- Alternate between forward motion and rotation
- Use timers to control timing

### Exercise 2: Fake Sensor Publisher

Create a node that publishes fake `LaserScan` data with random ranges.

**Hints**:
- Use `random.uniform(0.5, 10.0)` for ranges
- Set angle_min=-π, angle_max=π
- 360 range readings

### Exercise 3: Custom Message

Create a custom message for a robot's system status including:
- CPU usage (float)
- Memory usage (float)
- Disk space (float)
- Uptime (duration)

Build and publish this message.

## Key Takeaways

- **Messages** define data structures for ROS 2 communication
- **std_msgs** provides basic types (use sparingly in production)
- **geometry_msgs** provides geometric primitives (Point, Pose, Twist)
- **sensor_msgs** provides sensor data types (Image, LaserScan, Imu)
- **Headers** include timestamps and coordinate frames
- **Custom messages** enable domain-specific data structures
- Use `ros2 interface show` to inspect message definitions

## What's Next?

Now that you understand message types:

- **Next Lesson**: [Timers and Callbacks](./timers-callbacks.md) - Event-driven programming
- **Related**: [Parameters](./parameters.md) - Runtime configuration

## Further Reading

- [ROS 2 Interfaces Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)
- [Common ROS 2 Interfaces](https://github.com/ros2/common_interfaces)
- [Creating Custom Messages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

---

**Checkpoint**: You can now work with standard and custom ROS 2 message types!
