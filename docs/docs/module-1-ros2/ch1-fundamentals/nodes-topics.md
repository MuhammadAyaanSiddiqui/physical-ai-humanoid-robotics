---
sidebar_position: 2
---

# Nodes and Topics

## Overview

Nodes and topics are the foundation of ROS 2's communication architecture. Understanding how nodes exchange data through topics is essential for building distributed robot systems.

## What is a Node?

A **node** is an independent process that performs a specific computation or task. Examples include:

- **Sensor Driver**: Reads camera data and publishes images
- **Motion Planner**: Calculates paths for robot navigation
- **Controller**: Sends velocity commands to motors
- **Visualizer**: Displays robot state in RViz

### Key Characteristics

- **Independent Processes**: Each node runs in its own process
- **Single Responsibility**: Each node focuses on one task
- **Discoverable**: Nodes automatically find each other on the network
- **Language Agnostic**: Nodes can be written in C++, Python, or other supported languages

### Node Lifecycle

```
┌─────────────┐
│ Unconfigured│
└──────┬──────┘
       │ configure()
┌──────▼──────┐
│  Inactive   │
└──────┬──────┘
       │ activate()
┌──────▼──────┐
│   Active    │◄──┐
└──────┬──────┘   │
       │ cleanup()│
┌──────▼──────┐   │
│  Finalized  ├───┘
└─────────────┘
```

## What is a Topic?

A **topic** is a named channel for message communication. Nodes publish messages to topics and subscribe to topics to receive messages.

### Publisher-Subscriber Pattern

```
┌──────────────┐      /camera/image      ┌──────────────┐
│   Camera     │─────────────────────────►│  Detector    │
│   Driver     │                          │   Node       │
│  (Publisher) │                          │ (Subscriber) │
└──────────────┘                          └──────────────┘
```

### Key Characteristics

- **Anonymous**: Publishers don't know who subscribes
- **Many-to-Many**: Multiple publishers and subscribers on one topic
- **Typed**: Each topic has a specific message type
- **Buffered**: Messages are queued if subscribers are slow

## Message Types

ROS 2 uses **strongly typed messages** defined in `.msg` files.

### Standard Message Types

Common message types from `std_msgs`:

```python
std_msgs/String         # Text data
std_msgs/Int32          # Integer
std_msgs/Float64        # Floating point
std_msgs/Bool           # Boolean
std_msgs/Header         # Timestamp and frame info
```

### Sensor Message Types

From `sensor_msgs`:

```python
sensor_msgs/Image       # Camera images
sensor_msgs/LaserScan   # LiDAR data
sensor_msgs/Imu         # Inertial measurement unit
sensor_msgs/JointState  # Robot joint positions
```

### Geometry Message Types

From `geometry_msgs`:

```python
geometry_msgs/Twist         # Linear and angular velocity
geometry_msgs/Pose          # Position and orientation
geometry_msgs/PoseStamped   # Pose with timestamp
geometry_msgs/Transform     # Coordinate transformation
```

## Creating a Simple Publisher

Here's a minimal publisher node in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown

1. **Import Libraries**: `rclpy` (ROS 2 Python client) and `Node` base class
2. **Create Node Class**: Inherit from `Node` and name it
3. **Create Publisher**: `create_publisher(MessageType, 'topic_name', queue_size)`
4. **Create Timer**: Calls `timer_callback()` every 0.5 seconds
5. **Publish Message**: Create message, populate data, and publish
6. **Spin**: Keep node running and processing callbacks

## Creating a Simple Subscriber

Here's a minimal subscriber node in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown

1. **Create Subscriber**: `create_subscription(MessageType, 'topic_name', callback, queue_size)`
2. **Define Callback**: Function called when a message is received
3. **Process Message**: Access message data via `msg.data`

## Quality of Service (QoS)

QoS policies control how messages are delivered. ROS 2 provides flexible QoS settings for different use cases.

### Common QoS Profiles

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Reliable delivery (TCP-like)
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Best effort (UDP-like, faster)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

### When to Use Each Profile

| Profile | Use Case | Example |
|---------|----------|---------|
| **Reliable** | Critical commands | Motor control, navigation goals |
| **Best Effort** | High-frequency sensor data | Camera images, LiDAR scans |
| **Transient Local** | Late joiners need history | Map data, configuration |
| **Volatile** | Only current data matters | Velocity commands |

## Topic Names and Namespaces

### Naming Conventions

- **Lowercase with underscores**: `camera_image`, `joint_states`
- **Hierarchical structure**: `/robot1/camera/image`, `/robot1/lidar/scan`
- **Descriptive names**: Use meaningful names that describe the data

### Global vs Relative Names

```bash
# Global name (starts with /)
/camera/image

# Relative name (resolved within node namespace)
camera/image

# Private name (prefixed with node name)
~/my_parameter
```

### Using Namespaces

Launch nodes in namespaces to avoid conflicts:

```bash
# Without namespace
ros2 run my_package my_node

# With namespace
ros2 run my_package my_node --ros-args -r __ns:=/robot1
```

## Introspection with CLI Tools

### List Active Topics

```bash
ros2 topic list
```

Output:
```
/parameter_events
/rosout
/topic
```

### Show Topic Info

```bash
ros2 topic info /topic
```

Output:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscriber count: 1
```

### View Topic Data

```bash
# Print messages to terminal
ros2 topic echo /topic

# Print messages once
ros2 topic echo --once /topic
```

### Publish from Command Line

```bash
# Publish a single message
ros2 topic pub /topic std_msgs/msg/String "data: 'Hello ROS 2'"

# Publish at 10 Hz
ros2 topic pub --rate 10 /topic std_msgs/msg/String "data: 'Hello ROS 2'"
```

### Measure Topic Frequency

```bash
ros2 topic hz /topic
```

Output:
```
average rate: 2.000
  min: 0.500s max: 0.500s std dev: 0.00001s window: 10
```

### View Topic Bandwidth

```bash
ros2 topic bw /camera/image
```

Output:
```
Subscribed to [/camera/image]
average: 20.5MB/s
  mean: 2.05MB/s min: 2.00MB/s max: 2.10MB/s window: 10
```

## Visualizing Node Graph

Use `rqt_graph` to visualize nodes and topics:

```bash
ros2 run rqt_graph rqt_graph
```

This opens a GUI showing:
- **Nodes** (ovals)
- **Topics** (rectangles)
- **Connections** (arrows from publishers to subscribers)

## Hands-On Exercise

### Exercise 1: Create a Temperature Publisher

Create a node that publishes simulated temperature readings every second.

**Requirements:**
1. Node name: `temperature_sensor`
2. Topic: `/sensors/temperature`
3. Message type: `std_msgs/Float64`
4. Publish range: 20.0 to 30.0 degrees Celsius

<details>
<summary>Solution</summary>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher_ = self.create_publisher(Float64, '/sensors/temperature', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        msg.data = random.uniform(20.0, 30.0)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Temperature: {msg.data:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    sensor = TemperatureSensor()
    rclpy.spin(sensor)
    sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run with:
```bash
python3 temperature_sensor.py
```

Monitor with:
```bash
ros2 topic echo /sensors/temperature
ros2 topic hz /sensors/temperature
```
</details>

### Exercise 2: Create a Temperature Monitor

Create a subscriber node that monitors temperature and warns if it exceeds 28°C.

**Requirements:**
1. Node name: `temperature_monitor`
2. Subscribe to: `/sensors/temperature`
3. Log warning if temperature > 28.0

<details>
<summary>Solution</summary>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        self.subscription = self.create_subscription(
            Float64,
            '/sensors/temperature',
            self.temperature_callback,
            10)

    def temperature_callback(self, msg):
        temp = msg.data
        if temp > 28.0:
            self.get_logger().warn(f'High temperature: {temp:.2f}°C')
        else:
            self.get_logger().info(f'Temperature OK: {temp:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    monitor = TemperatureMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run both nodes in separate terminals:
```bash
# Terminal 1
python3 temperature_sensor.py

# Terminal 2
python3 temperature_monitor.py
```
</details>

## Best Practices

1. **Use Descriptive Names**: Topic names should clearly describe the data
2. **Keep Messages Small**: Large messages increase latency
3. **Choose Appropriate QoS**: Match QoS to your use case
4. **Log Meaningfully**: Use `get_logger()` for debugging
5. **Handle Shutdown Gracefully**: Always call `destroy_node()` and `shutdown()`
6. **One Node, One Purpose**: Keep nodes focused on a single task
7. **Namespace Your Topics**: Use hierarchical names to organize topics

## Common Pitfalls

### Pitfall 1: Mismatched Message Types

**Problem**: Subscriber expects `std_msgs/String`, publisher sends `std_msgs/Int32`

**Solution**: Ensure publisher and subscriber use the same message type

### Pitfall 2: Topic Name Typos

**Problem**: Publisher sends to `/camera/imag`, subscriber listens to `/camera/image`

**Solution**: Use constants or configuration files for topic names

### Pitfall 3: Blocking Callbacks

**Problem**: Subscriber callback takes too long, messages are dropped

**Solution**: Keep callbacks fast, offload heavy processing to separate threads

### Pitfall 4: Forgetting to Spin

**Problem**: Node doesn't receive callbacks because `spin()` wasn't called

**Solution**: Always call `rclpy.spin(node)` in your main function

## Next Steps

Now that you understand nodes and topics:

1. Learn about [Services and Actions](./services-actions.md) for request-response patterns
2. Explore [CLI Tools](./cli-tools.md) for debugging and introspection
3. Build your [First Python Node](../ch2-python-rclpy/first-node.md) from scratch

## Resources

- [ROS 2 Concepts: Nodes](https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html)
- [ROS 2 Concepts: Topics](https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html)
- [ROS 2 QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [Understanding ROS 2 Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

---

**Prerequisites**: ROS 2 Humble installed
**Estimated Time**: 45-60 minutes
**Learning Outcomes**:
- Understand nodes as independent processes
- Create publishers and subscribers in Python
- Configure Quality of Service policies
- Use CLI tools to inspect topics
- Visualize node graphs with rqt_graph
