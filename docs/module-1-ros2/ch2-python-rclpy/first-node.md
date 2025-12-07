# Creating Your First ROS 2 Node in Python

**Module**: The Robotic Nervous System
**Chapter**: Python Integration (rclpy)
**Estimated Time**: 2-3 hours
**Difficulty**: Beginner

## Prerequisites

- ROS 2 Humble installed and sourced
- Python 3.10+ installed
- Understanding of ROS 2 nodes and topics
- Basic Python knowledge (classes, functions, imports)

## Learning Objectives

By the end of this lesson, you will be able to:

- Write a minimal ROS 2 publisher node in Python
- Understand the structure of an rclpy node
- Initialize and run nodes using the rclpy library
- Publish messages to topics programmatically
- Use timers for periodic message publishing
- Run and test your custom nodes

## Introduction to rclpy

**rclpy** is the ROS 2 Python client library. It provides Python APIs for:
- Creating nodes
- Publishing and subscribing to topics
- Creating service servers and clients
- Managing timers and callbacks
- Handling parameters

### Why Python for ROS 2?

**Advantages**:
- ✅ Rapid prototyping and development
- ✅ Easier to learn than C++
- ✅ Excellent for high-level logic and AI integration
- ✅ Rich ecosystem (NumPy, OpenCV, TensorFlow)

**Disadvantages**:
- ❌ Slower than C++ (but sufficient for most tasks)
- ❌ Higher memory usage
- ❌ Not ideal for real-time control loops

**Best Use Cases**:
- Navigation planning
- Computer vision processing
- Machine learning inference
- High-level decision making
- Prototyping and testing

## Anatomy of a ROS 2 Python Node

A ROS 2 Python node consists of:

1. **Imports**: rclpy and message types
2. **Node Class**: Inherits from `rclpy.node.Node`
3. **Constructor**: Initialize publishers, subscribers, timers
4. **Callback Methods**: Handle events (timers, incoming messages)
5. **Main Function**: Initialize rclpy, create node, spin
6. **Cleanup**: Destroy node and shutdown rclpy

### Basic Template

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Initialize publishers, subscribers, timers here

    def callback_function(self):
        # Handle events here
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-On: Minimal Publisher Node

Let's create a simple publisher that sends "Hello ROS 2" messages.

### Step 1: Create the Node File

Create a new file called `minimal_publisher.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal ROS 2 publisher node that publishes string messages.
    """

    def __init__(self):
        # Initialize the node with name 'minimal_publisher'
        super().__init__('minimal_publisher')

        # Create a publisher
        # Arguments: message_type, topic_name, queue_size
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create a timer that calls timer_callback every 1.0 seconds
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for message numbering
        self.i = 0

        self.get_logger().info('Minimal Publisher has been started')

    def timer_callback(self):
        """
        This function is called every time the timer expires.
        """
        # Create a String message
        msg = String()
        msg.data = f'Hello ROS 2: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter
        self.i += 1


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of the node
    minimal_publisher = MinimalPublisher()

    # Spin the node (keep it alive and processing callbacks)
    rclpy.spin(minimal_publisher)

    # Clean shutdown (this code runs when spin is interrupted)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 2: Make the File Executable

```bash
chmod +x minimal_publisher.py
```

### Step 3: Run the Node

```bash
# Make sure ROS 2 is sourced
source /opt/ros/humble/setup.bash

# Run the node
python3 minimal_publisher.py
```

**Expected Output**:
```
[INFO] [minimal_publisher]: Minimal Publisher has been started
[INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 0"
[INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 1"
[INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 2"
[INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 3"
...
```

Press `Ctrl+C` to stop the node.

### Step 4: Verify with CLI Tools

While the publisher is running, open a new terminal and check:

```bash
# List running nodes
ros2 node list
# Output: /minimal_publisher

# List active topics
ros2 topic list
# Output: /chatter, /parameter_events, /rosout

# Echo messages
ros2 topic echo /chatter
# Output: data: 'Hello ROS 2: 10'
#         ---
#         data: 'Hello ROS 2: 11'
```

## Code Breakdown

Let's understand each part of the code:

### 1. Shebang and Imports

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```

- **Shebang**: `#!/usr/bin/env python3` allows running the file directly
- **rclpy**: Core ROS 2 Python library
- **Node**: Base class for all ROS 2 nodes
- **String**: Message type for text data

### 2. Node Class Definition

```python
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
```

- Inherits from `Node` base class
- `super().__init__('minimal_publisher')` registers the node with name `minimal_publisher`
- Node names must be unique in the ROS 2 network

### 3. Creating a Publisher

```python
self.publisher_ = self.create_publisher(String, 'chatter', 10)
```

**Parameters**:
- `String`: Message type (from `std_msgs.msg`)
- `'chatter'`: Topic name
- `10`: Queue size (buffer for messages if subscriber is slow)

**Queue Size**:
- If publisher is faster than subscriber, messages are queued
- When queue is full, oldest messages are dropped
- Typical values: 10 for most cases, 1 for "latest only", 100+ for critical data

### 4. Creating a Timer

```python
timer_period = 1.0  # seconds
self.timer = self.create_timer(timer_period, self.timer_callback)
```

- Timer calls `self.timer_callback` every 1.0 seconds
- Timers are the recommended way to publish at fixed rates
- Timers run in the node's executor thread

### 5. Timer Callback

```python
def timer_callback(self):
    msg = String()
    msg.data = f'Hello ROS 2: {self.i}'
    self.publisher_.publish(msg)
    self.get_logger().info(f'Publishing: "{msg.data}"')
    self.i += 1
```

- Creates a new `String` message
- Sets the `data` field
- Publishes the message
- Logs to console (visible with `ros2 run` or `ros2 topic echo /rosout`)
- Increments counter

### 6. Logging

```python
self.get_logger().info('Message')
```

**Log Levels**:
```python
self.get_logger().debug('Debug message')    # Detailed debugging info
self.get_logger().info('Info message')      # General information
self.get_logger().warn('Warning message')   # Warning (node still works)
self.get_logger().error('Error message')    # Error occurred
self.get_logger().fatal('Fatal message')    # Critical failure
```

Logs are published to `/rosout` topic and printed to console.

### 7. Main Function

```python
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

**Step-by-step**:
1. `rclpy.init()`: Initialize ROS 2 context (must be called before creating nodes)
2. `MinimalPublisher()`: Create node instance (calls `__init__`)
3. `rclpy.spin()`: Keep node alive, process callbacks (blocks until Ctrl+C)
4. `destroy_node()`: Clean up node resources
5. `rclpy.shutdown()`: Shutdown ROS 2 context

**Execution Flow**:
```
main()
  ├─ rclpy.init()
  ├─ Create node
  │   └─ __init__() runs
  │       ├─ Create publisher
  │       └─ Create timer
  ├─ rclpy.spin() [BLOCKS HERE]
  │   └─ Timer fires every 1.0s → timer_callback() runs
  │       └─ Publish message
  ├─ [User presses Ctrl+C]
  ├─ destroy_node()
  └─ rclpy.shutdown()
```

## Variations

### Publishing at Different Rates

**Fast (10 Hz)**:
```python
timer_period = 0.1  # 100ms = 10 Hz
self.timer = self.create_timer(timer_period, self.timer_callback)
```

**Slow (0.5 Hz)**:
```python
timer_period = 2.0  # 2 seconds = 0.5 Hz
self.timer = self.create_timer(timer_period, self.timer_callback)
```

### Publishing Different Message Types

**Integer Messages**:
```python
from std_msgs.msg import Int32

self.publisher_ = self.create_publisher(Int32, 'counter', 10)

def timer_callback(self):
    msg = Int32()
    msg.data = self.i
    self.publisher_.publish(msg)
    self.i += 1
```

**Float Messages**:
```python
from std_msgs.msg import Float64

self.publisher_ = self.create_publisher(Float64, 'temperature', 10)

def timer_callback(self):
    msg = Float64()
    msg.data = 25.5  # Temperature in Celsius
    self.publisher_.publish(msg)
```

### Publishing Without Timers (Event-Driven)

```python
class EventPublisher(Node):
    def __init__(self):
        super().__init__('event_publisher')
        self.publisher_ = self.create_publisher(String, 'events', 10)

    def publish_event(self, event_msg):
        """Call this method to publish an event"""
        msg = String()
        msg.data = event_msg
        self.publisher_.publish(msg)
        self.get_logger().info(f'Event: {event_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = EventPublisher()

    # Publish events manually
    node.publish_event('System started')
    node.publish_event('Sensors initialized')

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Common Patterns

### Pattern 1: Publisher with Configuration

```python
class ConfigurablePublisher(Node):
    def __init__(self, topic_name='chatter', rate=1.0):
        super().__init__('configurable_publisher')
        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Message {self.counter}'
        self.publisher_.publish(msg)
        self.counter += 1

# Usage
node = ConfigurablePublisher(topic_name='my_topic', rate=10.0)  # 10 Hz
```

### Pattern 2: Multiple Publishers

```python
class MultiPublisher(Node):
    def __init__(self):
        super().__init__('multi_publisher')

        # Create multiple publishers
        self.status_pub = self.create_publisher(String, 'robot/status', 10)
        self.health_pub = self.create_publisher(Float64, 'robot/health', 10)

        # Timer for status (1 Hz)
        self.create_timer(1.0, self.status_callback)

        # Timer for health (0.5 Hz)
        self.create_timer(2.0, self.health_callback)

    def status_callback(self):
        msg = String()
        msg.data = 'Running'
        self.status_pub.publish(msg)

    def health_callback(self):
        msg = Float64()
        msg.data = 98.5  # Health percentage
        self.health_pub.publish(msg)
```

## Troubleshooting

### Issue 1: ModuleNotFoundError: No module named 'rclpy'

**Error**:
```
ModuleNotFoundError: No module named 'rclpy'
```

**Solution**:
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Verify rclpy is available
python3 -c "import rclpy; print('rclpy found')"
```

### Issue 2: Node Not Appearing in ros2 node list

**Possible Causes**:
1. Node crashed during initialization
2. Different ROS_DOMAIN_ID
3. Network configuration issues

**Debug**:
```bash
# Check for Python errors in terminal
# Look for stack traces

# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Run ros2 doctor
ros2 doctor
```

### Issue 3: Messages Not Being Published

**Debug Steps**:
```bash
# 1. Check node is running
ros2 node list

# 2. Check topic exists
ros2 topic list

# 3. Check publisher count
ros2 topic info /chatter
# Should show: Publisher count: 1

# 4. Monitor publishing rate
ros2 topic hz /chatter
```

### Issue 4: Permission Denied

**Error**:
```
bash: ./minimal_publisher.py: Permission denied
```

**Solution**:
```bash
chmod +x minimal_publisher.py
```

Or run with Python explicitly:
```bash
python3 minimal_publisher.py
```

## Practice Exercises

### Exercise 1: Countdown Publisher

Modify the minimal publisher to count down from 10 to 0, then stop.

**Hints**:
- Initialize counter to 10
- Decrement instead of increment
- Call `rclpy.shutdown()` when counter reaches 0

<details>
<summary>Solution</summary>

```python
def __init__(self):
    super().__init__('countdown_publisher')
    self.publisher_ = self.create_publisher(String, 'countdown', 10)
    self.timer = self.create_timer(1.0, self.timer_callback)
    self.counter = 10

def timer_callback(self):
    if self.counter >= 0:
        msg = String()
        msg.data = f'Countdown: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)
        self.counter -= 1
    else:
        self.get_logger().info('Countdown complete!')
        rclpy.shutdown()
```
</details>

### Exercise 2: Random Number Publisher

Create a publisher that publishes random integers between 1-100 every 0.5 seconds.

**Hints**:
- `import random`
- `random.randint(1, 100)`
- Use `Int32` message type

<details>
<summary>Solution</summary>

```python
import random
from std_msgs.msg import Int32

class RandomPublisher(Node):
    def __init__(self):
        super().__init__('random_publisher')
        self.publisher_ = self.create_publisher(Int32, 'random_numbers', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Int32()
        msg.data = random.randint(1, 100)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Random: {msg.data}')
```
</details>

### Exercise 3: Timestamp Publisher

Create a publisher that publishes the current timestamp every second.

**Hints**:
- `from std_msgs.msg import String`
- `import time`
- `time.strftime('%Y-%m-%d %H:%M:%S')`

<details>
<summary>Solution</summary>

```python
import time

class TimestampPublisher(Node):
    def __init__(self):
        super().__init__('timestamp_publisher')
        self.publisher_ = self.create_publisher(String, 'timestamp', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = time.strftime('%Y-%m-%d %H:%M:%S')
        self.publisher_.publish(msg)
        self.get_logger().info(f'Time: {msg.data}')
```
</details>

## Best Practices

### 1. Use Descriptive Node Names

```python
# Good
super().__init__('camera_driver')
super().__init__('lidar_processor')

# Bad
super().__init__('node1')
super().__init__('my_node')
```

### 2. Initialize Members in __init__

```python
def __init__(self):
    super().__init__('my_node')

    # Good: Initialize all attributes in constructor
    self.publisher_ = self.create_publisher(...)
    self.counter = 0
    self.timer = self.create_timer(...)
```

### 3. Use Trailing Underscore for Publishers/Subscribers

```python
# Convention: trailing underscore for ROS communication objects
self.publisher_ = self.create_publisher(...)
self.subscription_ = self.create_subscription(...)

# Regular attributes without underscore
self.counter = 0
self.data = []
```

### 4. Always Log Important Events

```python
self.get_logger().info('Node started')
self.get_logger().warn('Sensor not responding')
self.get_logger().error('Failed to read data')
```

### 5. Clean Shutdown

```python
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Key Takeaways

- **rclpy** is the Python client library for ROS 2
- All nodes inherit from `rclpy.node.Node`
- Publishers are created with `create_publisher(msg_type, topic, queue_size)`
- Timers enable periodic publishing at fixed rates
- `rclpy.spin()` keeps the node alive and processes callbacks
- Always call `rclpy.init()` before creating nodes
- Always call `destroy_node()` and `rclpy.shutdown()` for cleanup

## What's Next?

Now that you can create publisher nodes, learn about:

- **Next Lesson**: [Message Types](./message-types.md) - Working with different ROS 2 messages
- **Related**: [Subscribers](./timers-callbacks.md) - Receiving messages from topics

## Further Reading

- [Official rclpy Documentation](https://docs.ros2.org/latest/api/rclpy/)
- [ROS 2 Python Publisher Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [rclpy API Reference](https://docs.ros2.org/latest/api/rclpy/api.html)

---

**Checkpoint**: You can now create and run ROS 2 publisher nodes in Python!
