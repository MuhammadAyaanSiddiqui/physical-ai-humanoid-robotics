# Your First ROS 2 Node in Python

## Introduction

In this lesson, you'll create your first ROS 2 node using Python and the `rclpy` library. You'll build a simple publisher node that sends messages to a topic, and a subscriber node that receives those messages. This hands-on experience will give you a solid foundation for building more complex robotic systems.

## Learning Objectives

By the end of this lesson, you will be able to:
- Create a basic ROS 2 publisher node in Python
- Create a basic ROS 2 subscriber node in Python
- Understand the structure of a ROS 2 Python node
- Run and test your nodes using ROS 2 CLI tools
- Understand the node lifecycle and initialization

## Prerequisites

- ROS 2 Humble installed and sourced
- Python 3.10+ installed
- Basic Python programming knowledge
- Understanding of ROS 2 concepts (nodes, topics, messages)

---

## Understanding rclpy

`rclpy` is the ROS 2 Python client library. It provides:
- **Node creation**: Base class for creating ROS 2 nodes
- **Communication**: Publishers, subscribers, services, actions
- **Utilities**: Logging, parameters, timers, and more

---

## Creating a Simple Publisher Node

### Step 1: Create the Python File

Create a file named `minimal_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('minimal_publisher')

        # Create a publisher on the 'chatter' topic
        # Queue size of 10 means it can hold 10 messages before dropping old ones
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create a timer that calls the callback every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for message numbering
        self.i = 0

        self.get_logger().info('Minimal Publisher has been started')

    def timer_callback(self):
        # Create a message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter
        self.i += 1


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Create the node
    minimal_publisher = MinimalPublisher()

    # Keep the node running (spin)
    # This blocks until the node is shut down (Ctrl+C)
    rclpy.spin(minimal_publisher)

    # Clean shutdown
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Understanding the Code

Let's break down each component:

#### 1. Imports
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```
- `rclpy`: The ROS 2 Python client library
- `Node`: Base class for all ROS 2 nodes
- `String`: A standard message type for text data

#### 2. Node Class Definition
```python
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
```
- Inherit from the `Node` base class
- Call `super().__init__()` with the node name
- The node name must be unique in the ROS 2 network

#### 3. Creating a Publisher
```python
self.publisher_ = self.create_publisher(String, 'chatter', 10)
```
- **First argument**: Message type (`String`)
- **Second argument**: Topic name (`'chatter'`)
- **Third argument**: Queue size (10 messages)

#### 4. Creating a Timer
```python
self.timer = self.create_timer(timer_period, self.timer_callback)
```
- Calls `timer_callback()` every `timer_period` seconds
- Enables periodic publishing without blocking

#### 5. Timer Callback
```python
def timer_callback(self):
    msg = String()
    msg.data = f'Hello World: {self.i}'
    self.publisher_.publish(msg)
```
- Create a message instance
- Set the message data
- Publish the message

#### 6. Main Function
```python
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```
- **`rclpy.init()`**: Initialize the ROS 2 Python client library
- **Create node**: Instantiate your node class
- **`rclpy.spin()`**: Keep the node alive and process callbacks
- **Cleanup**: Destroy the node and shutdown rclpy

---

## Creating a Simple Subscriber Node

### Step 2: Create the Subscriber

Create a file named `minimal_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    def __init__(self):
        # Initialize the node with a name
        super().__init__('minimal_subscriber')

        # Create a subscription to the 'chatter' topic
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        # Prevent unused variable warning
        self.subscription

        self.get_logger().info('Minimal Subscriber has been started')

    def listener_callback(self, msg):
        # This callback is executed when a message is received
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Create the node
    minimal_subscriber = MinimalSubscriber()

    # Keep the node running
    rclpy.spin(minimal_subscriber)

    # Clean shutdown
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Understanding the Subscriber

#### 1. Creating a Subscription
```python
self.subscription = self.create_subscription(
    String,           # Message type
    'chatter',        # Topic name
    self.listener_callback,  # Callback function
    10                # Queue size
)
```

#### 2. Listener Callback
```python
def listener_callback(self, msg):
    self.get_logger().info(f'I heard: "{msg.data}"')
```
- Called automatically when a message arrives
- Receives the message as a parameter
- Access message data via `msg.data`

---

## Running Your Nodes

### Method 1: Direct Python Execution

**Terminal 1 - Run Publisher:**
```bash
source /opt/ros/humble/setup.bash
python3 minimal_publisher.py
```

**Terminal 2 - Run Subscriber:**
```bash
source /opt/ros/humble/setup.bash
python3 minimal_subscriber.py
```

**Expected Output (Publisher):**
```
[INFO] [minimal_publisher]: Minimal Publisher has been started
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
```

**Expected Output (Subscriber):**
```
[INFO] [minimal_subscriber]: Minimal Subscriber has been started
[INFO] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [minimal_subscriber]: I heard: "Hello World: 1"
[INFO] [minimal_subscriber]: I heard: "Hello World: 2"
```

### Method 2: Verify with ROS 2 CLI Tools

While the nodes are running, open a third terminal:

```bash
# List active nodes
ros2 node list

# List active topics
ros2 topic list

# Echo messages
ros2 topic echo /chatter

# Check publish rate
ros2 topic hz /chatter

# Visualize the graph
rqt_graph
```

---

## Node Lifecycle

Understanding the lifecycle of a ROS 2 node:

```
1. rclpy.init()          → Initialize ROS 2
2. Node.__init__()       → Create node
3. create_publisher()    → Set up communication
4. create_timer()        → Set up periodic tasks
5. rclpy.spin()          → Enter event loop (process callbacks)
6. destroy_node()        → Clean up node resources
7. rclpy.shutdown()      → Shutdown ROS 2
```

### The Spin Loop

```python
rclpy.spin(node)
```

This function:
- Blocks and waits for events (messages, timers, service calls)
- Executes callbacks when events occur
- Continues until interrupted (Ctrl+C) or `rclpy.shutdown()` is called

---

## Best Practices

### 1. Always Use Context Managers (Optional)

For more robust cleanup:

```python
def main(args=None):
    rclpy.init(args=args)

    try:
        node = MinimalPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 2. Use Descriptive Node Names

```python
# Good
super().__init__('robot_controller')
super().__init__('camera_processor')

# Avoid
super().__init__('node1')
super().__init__('my_node')
```

### 3. Log Important Events

```python
self.get_logger().info('Node started successfully')
self.get_logger().warn('Low battery detected')
self.get_logger().error('Failed to connect to sensor')
```

### 4. Initialize All Attributes in __init__

```python
def __init__(self):
    super().__init__('my_node')

    # All initialization here
    self.counter = 0
    self.last_message_time = self.get_clock().now()
    self.publisher_ = self.create_publisher(String, 'topic', 10)
```

---

## Common Patterns

### Pattern 1: Publish on Demand (No Timer)

```python
class OnDemandPublisher(Node):
    def __init__(self):
        super().__init__('on_demand_publisher')
        self.publisher_ = self.create_publisher(String, 'events', 10)

    def publish_event(self, event_data):
        msg = String()
        msg.data = event_data
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {event_data}')
```

### Pattern 2: Subscriber That Republishes

```python
class Relay(Node):
    def __init__(self):
        super().__init__('relay')

        self.subscription = self.create_subscription(
            String, 'input_topic', self.callback, 10
        )
        self.publisher_ = self.create_publisher(
            String, 'output_topic', 10
        )

    def callback(self, msg):
        # Process or transform the message
        processed_msg = String()
        processed_msg.data = msg.data.upper()
        self.publisher_.publish(processed_msg)
```

### Pattern 3: Multi-Subscriber Node

```python
class MultiSubscriber(Node):
    def __init__(self):
        super().__init__('multi_subscriber')

        self.sub1 = self.create_subscription(
            String, 'topic1', self.callback1, 10
        )
        self.sub2 = self.create_subscription(
            String, 'topic2', self.callback2, 10
        )

    def callback1(self, msg):
        self.get_logger().info(f'Topic1: {msg.data}')

    def callback2(self, msg):
        self.get_logger().info(f'Topic2: {msg.data}')
```

---

## Debugging Tips

### Problem: Node Not Receiving Messages

**Check:**
1. Are both nodes running?
   ```bash
   ros2 node list
   ```

2. Is the topic name correct?
   ```bash
   ros2 topic list
   ```

3. Are messages being published?
   ```bash
   ros2 topic echo /chatter
   ```

### Problem: ImportError for Message Types

```python
# Wrong
from std_msgs import String

# Correct
from std_msgs.msg import String
```

### Problem: Node Exits Immediately

Make sure you call `rclpy.spin()`:
```python
# Without spin, the node will exit immediately
rclpy.spin(node)  # Don't forget this!
```

---

## Hands-On Exercise

### Exercise 1: Temperature Monitor

Create two nodes:

1. **Temperature Publisher**: Publishes random temperature readings every second
2. **Temperature Monitor**: Subscribes and logs a warning if temperature > 30°C

**Hints:**
```python
import random

# In publisher
temp = random.uniform(15.0, 35.0)
msg.data = f'{temp:.2f}'

# In subscriber
if float(msg.data) > 30.0:
    self.get_logger().warn(f'High temperature: {msg.data}°C')
```

### Exercise 2: Counter Node

Create a node that:
- Subscribes to `/increment` topic (String messages)
- Maintains an internal counter
- Increments counter each time a message is received
- Publishes current count to `/count` topic

---

## Summary

You've learned to:
- Create publisher and subscriber nodes in Python
- Use `rclpy` for ROS 2 communication
- Understand the node lifecycle
- Run and test nodes with CLI tools

**Key Takeaways:**
- Nodes inherit from `rclpy.node.Node`
- Publishers send messages to topics
- Subscribers receive messages via callbacks
- `rclpy.spin()` keeps the node alive
- Always initialize and shutdown properly

---

## Next Steps

- [Message Types](./message-types.md) - Learn about different ROS 2 message types
- [Timers and Callbacks](./timers-callbacks.md) - Advanced callback patterns
- [Parameters](./parameters.md) - Make your nodes configurable

---

## Additional Resources

- [rclpy API Documentation](https://docs.ros2.org/latest/api/rclpy/)
- [ROS 2 Python Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Complete Publisher Example](/code/ros2-packages/minimal_publisher.py)
- [Complete Subscriber Example](/code/ros2-packages/minimal_subscriber.py)
