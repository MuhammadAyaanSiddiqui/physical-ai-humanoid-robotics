# Timers and Callbacks in ROS 2

**Module**: The Robotic Nervous System
**Chapter**: Python Integration (rclpy)
**Estimated Time**: 2-2.5 hours
**Difficulty**: Beginner to Intermediate

## Prerequisites

- Understanding of ROS 2 nodes, topics, and publishers
- Ability to create basic Python nodes
- Python knowledge (functions, classes, async programming concepts helpful)

## Learning Objectives

By the end of this lesson, you will be able to:

- Understand event-driven programming in ROS 2
- Create and use timers for periodic tasks
- Implement callback functions for subscribers
- Handle multiple callbacks in a single node
- Understand the ROS 2 executor and callback execution
- Implement callback groups for advanced concurrency

## Event-Driven Programming

ROS 2 uses an **event-driven architecture**:
- Nodes react to **events** (timer expiry, message arrival, service call)
- **Callbacks** are functions that run when events occur
- The **executor** manages callback execution

### Traditional vs. Event-Driven

**Traditional (Polling)**:
```python
while True:
    if check_sensor():
        process_data()
    if check_timer():
        send_command()
    time.sleep(0.01)  # Check 100 times per second
```

Problems:
- Wastes CPU checking constantly
- Difficult to coordinate multiple events
- Hard to guarantee timing

**Event-Driven (ROS 2)**:
```python
# Callbacks registered once
self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
self.create_timer(1.0, self.timer_callback)

# Executor calls callbacks when events occur
rclpy.spin(node)  # Wait for events, call callbacks automatically
```

Benefits:
- ✅ CPU efficient (sleep until event)
- ✅ Multiple events handled cleanly
- ✅ Guaranteed callback execution

## Timers

**Timers** trigger callbacks at regular intervals.

### Creating a Timer

```python
class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Create timer that fires every 1.0 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer fired! Count: {self.counter}')
        self.counter += 1
```

**create_timer Parameters**:
- `timer_period_sec`: Seconds between callbacks (float)
- `callback`: Function to call when timer fires

### Timer Periods and Frequencies

```python
# 1 Hz (once per second)
self.create_timer(1.0, self.slow_callback)

# 10 Hz
self.create_timer(0.1, self.medium_callback)

# 100 Hz
self.create_timer(0.01, self.fast_callback)

# 0.5 Hz (every 2 seconds)
self.create_timer(2.0, self.very_slow_callback)
```

**Formula**: `period (seconds) = 1 / frequency (Hz)`

### Multiple Timers

A node can have multiple independent timers:

```python
class MultiTimerNode(Node):
    def __init__(self):
        super().__init__('multi_timer_node')

        # Heartbeat every second
        self.create_timer(1.0, self.heartbeat_callback)

        # Sensor reading every 100ms
        self.create_timer(0.1, self.sensor_callback)

        # Status report every 10 seconds
        self.create_timer(10.0, self.status_callback)

    def heartbeat_callback(self):
        self.get_logger().info('♥ Heartbeat')

    def sensor_callback(self):
        self.get_logger().debug('Reading sensor...')

    def status_callback(self):
        self.get_logger().info('=== Status Report ===')
```

All timers run independently and may interleave.

### Canceling Timers

```python
class CancelableTimerNode(Node):
    def __init__(self):
        super().__init__('cancelable_timer')

        # Create timer
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        self.count += 1
        self.get_logger().info(f'Count: {self.count}')

        if self.count >= 10:
            self.timer.cancel()
            self.get_logger().info('Timer cancelled')
```

### Resetting Timers

```python
class ResettableTimerNode(Node):
    def __init__(self):
        super().__init__('resettable_timer')
        self.timer = self.create_timer(5.0, self.timeout_callback)

    def reset_timer(self):
        """Call this to restart the countdown"""
        self.timer.cancel()
        self.timer = self.create_timer(5.0, self.timeout_callback)
        self.get_logger().info('Timer reset')

    def timeout_callback(self):
        self.get_logger().warn('Timeout occurred!')
```

## Subscriber Callbacks

**Subscriber callbacks** are called when messages arrive on a topic.

### Basic Subscriber

```python
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # Create subscriber
        self.subscription = self.create_subscription(
            String,                    # Message type
            'chatter',                 # Topic name
            self.listener_callback,    # Callback function
            10                         # Queue size
        )

    def listener_callback(self, msg):
        """Called when message received on 'chatter' topic"""
        self.get_logger().info(f'I heard: "{msg.data}"')
```

**create_subscription Parameters**:
- `msg_type`: Message class
- `topic`: Topic name
- `callback`: Function to call with received message
- `qos_profile`: Quality of Service (queue size or QoS object)

### Callback Signature

Callbacks receive the message as an argument:

```python
def callback_name(self, msg):
    # msg is an instance of the message type
    # Access fields: msg.data, msg.position.x, etc.
    pass
```

### Multiple Subscribers

```python
from sensor_msgs.msg import LaserScan, Imu

class MultiSubscriberNode(Node):
    def __init__(self):
        super().__init__('multi_subscriber')

        # Subscribe to laser scan
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Subscribe to IMU
        self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

    def scan_callback(self, msg):
        min_range = min(msg.ranges)
        self.get_logger().info(f'Closest obstacle: {min_range:.2f}m')

    def imu_callback(self, msg):
        az = msg.linear_acceleration.z
        self.get_logger().info(f'Accel Z: {az:.2f} m/s²')
```

## Combining Publishers, Subscribers, and Timers

Real nodes often combine all three:

```python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Publisher: velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber: laser scan data
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Timer: publish commands at 10 Hz
        self.create_timer(0.1, self.timer_callback)

        # State
        self.obstacle_detected = False
        self.min_distance = float('inf')

    def scan_callback(self, msg):
        """Update state based on sensor data"""
        self.min_distance = min(msg.ranges)
        self.obstacle_detected = self.min_distance < 1.0

    def timer_callback(self):
        """Generate and publish commands based on state"""
        cmd = Twist()

        if self.obstacle_detected:
            # Obstacle nearby: stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Rotate in place
        else:
            # No obstacle: move forward
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'Distance: {self.min_distance:.2f}m, '
            f'Speed: {cmd.linear.x:.2f}'
        )
```

**Pattern**:
1. **Subscriber** updates state when new data arrives
2. **Timer** reads state and publishes commands at fixed rate
3. **Publisher** sends commands to actuators

This decouples sensing (event-driven) from control (time-driven).

## Callback Execution Order

### Sequential Execution

By default, callbacks execute **sequentially** (one at a time):

```
Time: 0.0s → Timer1 callback starts
Time: 0.05s → Message arrives (waits for Timer1 to finish)
Time: 0.1s → Timer1 callback completes
Time: 0.1s → Message callback starts
Time: 0.15s → Message callback completes
```

**Rule**: Only one callback runs at a time per executor.

### Callback Queue

If multiple events occur before a callback finishes:

```
Events:
- Timer fires
- Message 1 arrives
- Message 2 arrives
- Timer fires again

Execution:
1. Timer callback
2. Message 1 callback
3. Message 2 callback
4. Timer callback
```

Callbacks execute in the order events occurred (FIFO).

### Blocking Callbacks

**Bad Practice** - Long-running callback:
```python
def slow_callback(self, msg):
    result = expensive_computation()  # Takes 5 seconds
    self.publisher.publish(result)
```

**Problem**: Blocks all other callbacks for 5 seconds!

**Solution** - Offload to separate thread:
```python
import threading

def callback(self, msg):
    # Process in background thread
    thread = threading.Thread(target=self.process_data, args=(msg,))
    thread.start()

def process_data(self, msg):
    result = expensive_computation()  # Runs in parallel
    # Be careful: may need thread-safe publishing
```

Or use callback groups (covered below).

## Callback Groups

**Callback groups** control callback execution concurrency.

### Default: Mutually Exclusive Group

All callbacks in the same group execute one at a time:

```python
class DefaultNode(Node):
    def __init__(self):
        super().__init__('default_node')

        # All use default callback group (mutually exclusive)
        self.create_timer(1.0, self.callback1)
        self.create_timer(1.0, self.callback2)
        # callback1 and callback2 never run simultaneously
```

### Reentrant Callback Group

Callbacks can run in parallel:

```python
from rclpy.callback_groups import ReentrantCallbackGroup

class ParallelNode(Node):
    def __init__(self):
        super().__init__('parallel_node')

        # Create reentrant group
        self.reentrant_group = ReentrantCallbackGroup()

        # Timers in reentrant group can run in parallel
        self.create_timer(
            1.0,
            self.slow_callback,
            callback_group=self.reentrant_group
        )

        self.create_timer(
            1.0,
            self.fast_callback,
            callback_group=self.reentrant_group
        )

    def slow_callback(self):
        time.sleep(2)  # Simulates slow processing
        self.get_logger().info('Slow callback done')

    def fast_callback(self):
        self.get_logger().info('Fast callback')
        # Can run while slow_callback is still processing!
```

**Note**: Requires multi-threaded executor (see below).

### Separate Callback Groups

Mix mutually exclusive and reentrant:

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class MixedNode(Node):
    def __init__(self):
        super().__init__('mixed_node')

        # Critical callbacks (never concurrent)
        self.critical_group = MutuallyExclusiveCallbackGroup()

        # Background tasks (can be concurrent)
        self.background_group = ReentrantCallbackGroup()

        # Control loop (critical)
        self.create_timer(0.1, self.control_callback, callback_group=self.critical_group)

        # Logging (background)
        self.create_timer(1.0, self.log_callback, callback_group=self.background_group)

        # Data processing (background)
        self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10,
            callback_group=self.background_group
        )
```

## Executors

**Executors** manage callback execution.

### Single-Threaded Executor (Default)

```python
rclpy.init()
node = MyNode()
rclpy.spin(node)  # Uses single-threaded executor
```

- One callback at a time
- No concurrency
- Simple and safe

### Multi-Threaded Executor

```python
from rclpy.executors import MultiThreadedExecutor

rclpy.init()
node = MyNode()

executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
```

- Multiple threads
- Callbacks in reentrant groups can run in parallel
- Requires thread-safe code

### When to Use Multi-Threaded

**Use multi-threaded when**:
- ✅ Some callbacks are slow and you need others to run concurrently
- ✅ High-frequency real-time control + slow background processing
- ✅ You understand thread safety

**Stick with single-threaded when**:
- ✅ All callbacks are fast (<10ms)
- ✅ Simplicity is important
- ✅ No need for concurrency

## Practical Examples

### Example 1: Heartbeat with Subscriber

```python
from std_msgs.msg import Bool

class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('heartbeat_node')

        # Publish heartbeat every second
        self.heartbeat_pub = self.create_publisher(Bool, 'heartbeat', 10)
        self.create_timer(1.0, self.heartbeat_callback)

        # Subscribe to enable/disable
        self.create_subscription(Bool, 'enable', self.enable_callback, 10)
        self.enabled = True

    def heartbeat_callback(self):
        if self.enabled:
            msg = Bool()
            msg.data = True
            self.heartbeat_pub.publish(msg)
            self.get_logger().info('♥')

    def enable_callback(self, msg):
        self.enabled = msg.data
        self.get_logger().info(f'Heartbeat {"enabled" if self.enabled else "disabled"}')
```

### Example 2: Sensor Fusion

```python
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscribe to sensors
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publish commands at 20 Hz
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.05, self.control_callback)

        # State
        self.angular_velocity = 0.0
        self.front_distance = float('inf')

    def imu_callback(self, msg):
        self.angular_velocity = msg.angular_velocity.z

    def scan_callback(self, msg):
        # Get distance directly in front (middle of scan)
        mid_index = len(msg.ranges) // 2
        self.front_distance = msg.ranges[mid_index]

    def control_callback(self):
        cmd = Twist()

        # Stop if obstacle ahead
        if self.front_distance < 0.5:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = 0.3

        # Compensate for drift using IMU
        cmd.angular.z = -self.angular_velocity * 0.5

        self.cmd_pub.publish(cmd)
```

### Example 3: Rate-Limited Publisher

```python
class RateLimitedPublisher(Node):
    def __init__(self):
        super().__init__('rate_limited_publisher')

        # Subscribe to high-rate topic
        self.create_subscription(
            String,
            'input',
            self.input_callback,
            10
        )

        # Publish at limited rate (1 Hz)
        self.output_pub = self.create_publisher(String, 'output', 10)
        self.create_timer(1.0, self.timer_callback)

        # Buffer latest message
        self.latest_msg = None

    def input_callback(self, msg):
        # Just store latest (may arrive at 100 Hz)
        self.latest_msg = msg

    def timer_callback(self):
        # Publish latest at 1 Hz
        if self.latest_msg:
            self.output_pub.publish(self.latest_msg)
```

## Debugging Callbacks

### Log Callback Execution

```python
def callback(self, msg):
    self.get_logger().info('Callback started')
    # ... process ...
    self.get_logger().info('Callback finished')
```

### Measure Callback Duration

```python
import time

def callback(self, msg):
    start_time = time.time()

    # ... process ...

    duration = time.time() - start_time
    self.get_logger().info(f'Callback took {duration*1000:.2f}ms')
```

### Detect Slow Callbacks

```python
def callback(self, msg):
    start = time.time()
    # ... process ...
    duration = time.time() - start

    if duration > 0.1:  # 100ms threshold
        self.get_logger().warn(f'Slow callback! {duration*1000:.0f}ms')
```

## Best Practices

### 1. Keep Callbacks Short

```python
# Good: Fast callback
def callback(self, msg):
    self.latest_data = msg  # Just store
    self.data_updated = True

# Bad: Slow callback
def callback(self, msg):
    result = expensive_ai_inference(msg)  # Blocks other callbacks!
```

### 2. Use Timers for Periodic Tasks

```python
# Good: Timer for regular publishing
self.create_timer(0.1, self.publish_callback)

# Bad: Publish in subscriber callback
def subscriber_callback(self, msg):
    self.publish(...)  # Tied to message rate, not controlled
```

### 3. Decouple Sensing and Acting

```python
# Good: Separate responsibilities
def sensor_callback(self, msg):
    self.sensor_data = msg  # Update state

def control_timer(self):
    cmd = compute_control(self.sensor_data)  # Use state
    self.publish(cmd)
```

### 4. Initialize State in __init__

```python
def __init__(self):
    super().__init__('my_node')
    # Initialize before callbacks can fire
    self.counter = 0
    self.latest_msg = None
    # Then create timers/subscribers
    self.create_timer(1.0, self.callback)
```

## Common Pitfalls

### Pitfall 1: Uninitialized State

```python
# Bad: state not initialized
def callback(self, msg):
    self.counter += 1  # Error if counter not set in __init__!
```

### Pitfall 2: Forgetting to Store Subscription

```python
# Bad: subscription might be garbage collected
def __init__(self):
    self.create_subscription(...)  # No reference stored!

# Good: store reference
def __init__(self):
    self.sub = self.create_subscription(...)
```

### Pitfall 3: Blocking Operations in Callbacks

```python
# Bad
def callback(self, msg):
    time.sleep(5)  # Blocks everything!

# Good: use threading or callback groups
```

## Practice Exercises

### Exercise 1: Flip-Flop Timer

Create a node with one timer that alternates between two states every second, logging the current state.

### Exercise 2: Echo with Delay

Create a subscriber that echoes received messages after a 2-second delay using a one-shot timer.

**Hint**: Cancel and recreate timer on each message.

### Exercise 3: Sensor Aggregator

Subscribe to 3 different topics. Use a timer to publish an aggregated message containing the latest data from all 3 sensors every second.

## Key Takeaways

- **Timers** trigger callbacks at regular intervals
- **Subscriber callbacks** fire when messages arrive
- **Callbacks execute sequentially** by default (one at a time)
- **Callback groups** control concurrency (mutually exclusive or reentrant)
- **Multi-threaded executor** enables parallel callback execution
- Keep callbacks **short** to avoid blocking
- **Decouple** sensing (subscribers) and acting (timers + publishers)

## What's Next?

Now that you understand event-driven programming:

- **Next Lesson**: [Parameters](./parameters.md) - Runtime configuration
- **Related**: [Services and Actions](../ch1-fundamentals/services-actions.md)

## Further Reading

- [rclpy Timers Documentation](https://docs.ros2.org/latest/api/rclpy/api/timers.html)
- [rclpy Callback Groups](https://docs.ros.org/en/humble/Concepts/About-Executors.html)
- [ROS 2 Executors](https://docs.ros.org/en/humble/Concepts/About-Executors.html)

---

**Checkpoint**: You can now create event-driven ROS 2 nodes with timers and callbacks!
