# Timers and Callbacks in ROS 2

## Introduction

Callbacks are the heart of event-driven programming in ROS 2. Understanding how to use timers and callbacks effectively enables you to build responsive, efficient robotic systems. This lesson covers timer creation, callback execution, and advanced patterns for managing multiple callbacks.

## Learning Objectives

By the end of this lesson, you will be able to:
- Create and manage timers in ROS 2 nodes
- Understand callback execution and the event loop
- Handle multiple callbacks efficiently
- Implement callback groups for parallel execution
- Debug callback timing issues
- Use best practices for callback design

## Prerequisites

- Completed "First ROS 2 Node" lesson
- Understanding of ROS 2 publishers and subscribers
- Basic Python knowledge including functions and classes

---

## Understanding the Event Loop

ROS 2 uses an event-driven architecture:

```
┌─────────────────────────────────────┐
│         rclpy.spin(node)            │
│         ────────────────            │
│                                     │
│  ┌───────────────────────────────┐ │
│  │   Wait for Events             │ │
│  │   - Timer expires             │ │
│  │   - Message arrives           │ │
│  │   - Service called            │ │
│  └───────────────────────────────┘ │
│              │                      │
│              ↓                      │
│  ┌───────────────────────────────┐ │
│  │   Execute Callback            │ │
│  │   - Process event             │ │
│  │   - Return to waiting         │ │
│  └───────────────────────────────┘ │
│              │                      │
│              ↓                      │
│         (repeat)                    │
└─────────────────────────────────────┘
```

**Key Points:**
- `rclpy.spin()` runs an infinite loop
- Blocks until an event occurs
- Executes the associated callback
- Returns to waiting for next event

---

## Creating Timers

### Basic Timer

```python
import rclpy
from rclpy.node import Node


class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Create timer that fires every 1.0 seconds
        self.timer = self.create_timer(
            timer_period_sec=1.0,
            callback=self.timer_callback
        )
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer fired: {self.counter}')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = TimerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Timer with Different Periods

```python
class MultiTimerNode(Node):
    def __init__(self):
        super().__init__('multi_timer_node')

        # Fast timer (10 Hz)
        self.fast_timer = self.create_timer(0.1, self.fast_callback)

        # Medium timer (1 Hz)
        self.medium_timer = self.create_timer(1.0, self.medium_callback)

        # Slow timer (0.2 Hz)
        self.slow_timer = self.create_timer(5.0, self.slow_callback)

    def fast_callback(self):
        self.get_logger().debug('Fast: 10 Hz')

    def medium_callback(self):
        self.get_logger().info('Medium: 1 Hz')

    def slow_callback(self):
        self.get_logger().warn('Slow: 0.2 Hz')
```

---

## Subscription Callbacks

Callbacks are automatically called when messages arrive:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,  # Callback function
            10
        )

    def listener_callback(self, msg):
        # This is called automatically when a message arrives
        self.get_logger().info(f'Received: {msg.data}')
```

**Callback Signature:**
```python
def listener_callback(self, msg: MessageType):
    # msg is the received message
    # Use msg.field_name to access data
    pass
```

---

## Combining Timers and Subscriptions

### Pattern 1: Process on Timer, Publish Latest Data

```python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Subscribe to sensor
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Publish processed result on timer
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.latest_scan = None

    def scan_callback(self, msg):
        # Store latest scan data
        self.latest_scan = msg

    def control_loop(self):
        # Process latest scan and publish command
        if self.latest_scan is not None:
            cmd = self.compute_velocity(self.latest_scan)
            self.publisher.publish(cmd)

    def compute_velocity(self, scan):
        # Example: Simple obstacle avoidance
        cmd = Twist()

        # Get minimum distance
        if scan.ranges:
            min_dist = min(scan.ranges)

            if min_dist < 1.0:
                # Too close, stop
                cmd.linear.x = 0.0
            else:
                # Move forward
                cmd.linear.x = 0.5

        return cmd
```

### Pattern 2: State Machine with Timer

```python
from enum import Enum


class RobotState(Enum):
    IDLE = 0
    MOVING = 1
    TURNING = 2
    STOPPED = 3


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.state = RobotState.IDLE
        self.timer = self.create_timer(0.1, self.state_machine_callback)
        self.state_entry_time = self.get_clock().now()

    def state_machine_callback(self):
        current_time = self.get_clock().now()

        if self.state == RobotState.IDLE:
            # Stay idle for 2 seconds, then move
            if (current_time - self.state_entry_time).nanoseconds > 2e9:
                self.transition_to(RobotState.MOVING)

        elif self.state == RobotState.MOVING:
            # Move for 5 seconds, then turn
            self.get_logger().info('Moving forward...')
            if (current_time - self.state_entry_time).nanoseconds > 5e9:
                self.transition_to(RobotState.TURNING)

        elif self.state == RobotState.TURNING:
            # Turn for 2 seconds, then stop
            self.get_logger().info('Turning...')
            if (current_time - self.state_entry_time).nanoseconds > 2e9:
                self.transition_to(RobotState.STOPPED)

        elif self.state == RobotState.STOPPED:
            self.get_logger().info('Stopped')

    def transition_to(self, new_state):
        self.get_logger().info(f'State: {self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_entry_time = self.get_clock().now()
```

---

## Lambda Functions as Callbacks

You can use lambda functions for simple callbacks:

```python
class LambdaNode(Node):
    def __init__(self):
        super().__init__('lambda_node')

        # Using lambda for simple timer
        self.timer = self.create_timer(
            1.0,
            lambda: self.get_logger().info('Lambda timer fired')
        )

        # Using lambda with counter (requires careful state management)
        self.counter = 0
        self.timer2 = self.create_timer(
            2.0,
            lambda: self.increment_and_log()
        )

    def increment_and_log(self):
        self.counter += 1
        self.get_logger().info(f'Count: {self.counter}')
```

**Note:** For complex logic, use named methods instead of lambdas.

---

## Callback Groups and Parallel Execution

By default, all callbacks execute sequentially. Use callback groups for parallel execution:

### Sequential Execution (Default)

```python
class SequentialNode(Node):
    def __init__(self):
        super().__init__('sequential_node')

        # These callbacks will NEVER run simultaneously
        self.timer1 = self.create_timer(0.1, self.callback1)
        self.timer2 = self.create_timer(0.1, self.callback2)

    def callback1(self):
        # If this takes 1 second, callback2 must wait
        self.get_logger().info('Callback 1 start')
        time.sleep(1.0)
        self.get_logger().info('Callback 1 end')

    def callback2(self):
        self.get_logger().info('Callback 2')
```

### Parallel Execution with Callback Groups

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time


class ParallelNode(Node):
    def __init__(self):
        super().__init__('parallel_node')

        # Create separate callback groups
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()

        # Assign timers to different groups
        self.timer1 = self.create_timer(
            0.1,
            self.callback1,
            callback_group=self.group1
        )

        self.timer2 = self.create_timer(
            0.1,
            self.callback2,
            callback_group=self.group2
        )

    def callback1(self):
        self.get_logger().info('Callback 1 start')
        time.sleep(1.0)
        self.get_logger().info('Callback 1 end')

    def callback2(self):
        # This CAN run while callback1 is sleeping
        self.get_logger().info('Callback 2')


def main(args=None):
    rclpy.init(args=args)
    node = ParallelNode()

    # Use MultiThreadedExecutor for parallel execution
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
```

### Callback Group Types

| Type | Behavior |
|------|----------|
| `MutuallyExclusiveCallbackGroup` | Only one callback in the group executes at a time |
| `ReentrantCallbackGroup` | Multiple callbacks in the group can run simultaneously |

---

## Managing Callback Timing

### Measuring Execution Time

```python
class TimingNode(Node):
    def __init__(self):
        super().__init__('timing_node')
        self.timer = self.create_timer(0.1, self.timed_callback)

    def timed_callback(self):
        start_time = self.get_clock().now()

        # Do work
        self.do_expensive_computation()

        end_time = self.get_clock().now()
        duration = (end_time - start_time).nanoseconds / 1e6  # Convert to ms

        self.get_logger().info(f'Callback took {duration:.2f} ms')

    def do_expensive_computation(self):
        # Simulate work
        result = sum(range(10000))
```

### Rate Limiting

```python
class RateLimitedNode(Node):
    def __init__(self):
        super().__init__('rate_limited_node')

        # Fast subscription
        self.subscription = self.create_subscription(
            String,
            'fast_topic',
            self.fast_callback,
            100  # Large queue
        )

        # Slow publisher (1 Hz)
        self.publisher = self.create_publisher(String, 'slow_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_latest)

        self.latest_message = None
        self.message_count = 0

    def fast_callback(self, msg):
        # Receives messages at high rate (e.g., 100 Hz)
        self.latest_message = msg
        self.message_count += 1

    def publish_latest(self):
        # Publishes only latest at 1 Hz
        if self.latest_message:
            self.publisher.publish(self.latest_message)
            self.get_logger().info(
                f'Published latest (received {self.message_count} messages)'
            )
            self.message_count = 0
```

---

## Canceling and Resetting Timers

### Canceling a Timer

```python
class CancelableTimerNode(Node):
    def __init__(self):
        super().__init__('cancelable_timer_node')

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Count: {self.counter}')

        # Stop after 5 callbacks
        if self.counter >= 5:
            self.timer.cancel()
            self.get_logger().info('Timer canceled')
```

### Creating One-Shot Timers

```python
class OneShotTimerNode(Node):
    def __init__(self):
        super().__init__('one_shot_timer_node')

        # Start a timer
        self.start_one_shot_timer()

    def start_one_shot_timer(self):
        # Create timer
        self.timer = self.create_timer(5.0, self.one_shot_callback)

    def one_shot_callback(self):
        self.get_logger().info('One-shot timer fired!')

        # Cancel immediately to prevent repeat
        self.timer.cancel()

        # Optionally, start another one-shot
        # self.start_one_shot_timer()
```

---

## Error Handling in Callbacks

### Try-Except Pattern

```python
class SafeCallbackNode(Node):
    def __init__(self):
        super().__init__('safe_callback_node')
        self.timer = self.create_timer(1.0, self.safe_callback)

    def safe_callback(self):
        try:
            # Risky operation
            result = self.risky_computation()
            self.get_logger().info(f'Result: {result}')

        except ZeroDivisionError:
            self.get_logger().error('Division by zero!')

        except Exception as e:
            self.get_logger().error(f'Unexpected error: {str(e)}')

    def risky_computation(self):
        # Might raise exceptions
        return 10 / 0  # Will raise ZeroDivisionError
```

### Defensive Programming

```python
class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

    def scan_callback(self, msg):
        # Check data validity before processing
        if not msg.ranges:
            self.get_logger().warn('Empty scan received')
            return

        if len(msg.ranges) < 10:
            self.get_logger().warn('Insufficient scan data')
            return

        # Safe to process
        min_range = min(msg.ranges)
        self.get_logger().info(f'Min range: {min_range:.2f}m')
```

---

## Best Practices

### 1. Keep Callbacks Short and Fast

```python
# Good: Quick callback
def timer_callback(self):
    msg = String()
    msg.data = f'Count: {self.counter}'
    self.publisher.publish(msg)
    self.counter += 1

# Avoid: Blocking callback
def bad_callback(self):
    time.sleep(5.0)  # Blocks entire event loop!
    # Other callbacks can't run
```

### 2. Use Timers for Periodic Tasks

```python
# Good: Timer for periodic publishing
self.timer = self.create_timer(0.1, self.publish_callback)

# Avoid: Loop in subscription callback
def bad_subscription_callback(self, msg):
    while True:  # Never do this!
        # Process and publish
        time.sleep(0.1)
```

### 3. Store Latest Data, Process on Schedule

```python
# Good pattern
def __init__(self):
    self.subscription = self.create_subscription(...)
    self.timer = self.create_timer(0.1, self.process_callback)
    self.latest_data = None

def subscription_callback(self, msg):
    self.latest_data = msg  # Just store

def process_callback(self):
    if self.latest_data:
        self.process(self.latest_data)  # Process on timer
```

### 4. Log at Appropriate Levels

```python
# DEBUG: Verbose details
self.get_logger().debug('Entering callback')

# INFO: Normal operation
self.get_logger().info('Message received')

# WARN: Unexpected but handled
self.get_logger().warn('Data out of range, clamping')

# ERROR: Error occurred
self.get_logger().error('Failed to process data')

# FATAL: Critical failure
self.get_logger().fatal('Hardware failure detected')
```

---

## Hands-On Exercise

### Exercise 1: Multi-Rate Publishing

Create a node with:
- Fast timer (10 Hz) that increments a counter
- Medium timer (1 Hz) that publishes the counter value
- Slow timer (0.2 Hz) that resets the counter

```python
class MultiRateNode(Node):
    def __init__(self):
        super().__init__('multi_rate_node')

        # TODO: Create publisher for counter
        # TODO: Create three timers with different rates
        # TODO: Initialize counter

    def fast_callback(self):
        # TODO: Increment counter
        pass

    def medium_callback(self):
        # TODO: Publish counter value
        pass

    def slow_callback(self):
        # TODO: Reset counter
        pass
```

### Exercise 2: Sensor Fusion

Create a node that:
- Subscribes to two sensor topics (simulate with String messages)
- Stores the latest reading from each sensor
- On a timer (5 Hz), combines both readings and publishes result

---

## Summary

You've learned to:
- Create and manage timers
- Handle multiple callbacks efficiently
- Use callback groups for parallel execution
- Measure and optimize callback timing
- Implement error handling in callbacks

**Key Takeaways:**
- Keep callbacks fast to avoid blocking
- Use timers for periodic operations
- Store latest data, process on schedule
- Use callback groups for parallelism
- Always handle exceptions

---

## Next Steps

- [Parameters](./parameters.md) - Make nodes configurable
- [URDF Syntax](../ch3-urdf/urdf-syntax.md) - Model robots
- [Debugging](../ch4-packages/debugging.md) - Debug timing issues

---

## Additional Resources

- [rclpy Timer API](https://docs.ros2.org/latest/api/rclpy/api/timers.html)
- [Callback Groups Tutorial](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html)
- [Executors and Callback Groups](https://docs.ros.org/en/humble/Concepts/About-Executors.html)
