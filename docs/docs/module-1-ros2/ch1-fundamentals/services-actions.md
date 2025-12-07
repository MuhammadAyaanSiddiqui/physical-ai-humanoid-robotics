---
sidebar_position: 3
---

# Services and Actions

## Overview

While topics provide one-way, streaming communication, **services** and **actions** enable request-response interactions between nodes. This lesson covers when and how to use each pattern for different robotic tasks.

## Communication Patterns Compared

| Pattern | Direction | Use Case | Example |
|---------|-----------|----------|---------|
| **Topic** | One-way, continuous | Sensor streaming | Camera publishing images |
| **Service** | Request-response, synchronous | One-time queries | "Get robot pose" |
| **Action** | Request-response, asynchronous | Long-running tasks | "Navigate to goal" |

## ROS 2 Services

### What is a Service?

A **service** is a synchronous, request-response communication pattern. The client sends a request and **blocks** until the server responds.

```
┌────────────┐                    ┌────────────┐
│   Client   │───── Request ─────►│   Server   │
│            │                    │            │
│  (Blocks)  │◄──── Response ────│ (Computes) │
└────────────┘                    └────────────┘
```

### When to Use Services

Services are ideal for:
- **Configuration queries**: Get/set parameters
- **State queries**: "What's the battery level?"
- **Quick computations**: "Calculate inverse kinematics"
- **Mode switches**: "Enable/disable motor"

### Service Definition Files (.srv)

Services are defined with `.srv` files:

```srv
# Request
int64 a
int64 b
---
# Response
int64 sum
```

The `---` separator divides request and response fields.

### Built-in Service Types

Common service types from `std_srvs`:

```python
std_srvs/Empty           # No request or response data
std_srvs/SetBool         # Enable/disable something
std_srvs/Trigger         # Trigger an action, get success/message
```

From `example_interfaces`:

```python
example_interfaces/AddTwoInts    # Add two integers
example_interfaces/SetBool       # Set boolean value
```

## Creating a Service Server

Here's a simple service server that adds two integers:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('Add Two Ints Server ready')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: {request.a} + {request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    server = AddTwoIntsServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown

1. **Create Service**: `create_service(ServiceType, 'service_name', callback)`
2. **Define Callback**: Function receives `request` and `response` objects
3. **Populate Response**: Set response fields based on request
4. **Return Response**: Return the response object

## Creating a Service Client

Here's a service client that calls the addition service:

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: add_two_ints_client <a> <b>')
        return

    client = AddTwoIntsClient()
    response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    client.get_logger().info(f'Result: {sys.argv[1]} + {sys.argv[2]} = {response.sum}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown

1. **Create Client**: `create_client(ServiceType, 'service_name')`
2. **Wait for Service**: `wait_for_service()` blocks until server is ready
3. **Create Request**: Populate request fields
4. **Call Service**: `call_async()` sends request and returns a future
5. **Wait for Response**: `spin_until_future_complete()` blocks until response arrives

## Service CLI Tools

### List Available Services

```bash
ros2 service list
```

Output:
```
/add_two_ints
/parameter_events
/describe_parameters
/get_parameters
```

### Get Service Type

```bash
ros2 service type /add_two_ints
```

Output:
```
example_interfaces/srv/AddTwoInts
```

### Call Service from CLI

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

Output:
```
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=5, b=3)

response:
example_interfaces.srv.AddTwoInts_Response(sum=8)
```

### Find Services by Type

```bash
ros2 service find example_interfaces/srv/AddTwoInts
```

Output:
```
/add_two_ints
```

## ROS 2 Actions

### What is an Action?

An **action** is an asynchronous, long-running task with **feedback**. Unlike services, actions don't block the client and provide progress updates.

```
┌────────────┐                    ┌────────────┐
│   Client   │───── Goal ────────►│   Server   │
│            │                    │            │
│  (Async)   │◄─ Feedback (5%) ──│  (Working) │
│            │◄─ Feedback (50%)──│            │
│            │◄─ Feedback (95%)──│            │
│            │◄──── Result ──────│  (Done)    │
└────────────┘                    └────────────┘
```

### When to Use Actions

Actions are ideal for:
- **Navigation**: Move robot to goal position
- **Manipulation**: Pick and place object
- **Long computations**: Train ML model, process large dataset
- **Any task with progress**: Task that takes >1 second

### Action Definition Files (.action)

Actions are defined with `.action` files:

```action
# Goal
float64 target_x
float64 target_y
---
# Result
float64 final_x
float64 final_y
bool success
---
# Feedback
float64 current_x
float64 current_y
float64 distance_remaining
```

Three sections separated by `---`:
1. **Goal**: What client wants
2. **Result**: Final outcome
3. **Feedback**: Progress updates

### Built-in Action Types

Common action from `example_interfaces`:

```python
example_interfaces/Fibonacci     # Calculate Fibonacci sequence
```

From `nav2_msgs`:

```python
nav2_msgs/NavigateToPose        # Navigate robot to goal pose
nav2_msgs/FollowWaypoints       # Follow sequence of waypoints
```

## Creating an Action Server

Here's an action server that calculates Fibonacci sequence with feedback:

```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        self.get_logger().info('Fibonacci Action Server ready')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Initialize feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Calculate Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

            time.sleep(0.5)  # Simulate work

        # Send result
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Goal succeeded! Result: {result.sequence}')
        return result

def main(args=None):
    rclpy.init(args=args)
    server = FibonacciActionServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown

1. **Create Action Server**: `ActionServer(node, ActionType, 'action_name', callback)`
2. **Execute Callback**: Long-running function that processes goal
3. **Publish Feedback**: `goal_handle.publish_feedback()` sends progress updates
4. **Check for Cancel**: `goal_handle.is_cancel_requested` handles cancellation
5. **Send Result**: `goal_handle.succeed()` marks completion and returns result

## Creating an Action Client

Here's an action client that sends a goal and monitors feedback:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: order={order}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.sequence}')

def main(args=None):
    rclpy.init(args=args)
    client = FibonacciActionClient()
    client.send_goal(10)
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

### Code Breakdown

1. **Create Action Client**: `ActionClient(node, ActionType, 'action_name')`
2. **Send Goal**: `send_goal_async()` with feedback callback
3. **Goal Response**: Callback handles goal acceptance/rejection
4. **Feedback Callback**: Receives progress updates during execution
5. **Result Callback**: Receives final result when action completes

## Action CLI Tools

### List Available Actions

```bash
ros2 action list
```

Output:
```
/fibonacci
/turtle1/rotate_absolute
```

### Get Action Info

```bash
ros2 action info /fibonacci
```

Output:
```
Action: /fibonacci
Action clients: 1
    /fibonacci_action_client
Action servers: 1
    /fibonacci_action_server
```

### Send Action Goal from CLI

```bash
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}"
```

Output:
```
Waiting for action server...
Sending goal...
Goal accepted
Feedback: {sequence: [0, 1, 1]}
Feedback: {sequence: [0, 1, 1, 2]}
Feedback: {sequence: [0, 1, 1, 2, 3]}
Result: {sequence: [0, 1, 1, 2, 3, 5]}
```

### Send Goal with Feedback Display

```bash
ros2 action send_goal --feedback /fibonacci example_interfaces/action/Fibonacci "{order: 10}"
```

## Choosing the Right Pattern

Use this decision tree to choose the appropriate communication pattern:

```
Is the data continuous/streaming?
├─ YES → Use TOPIC
└─ NO → Is it a request-response?
    ├─ YES → Does it take >1 second?
    │   ├─ YES → Use ACTION
    │   └─ NO → Use SERVICE
    └─ NO → Use TOPIC
```

### Pattern Comparison

| Aspect | Topic | Service | Action |
|--------|-------|---------|--------|
| **Direction** | One-way | Two-way | Two-way |
| **Timing** | Continuous | Synchronous | Asynchronous |
| **Feedback** | No | No | Yes |
| **Cancellable** | N/A | No | Yes |
| **Blocking** | No | Yes | No |

## Hands-On Exercises

### Exercise 1: Battery Service

Create a service that returns simulated battery level.

**Requirements:**
1. Service name: `/robot/battery_level`
2. Service type: `example_interfaces/srv/Trigger`
3. Response: Success=true, message="Battery: XX%"

<details>
<summary>Solution</summary>

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
import random

class BatteryService(Node):
    def __init__(self):
        super().__init__('battery_service')
        self.srv = self.create_service(
            Trigger,
            '/robot/battery_level',
            self.battery_callback
        )
        self.get_logger().info('Battery service ready')

    def battery_callback(self, request, response):
        level = random.randint(0, 100)
        response.success = True
        response.message = f'Battery: {level}%'
        self.get_logger().info(f'Battery query: {level}%')
        return response

def main(args=None):
    rclpy.init(args=args)
    service = BatteryService()
    rclpy.spin(service)
    service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Test with:
```bash
ros2 service call /robot/battery_level example_interfaces/srv/Trigger
```
</details>

### Exercise 2: Countdown Action

Create an action that counts down from a target number to zero with feedback.

**Requirements:**
1. Action name: `/countdown`
2. Goal: Integer target value
3. Feedback: Current count every second
4. Result: Success message

<details>
<summary>Solution</summary>

```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci  # Reuse for simplicity

class CountdownActionServer(Node):
    def __init__(self):
        super().__init__('countdown_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'countdown',
            self.execute_callback
        )
        self.get_logger().info('Countdown Action Server ready')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Starting countdown from {goal_handle.request.order}')

        feedback_msg = Fibonacci.Feedback()

        for i in range(goal_handle.request.order, -1, -1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()

            feedback_msg.sequence = [i]
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Count: {i}')
            time.sleep(1.0)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = [0]
        self.get_logger().info('Countdown complete!')
        return result

def main(args=None):
    rclpy.init(args=args)
    server = CountdownActionServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Test with:
```bash
ros2 action send_goal --feedback /countdown example_interfaces/action/Fibonacci "{order: 10}"
```
</details>

## Best Practices

### Services

1. **Keep services fast**: Services block clients, so compute quickly (&lt;100ms)
2. **Use for queries**: Services are perfect for state queries
3. **Handle errors**: Return success/failure in response
4. **Wait for service**: Always check if service is available before calling

### Actions

1. **Provide frequent feedback**: Update clients every 0.5-2 seconds
2. **Handle cancellation**: Check `is_cancel_requested` regularly
3. **Set goals**: Use for tasks taking >1 second
4. **Report progress**: Include meaningful progress information in feedback

## Common Pitfalls

### Pitfall 1: Blocking Service Callbacks

**Problem**: Service callback takes too long, blocks the executor

**Solution**: Keep callbacks under 100ms, use actions for long tasks

### Pitfall 2: Not Checking Service Availability

**Problem**: Client calls service before server is ready, hangs forever

**Solution**: Use `wait_for_service()` with timeout

### Pitfall 3: Forgetting Feedback

**Problem**: Action runs but never sends feedback, client has no progress info

**Solution**: Call `publish_feedback()` regularly during execution

### Pitfall 4: Not Handling Cancellation

**Problem**: Action ignores cancel requests, wastes resources

**Solution**: Check `is_cancel_requested` in action loop

## Next Steps

Now that you understand services and actions:

1. Learn about [CLI Tools](./cli-tools.md) for debugging and introspection
2. Build your [First Python Node](../ch2-python-rclpy/first-node.md) from scratch
3. Explore [Message Types](../ch2-python-rclpy/message-types.md) for custom messages

## Resources

- [ROS 2 Services](https://docs.ros.org/en/humble/Concepts/Basic/About-Services.html)
- [ROS 2 Actions](https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html)
- [Understanding Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [Understanding Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

---

**Prerequisites**: Understanding of nodes and topics
**Estimated Time**: 60-75 minutes
**Learning Outcomes**:
- Understand service request-response pattern
- Create service servers and clients
- Understand action goal-feedback-result pattern
- Create action servers and clients
- Choose appropriate communication pattern for different tasks
