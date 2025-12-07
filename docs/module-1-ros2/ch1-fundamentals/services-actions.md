# ROS 2 Services and Actions

**Module**: The Robotic Nervous System
**Chapter**: ROS 2 Fundamentals
**Estimated Time**: 2-2.5 hours
**Difficulty**: Beginner to Intermediate

## Prerequisites

- Understanding of ROS 2 nodes and topics
- ROS 2 Humble installed on Ubuntu 22.04
- Familiarity with publisher-subscriber pattern

## Learning Objectives

By the end of this lesson, you will be able to:

- Explain the difference between topics, services, and actions
- Implement request-response communication using services
- Use actions for long-running, preemptible tasks with feedback
- Choose the appropriate communication method for different scenarios
- Inspect and call services and actions from the command line

## Communication Patterns Comparison

ROS 2 provides three primary communication mechanisms:

| Feature | Topics | Services | Actions |
|---------|--------|----------|---------|
| **Pattern** | Publish-Subscribe | Request-Response | Goal-Based with Feedback |
| **Direction** | One-way | Two-way (synchronous) | Two-way (asynchronous) |
| **Timing** | Continuous stream | On-demand | Long-running |
| **Feedback** | None | Single response | Periodic feedback + final result |
| **Cancellation** | N/A | No | Yes (preemptible) |
| **Use Case** | Sensor data, commands | Computations, queries | Navigation, grasping, long tasks |

## What is a Service?

A **service** implements a request-response pattern where:
- A **client** sends a request with parameters
- A **server** processes the request and returns a response
- Communication is **synchronous** (client waits for response)

### Real-World Analogy

Think of a service like calling a function:

```python
# Function call (synchronous)
result = calculate_sum(5, 10)  # Wait for result
print(result)  # 15
```

Similarly, a ROS 2 service:

```python
# Service call (synchronous)
response = service_client.call(request)  # Wait for response
print(response.result)
```

### When to Use Services

**Use services for:**
- ✅ One-time computations (e.g., inverse kinematics calculation)
- ✅ Database queries (e.g., "get robot configuration")
- ✅ State changes (e.g., "enable motors", "save map")
- ✅ Atomic operations (e.g., "capture single image")
- ✅ Requests that complete quickly (<1 second typical)

**Don't use services for:**
- ❌ Continuous data streams (use topics)
- ❌ Long-running tasks (use actions)
- ❌ Broadcasting to multiple receivers (use topics)
- ❌ Fire-and-forget commands (use topics)

## Service Architecture

```
Client Node                  Service Server Node
     │                              │
     │  Request: add(5, 10)         │
     │─────────────────────────────►│
     │                              │
     │         (Processing...)      │
     │                              │
     │  Response: sum=15            │
     │◄─────────────────────────────│
     │                              │
     │  (Client resumes)            │
     │                              │
```

**Key Points**:
- Client blocks (waits) until server responds
- Server must be running when client calls
- If server crashes, client receives error

## Hands-On: Working with Services

### Step 1: Run a Service Server

Start the turtlesim simulator (which provides several services):

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

### Step 2: List Available Services

In a new terminal, list all active services:

```bash
ros2 service list
```

**Output**:
```
/clear
/kill
/reset
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
...
```

### Step 3: Inspect Service Type

Check the type of the `/spawn` service:

```bash
ros2 service type /spawn
```

**Output**:
```
turtlesim/srv/Spawn
```

### Step 4: View Service Definition

See the request and response structure:

```bash
ros2 interface show turtlesim/srv/Spawn
```

**Output**:
```
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

**Structure**:
- Lines above `---`: Request fields (x, y, theta, name)
- Lines below `---`: Response fields (name)

### Step 5: Call a Service from CLI

Spawn a new turtle named "leonardo" at position (2, 2):

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'leonardo'}"
```

**Output**:
```
waiting for service to become available...
requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.0, name='leonardo')

response:
turtlesim.srv.Spawn_Response(name='leonardo')
```

You'll see a new turtle appear in the simulator!

### Step 6: Call Service Without Arguments

Clear the drawing in turtlesim:

```bash
ros2 service call /clear std_srvs/srv/Empty
```

**Output**:
```
waiting for service to become available...
requester: making request: std_srvs.srv.Empty_Request()

response:
std_srvs.srv.Empty_Response()
```

The drawing disappears.

## Common Service Types

| Service Type | Description | Request Fields | Response Fields |
|--------------|-------------|----------------|-----------------|
| `std_srvs/srv/Empty` | Trigger with no data | None | None |
| `std_srvs/srv/SetBool` | Enable/disable feature | `bool data` | `bool success`, `string message` |
| `std_srvs/srv/Trigger` | Generic trigger | None | `bool success`, `string message` |
| `example_interfaces/srv/AddTwoInts` | Add integers | `int64 a`, `int64 b` | `int64 sum` |
| `nav2_msgs/srv/LoadMap` | Load a map file | `string map_url` | `bool result` |

## Service Example: Add Two Integers

Let's use the built-in example service:

### Start the Service Server

```bash
ros2 run demo_nodes_cpp add_two_ints_server
```

**Output**:
```
[INFO] [add_two_ints_server]: Ready to add two ints.
```

### Call the Service

In a new terminal:

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"
```

**Output**:
```
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=5, b=10)

response:
example_interfaces.srv.AddTwoInts_Response(sum=15)
```

The server logs:
```
[INFO] [add_two_ints_server]: Incoming request: a=5, b=10
[INFO] [add_two_ints_server]: Sending back response: sum=15
```

## What is an Action?

An **action** is designed for long-running, preemptible tasks with periodic feedback:

- **Client** sends a **goal**
- **Server** executes the goal and sends **feedback** updates
- **Server** returns a **result** when complete
- **Client** can **cancel** the goal at any time

### Real-World Analogy

Think of an action like ordering food delivery:

1. **Goal**: "Deliver pizza to my address"
2. **Feedback**: "Order confirmed" → "Preparing" → "Out for delivery" → "5 minutes away"
3. **Result**: "Delivered successfully" or "Delivery failed"
4. **Cancellation**: You can cancel before delivery completes

### When to Use Actions

**Use actions for:**
- ✅ Navigation to a goal (takes seconds/minutes, want feedback on progress)
- ✅ Grasping object (multi-step process with feedback)
- ✅ Training a model (long-running, want loss updates)
- ✅ Any task that can be cancelled or has intermediate states

**Don't use actions for:**
- ❌ Quick computations (use services)
- ❌ Continuous data streams (use topics)
- ❌ Operations that complete in <1 second (use services)

## Action Architecture

```
Action Client                      Action Server
     │                                   │
     │  Goal: Navigate to (10, 5)        │
     │──────────────────────────────────►│
     │                                   │
     │  Feedback: 20% complete           │
     │◄──────────────────────────────────│
     │                                   │
     │  Feedback: 50% complete           │
     │◄──────────────────────────────────│
     │                                   │
     │  (User decides to cancel)         │
     │  Cancel Request                   │
     │──────────────────────────────────►│
     │                                   │
     │  Result: Cancelled                │
     │◄──────────────────────────────────│
     │                                   │
```

## Action Components

An action has three parts:

1. **Goal**: Parameters defining what to accomplish
   ```
   geometry_msgs/PoseStamped target_pose  # Where to navigate
   ```

2. **Feedback**: Periodic updates during execution
   ```
   float32 distance_remaining  # How far left to goal
   duration time_elapsed       # How long running
   ```

3. **Result**: Final outcome when complete
   ```
   bool success                 # Did we reach goal?
   string error_message         # If failed, why?
   ```

## Hands-On: Working with Actions

### Step 1: Run an Action Server

Start the Fibonacci action server:

```bash
ros2 run action_tutorials_cpp fibonacci_action_server
```

**Output**:
```
[INFO] [fibonacci_action_server]: Fibonacci action server started
```

### Step 2: List Available Actions

In a new terminal:

```bash
ros2 action list
```

**Output**:
```
/fibonacci
```

### Step 3: Inspect Action Type

```bash
ros2 action info /fibonacci
```

**Output**:
```
Action: /fibonacci
Action clients: 0
Action servers: 1
    /fibonacci_action_server
```

### Step 4: View Action Definition

```bash
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```

**Output**:
```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

**Structure**:
- First section: Goal (compute Fibonacci sequence of length `order`)
- Second section: Result (final sequence)
- Third section: Feedback (partial sequence as it's computed)

### Step 5: Send an Action Goal

Send a goal to compute Fibonacci sequence of order 10:

```bash
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 10}"
```

**Output**:
```
Waiting for an action server to become available...
Sending goal:
     order: 10

Goal accepted with ID: ...

Result:
    sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]

Goal finished with status: SUCCEEDED
```

### Step 6: Send Goal with Feedback

Add `--feedback` flag to see periodic updates:

```bash
ros2 action send_goal --feedback /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 10}"
```

**Output**:
```
Sending goal:
     order: 10

Goal accepted with ID: ...

Feedback:
    partial_sequence: [0, 1]

Feedback:
    partial_sequence: [0, 1, 1]

Feedback:
    partial_sequence: [0, 1, 1, 2]

Feedback:
    partial_sequence: [0, 1, 1, 2, 3]

... (more feedback) ...

Result:
    sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]

Goal finished with status: SUCCEEDED
```

You see the sequence building up step by step!

## Action States

An action goal can be in several states:

| State | Description |
|-------|-------------|
| `UNKNOWN` | Initial state, not yet accepted |
| `ACCEPTED` | Server accepted the goal and is executing |
| `EXECUTING` | Goal is actively being processed |
| `CANCELING` | Cancel request received, cleaning up |
| `SUCCEEDED` | Goal completed successfully |
| `CANCELED` | Goal was canceled before completion |
| `ABORTED` | Goal failed during execution |

## Practical Robotics Examples

### Example 1: Navigation Service vs. Action

**Bad Design** (using a service):
```python
# Service call blocks until navigation completes (could be minutes!)
response = navigate_service.call(goal_pose)  # Client waits...
# Cannot monitor progress or cancel
```

**Good Design** (using an action):
```python
# Send goal asynchronously
goal_handle = navigate_action.send_goal(goal_pose)

# Get feedback while navigating
while not goal_handle.is_finished():
    feedback = goal_handle.get_feedback()
    print(f"Distance remaining: {feedback.distance_remaining}m")

    # User can cancel
    if user_pressed_stop():
        goal_handle.cancel()
        break

# Get result
result = goal_handle.get_result()
if result.success:
    print("Reached destination!")
```

### Example 2: Motor Enable Service

**Good Design** (using a service):
```python
# Quick state change operation
request = SetBool.Request()
request.data = True  # Enable motors
response = enable_motors_service.call(request)

if response.success:
    print("Motors enabled")
else:
    print(f"Failed: {response.message}")
```

This is a perfect service use case: quick, atomic, synchronous.

### Example 3: Image Capture Service

**Good Design** (using a service):
```python
# Capture a single image on demand
response = capture_image_service.call(CaptureImage.Request())
image = response.image
cv2.imwrite("snapshot.jpg", image)
```

Better than subscribing to continuous image stream if you only need one frame.

## Combining Communication Methods

Real robots use all three methods together:

```
┌─────────────────────┐
│  Navigation Node    │
│  (Action Server)    │
└──────────┬──────────┘
           │ subscribes /scan
           │ subscribes /odom
           │ publishes /cmd_vel
           │
           ▼
┌─────────────────────┐
│  User Interface     │
│  (Action Client)    │
│  (Service Client)   │
└─────────────────────┘
           │
           │ action: navigate_to_pose
           │ service: save_map
```

- **Topics**: Continuous sensor data (`/scan`, `/odom`) and motor commands (`/cmd_vel`)
- **Actions**: Long-running navigation goals with feedback
- **Services**: Quick commands like saving a map

## Practice Exercises

### Exercise 1: Turtle Teleportation

1. Start turtlesim
2. Use the `/turtle1/teleport_absolute` service to move turtle to (3, 3)
3. Verify the new position by echoing `/turtle1/pose` topic

**Solution**:
```bash
# Terminal 1
ros2 run turtlesim turtlesim_node

# Terminal 2
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 3.0, y: 3.0, theta: 0.0}"

# Terminal 3
ros2 topic echo /turtle1/pose --once
```

### Exercise 2: Action Cancellation

1. Run Fibonacci action server
2. Send goal with large order (e.g., 50) with feedback
3. Press `Ctrl+C` to cancel during execution
4. Observe that goal is cancelled

**Solution**:
```bash
# Terminal 1
ros2 run action_tutorials_cpp fibonacci_action_server

# Terminal 2
ros2 action send_goal --feedback /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 50}"
# Press Ctrl+C while it's running
```

### Exercise 3: Compare Service vs. Topic

Create a scenario where:
- Topic is used for continuous sensor readings (temperature)
- Service is used to request statistical analysis of recent readings

Think: Why not use a service for temperature readings? Why not use a topic for statistics?

<details>
<summary>Answer</summary>

- **Topic for temperature**: Continuous stream, many subscribers might need it, fire-and-forget
- **Service for statistics**: On-demand computation, returns single result, not needed continuously
</details>

## Decision Tree: Which Communication Method?

```
START: Need to communicate between nodes
│
├─ Is it continuous data? (sensor streams, status updates)
│   └─ YES → Use TOPIC
│
├─ Is it a quick request-response? (&lt; 1 second)
│   └─ YES → Use SERVICE
│
├─ Is it long-running with feedback? (navigation, manipulation)
│   └─ YES → Use ACTION
│
└─ Is it parameter configuration?
    └─ YES → Use PARAMETERS (covered in later lesson)
```

## Common Patterns

### Pattern 1: Sensor Data + Command Service

```
[Sensor Node] ──/sensor_data──► [Processing Node]
                                       │
                                       │ /process_command (service)
                                       │◄───────────────
[Controller]───────────────────────────┘
```

### Pattern 2: Action with Topic Feedback

```
[Action Server] ──/progress──► [Monitor]
                                    │
                                    │ /cancel_task (action cancel)
[User Interface]────────────────────┘
```

### Pattern 3: Service Chain

```
[Client] ──/get_data──► [Database] ──/analyze──► [Analyzer]
                                                       │
                                                       │ /result
                                                       └───────► [Client]
```

## Check Your Understanding

1. **When should you use a service instead of a topic?**
   <details>
   <summary>Answer</summary>
   Use a service when you need request-response communication, want to wait for a result, and the operation completes quickly. Use a topic for continuous data streams or fire-and-forget commands.
   </details>

2. **What are the three parts of an action?**
   <details>
   <summary>Answer</summary>
   Goal (what to accomplish), Feedback (periodic updates), Result (final outcome).
   </details>

3. **Can you cancel a service call?**
   <details>
   <summary>Answer</summary>
   No. Service calls are synchronous and cannot be cancelled once sent. Use actions for cancellable tasks.
   </details>

4. **Why use actions instead of services for navigation?**
   <details>
   <summary>Answer</summary>
   Navigation takes seconds to minutes, and you want periodic feedback on progress and the ability to cancel if plans change. Services block until complete with no feedback or cancellation.
   </details>

## Key Takeaways

- **Services**: Synchronous request-response for quick operations
- **Actions**: Asynchronous goal-based communication for long tasks
- Services have request and response; actions have goal, feedback, and result
- Actions can be cancelled; services cannot
- Use CLI tools (`ros2 service`, `ros2 action`) to inspect and test
- Choose the right method based on timing, feedback needs, and cancellation requirements

## What's Next?

Now that you understand all ROS 2 communication methods:

- **Next Lesson**: [ROS 2 CLI Tools](./cli-tools.md) - Master command-line debugging
- **Code Example**: [Creating Your First Node in Python](../ch2-python-rclpy/first-node.md)

## Further Reading

- [Official ROS 2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [Official ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- [ROS 2 Design: Actions](https://design.ros2.org/articles/actions.html)

---

**Checkpoint**: You now understand all three ROS 2 communication patterns and when to use each!
