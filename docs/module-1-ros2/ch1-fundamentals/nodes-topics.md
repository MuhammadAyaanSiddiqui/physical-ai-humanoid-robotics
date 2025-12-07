# ROS 2 Nodes and Topics

**Module**: The Robotic Nervous System
**Chapter**: ROS 2 Fundamentals
**Estimated Time**: 1.5-2 hours
**Difficulty**: Beginner

## Prerequisites

- ROS 2 Humble installed on Ubuntu 22.04
- Terminal/command-line familiarity
- Understanding of basic programming concepts (functions, objects)

## Learning Objectives

By the end of this lesson, you will be able to:

- Explain what ROS 2 nodes are and their role in robotic systems
- Understand the publisher-subscriber communication pattern
- Use ROS 2 topics to send and receive messages between nodes
- Inspect running nodes and topics using CLI tools
- Understand when to use topics vs. other communication methods

## What is a Node?

A **node** is a fundamental building block in ROS 2. Think of a node as a single-purpose executable program that performs a specific computation or task.

### Key Characteristics of Nodes

1. **Single Responsibility**: Each node should do one thing well (e.g., read from a camera, process images, control motors)
2. **Independent Process**: Each node runs as a separate operating system process
3. **Communication**: Nodes communicate with each other through topics, services, and actions
4. **Modular**: Nodes can be started, stopped, and replaced independently

### Real-World Analogy

Think of a robot as a company:
- **CEO Node**: High-level decision-making (navigation planner)
- **Sensor Nodes**: Employees gathering data (camera, LiDAR, IMU)
- **Actuator Nodes**: Employees taking action (motor controllers)
- **Topics**: Company communication channels (email, meetings)

Each employee (node) has a specific job, and they communicate via defined channels (topics) to accomplish the company's goal (robot mission).

## Example: Mobile Robot Node Architecture

```
┌──────────────────┐     /camera/image     ┌──────────────────┐
│  Camera Node     │─────────────────────► │ Object Detection │
│  (Sensor)        │                        │ Node (Processing)│
└──────────────────┘                        └──────────────────┘
                                                     │
                                                     │ /objects/detected
                                                     ▼
┌──────────────────┐     /cmd_vel          ┌──────────────────┐
│  Motor Control   │◄───────────────────── │  Navigation      │
│  Node (Actuator) │                        │  Node (Planning) │
└──────────────────┘                        └──────────────────┘
         │                                           ▲
         │ /odom                                     │
         └───────────────────────────────────────────┘
```

Each box is a node, and arrows represent topics carrying messages.

## What is a Topic?

A **topic** is a named bus over which nodes exchange messages. Topics implement a **publish-subscribe** pattern:

- **Publishers**: Nodes that send messages to a topic
- **Subscribers**: Nodes that receive messages from a topic

### Key Properties of Topics

1. **Anonymous Communication**: Publishers don't know who (if anyone) is subscribed
2. **Many-to-Many**: Multiple publishers and subscribers can connect to the same topic
3. **Asynchronous**: Publishers send messages without waiting for subscribers
4. **Typed**: Each topic has a specific message type (e.g., `String`, `Image`, `LaserScan`)

### Topic Naming Conventions

Topics use hierarchical namespaces separated by `/`:

```
/camera/front/image_raw
/camera/front/camera_info
/lidar/scan
/robot/odom
/cmd_vel
```

**Best Practices**:
- Use descriptive names (e.g., `/camera/image` not `/topic1`)
- Group related topics (e.g., `/camera/image`, `/camera/info`)
- Use lowercase with underscores (e.g., `/image_raw` not `/ImageRaw`)

## Publisher-Subscriber Pattern

### How It Works

```
Publisher Node                Topic                 Subscriber Node(s)
     │                    /temperature                    │
     │                         │                          │
     │  publish(temp=25.5)     │                          │
     │────────────────────────►│                          │
     │                         │  receive(temp=25.5)      │
     │                         │─────────────────────────►│
     │                         │                          │
     │  publish(temp=26.0)     │                          │
     │────────────────────────►│  receive(temp=26.0)      │
     │                         │─────────────────────────►│
     │                         │                          │
```

### Real-World Example: Robot Localization

**Scenario**: A robot needs to know its position in the environment.

1. **Wheel Encoder Node** (Publisher):
   - Publishes odometry data to `/odom` topic at 50 Hz
   - Message type: `nav_msgs/Odometry`

2. **LiDAR Node** (Publisher):
   - Publishes laser scans to `/scan` topic at 10 Hz
   - Message type: `sensor_msgs/LaserScan`

3. **Localization Node** (Subscriber + Publisher):
   - Subscribes to `/odom` and `/scan`
   - Fuses data using particle filter algorithm
   - Publishes estimated pose to `/robot_pose` topic

4. **Navigation Node** (Subscriber):
   - Subscribes to `/robot_pose`
   - Uses pose to plan path to goal

## Hands-On: Inspecting Nodes and Topics

Let's explore nodes and topics using the ROS 2 CLI tools.

### Step 1: Run Demo Nodes

Open Terminal 1 and start a talker node:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Output**:
```
[INFO] [talker]: Publishing: 'Hello World: 0'
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
...
```

Open Terminal 2 and start a listener node:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

**Output**:
```
[INFO] [listener]: I heard: [Hello World: 0]
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
...
```

### Step 2: List Running Nodes

Open Terminal 3 and list all active nodes:

```bash
ros2 node list
```

**Output**:
```
/talker
/listener
```

### Step 3: Inspect Node Information

Get detailed information about the talker node:

```bash
ros2 node info /talker
```

**Output**:
```
/talker
  Subscribers:

  Publishers:
    /chatter: std_msgs/msg/String
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /talker/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /talker/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    ...
  Service Clients:

  Action Servers:

  Action Clients:
```

**What this tells us**:
- The talker node publishes to `/chatter` topic with message type `std_msgs/msg/String`
- It also publishes to system topics like `/rosout` (logging) and `/parameter_events`

### Step 4: List Active Topics

List all topics currently being published:

```bash
ros2 topic list
```

**Output**:
```
/chatter
/parameter_events
/rosout
```

Add `-t` flag to see topic types:

```bash
ros2 topic list -t
```

**Output**:
```
/chatter [std_msgs/msg/String]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
```

### Step 5: Echo Topic Messages

Listen to messages being published on `/chatter`:

```bash
ros2 topic echo /chatter
```

**Output**:
```
data: 'Hello World: 12'
---
data: 'Hello World: 13'
---
data: 'Hello World: 14'
---
```

Press `Ctrl+C` to stop echoing.

### Step 6: Inspect Topic Details

Get detailed information about the `/chatter` topic:

```bash
ros2 topic info /chatter
```

**Output**:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

### Step 7: Check Topic Publishing Rate

Measure the publishing frequency:

```bash
ros2 topic hz /chatter
```

**Output**:
```
average rate: 1.000
	min: 0.999s max: 1.001s std dev: 0.00082s window: 10
```

This shows the talker publishes at approximately 1 Hz (once per second).

## Message Types

Every topic has a specific message type that defines its structure. Common message types:

| Message Type | Description | Example Use |
|--------------|-------------|-------------|
| `std_msgs/msg/String` | Simple text string | Debugging, text data |
| `std_msgs/msg/Int32` | 32-bit integer | Counters, IDs |
| `std_msgs/msg/Float64` | 64-bit float | Sensor readings |
| `geometry_msgs/msg/Twist` | Linear + angular velocity | Robot movement commands |
| `sensor_msgs/msg/Image` | Camera image | Vision processing |
| `sensor_msgs/msg/LaserScan` | LiDAR scan data | Obstacle detection |
| `nav_msgs/msg/Odometry` | Robot position + velocity | Localization |

### Inspecting Message Structure

To see the structure of a message type:

```bash
ros2 interface show std_msgs/msg/String
```

**Output**:
```
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string data
```

For a more complex message:

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

This message has two 3D vectors: `linear` (forward, left, up velocities) and `angular` (roll, pitch, yaw rates).

## Publishing from Command Line

You can manually publish messages to topics for testing:

```bash
# Publish a string message to /chatter
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from CLI'"
```

**Output**:
```
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='Hello from CLI')
publishing #2: std_msgs.msg.String(data='Hello from CLI')
...
```

The listener node will receive these messages:
```
[INFO] [listener]: I heard: [Hello from CLI]
```

### Publish Once

Use `--once` flag to publish a single message:

```bash
ros2 topic pub --once /chatter std_msgs/msg/String "data: 'One-time message'"
```

### Publish at Specific Rate

Use `--rate` flag to set publishing frequency (Hz):

```bash
ros2 topic pub --rate 10 /chatter std_msgs/msg/String "data: 'Fast message at 10 Hz'"
```

## Topic Communication Patterns

### One-to-One

Single publisher, single subscriber (simplest case):

```
[Camera Node] ──/image_raw──► [Image Viewer Node]
```

### One-to-Many (Broadcasting)

Single publisher, multiple subscribers:

```
                              ┌──► [Logger Node]
[Sensor Node] ──/temperature──┤
                              ├──► [Control Node]
                              └──► [Display Node]
```

All subscribers receive the same messages.

### Many-to-One (Fusion)

Multiple publishers, single subscriber:

```
[GPS Node]────┐
              ├──/sensor_fusion──► [Localization Node]
[IMU Node]────┤
              │
[Wheel Odom]──┘
```

Subscriber receives messages from all publishers (may need to handle different rates/timestamps).

### Many-to-Many (Mesh)

Multiple publishers and subscribers (complex systems):

```
[Node A]──┐     ┌──► [Node C]
          ├─/data┤
[Node B]──┘     └──► [Node D]
```

## When to Use Topics

**Use topics when:**
- ✅ Broadcasting sensor data (camera, LiDAR)
- ✅ Continuous data streams (odometry, IMU)
- ✅ One-way communication (publisher doesn't need response)
- ✅ Multiple subscribers need same data

**Don't use topics when:**
- ❌ Need request-response pattern (use **services** instead)
- ❌ Need confirmation of delivery (topics are "fire and forget")
- ❌ Need long-running tasks with feedback (use **actions** instead)
- ❌ Need guaranteed message delivery (topics can drop messages if subscriber is slow)

## Common Patterns in Robotics

### Pattern 1: Sensor Data Pipeline

```
[Camera Driver] ──/image_raw──► [Image Processor] ──/objects──► [Decision Maker]
```

### Pattern 2: Command Chain

```
[User Interface] ──/goal──► [Path Planner] ──/path──► [Controller] ──/cmd_vel──► [Motors]
```

### Pattern 3: State Broadcasting

```
                                  ┌──► [Logger]
                                  │
[Robot State] ──/robot/status────┼──► [Monitor]
                                  │
                                  └──► [Dashboard]
```

## Visualizing Topics with rqt_graph

ROS 2 includes a graphical tool to visualize nodes and topics:

```bash
ros2 run rqt_graph rqt_graph
```

With the talker and listener running, you'll see:

```
┌────────┐     /chatter     ┌──────────┐
│ talker │─────────────────►│ listener │
└────────┘                  └──────────┘
```

**Features**:
- Circles/ovals represent nodes
- Arrows represent topics
- Color-coded by node namespace

## Practice Exercises

### Exercise 1: Topic Discovery

1. Run the turtlesim simulator:
   ```bash
   ros2 run turtlesim turtlesim_node
   ```

2. List all topics published by turtlesim
3. Find the topic that controls turtle movement
4. Identify the message type for turtle pose

**Solution**:
```bash
ros2 topic list
# Output includes: /turtle1/cmd_vel, /turtle1/pose, ...

ros2 topic info /turtle1/cmd_vel
# Type: geometry_msgs/msg/Twist

ros2 topic info /turtle1/pose
# Type: turtlesim/msg/Pose
```

### Exercise 2: Control Turtle from CLI

Make the turtle move in a circle by publishing to `/turtle1/cmd_vel`:

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "linear:
    x: 2.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 1.8"
```

**Expected**: Turtle draws a circular path.

### Exercise 3: Monitor Turtle Pose

Echo the turtle's current position:

```bash
ros2 topic echo /turtle1/pose
```

**Output**:
```
x: 5.544444561004639
y: 5.544444561004639
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
```

## Check Your Understanding

1. **What is a node?**
   <details>
   <summary>Answer</summary>
   A node is an independent process in ROS 2 that performs a specific computation or task, such as reading sensor data or controlling actuators.
   </details>

2. **What is the difference between a publisher and a subscriber?**
   <details>
   <summary>Answer</summary>
   A publisher sends messages to a topic, while a subscriber receives messages from a topic. Multiple publishers and subscribers can connect to the same topic.
   </details>

3. **Can a topic have multiple subscribers?**
   <details>
   <summary>Answer</summary>
   Yes! Topics support one-to-many communication. All subscribers receive copies of published messages.
   </details>

4. **How do you find the message type of a topic?**
   <details>
   <summary>Answer</summary>
   Use `ros2 topic info /topic_name` or `ros2 topic list -t`.
   </details>

## Key Takeaways

- **Nodes** are independent processes that perform specific tasks
- **Topics** are named communication channels using publish-subscribe pattern
- **Publishers** send messages; **subscribers** receive messages
- Topics are anonymous, asynchronous, and typed
- Use `ros2 node` and `ros2 topic` CLI tools to inspect running systems
- Topics are ideal for continuous data streams but not for request-response communication

## What's Next?

Now that you understand nodes and topics, you're ready to learn about:

- **Next Lesson**: [Services and Actions](./services-actions.md) - Request-response and goal-based communication
- **Code Example**: [Creating Your First Node in Python](../ch2-python-rclpy/first-node.md)

## Further Reading

- [Official ROS 2 Nodes Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [Official ROS 2 Topics Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [ROS 2 Design: Topic and Service Quality of Service](https://design.ros2.org/articles/qos.html)

---

**Checkpoint**: You now understand how ROS 2 nodes communicate via topics using the publisher-subscriber pattern!
