# ROS 2 Command-Line Tools

**Module**: The Robotic Nervous System
**Chapter**: ROS 2 Fundamentals
**Estimated Time**: 1.5-2 hours
**Difficulty**: Beginner

## Prerequisites

- ROS 2 Humble installed and sourced
- Understanding of nodes, topics, services, and actions
- Terminal familiarity

## Learning Objectives

By the end of this lesson, you will be able to:

- Use `ros2` CLI tools to inspect running systems
- Debug communication issues between nodes
- Monitor system performance and health
- Test nodes without writing code
- Understand the complete `ros2` command suite

## The ros2 CLI

The `ros2` command-line interface (CLI) is your primary tool for:
- **Inspecting** running systems (what nodes, topics, services exist?)
- **Testing** nodes and communication (publish test messages, call services)
- **Debugging** issues (why isn't my node receiving messages?)
- **Monitoring** system health (is ROS 2 configured correctly?)

### General Syntax

```bash
ros2 <verb> <subcommand> [options] [arguments]
```

**Examples**:
```bash
ros2 topic list           # verb=topic, subcommand=list
ros2 node info /my_node   # verb=node, subcommand=info
ros2 run pkg executable   # verb=run
```

### Getting Help

Get help for any command:

```bash
# General help
ros2 --help

# Help for specific verb
ros2 topic --help

# Help for specific subcommand
ros2 topic list --help
```

## Core Commands Overview

| Command | Purpose |
|---------|---------|
| `ros2 node` | Inspect and manage nodes |
| `ros2 topic` | Work with topics |
| `ros2 service` | Work with services |
| `ros2 action` | Work with actions |
| `ros2 param` | Manage parameters |
| `ros2 run` | Run nodes from packages |
| `ros2 launch` | Run launch files |
| `ros2 pkg` | Package management |
| `ros2 interface` | Inspect message/service/action types |
| `ros2 doctor` | Diagnose system issues |
| `ros2 bag` | Record and replay data |
| `ros2 daemon` | Manage ROS 2 daemon |

---

## ros2 node - Node Management

### List Running Nodes

```bash
ros2 node list
```

**Output** (with turtlesim running):
```
/turtlesim
```

### Get Node Information

```bash
ros2 node info /turtlesim
```

**Output**:
```
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    ...
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```

**What this tells you**:
- Topics the node subscribes to (inputs)
- Topics the node publishes to (outputs)
- Services the node provides (server)
- Services the node uses (client)
- Actions the node implements

### Common Use Cases

**Check if a node is running**:
```bash
ros2 node list | grep my_node
```

**Monitor all nodes continuously**:
```bash
watch -n 1 'ros2 node list'
```

---

## ros2 topic - Topic Operations

### List Active Topics

```bash
# List all topics
ros2 topic list

# List with message types
ros2 topic list -t
```

**Output**:
```
/chatter [std_msgs/msg/String]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
```

### Topic Information

```bash
ros2 topic info /chatter
```

**Output**:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

### Echo Topic Messages

```bash
# Print all messages
ros2 topic echo /chatter

# Print once and exit
ros2 topic echo --once /chatter

# Print with timestamps
ros2 topic echo --no-arr --no-str /chatter

# Filter fields (print only data field)
ros2 topic echo --field data /chatter
```

**Example Output**:
```
data: 'Hello World: 0'
---
data: 'Hello World: 1'
---
```

### Measure Publishing Rate

```bash
ros2 topic hz /chatter
```

**Output**:
```
average rate: 1.000
	min: 0.999s max: 1.001s std dev: 0.00082s window: 10
```

Shows average frequency (Hz), min/max intervals, and standard deviation.

### Monitor Bandwidth

```bash
ros2 topic bw /camera/image_raw
```

**Output**:
```
Subscribed to [/camera/image_raw]
average: 12.34 MB/s
	mean: 0.82 MB min: 0.81 MB max: 0.83 MB window: 10
```

Useful for high-bandwidth topics like cameras.

### Publish from Command Line

```bash
# Publish continuously at 1 Hz (default)
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from CLI'"

# Publish once
ros2 topic pub --once /chatter std_msgs/msg/String "data: 'One message'"

# Publish at specific rate (10 Hz)
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

### Find Topics by Type

```bash
ros2 topic find sensor_msgs/msg/Image
```

**Output**:
```
/camera/image_raw
/camera/image_compressed
```

Lists all topics publishing `Image` messages.

### Common Use Cases

**Debug why subscriber isn't receiving**:
```bash
# 1. Check topic exists
ros2 topic list | grep my_topic

# 2. Check publishers
ros2 topic info /my_topic

# 3. Verify messages are being sent
ros2 topic hz /my_topic
ros2 topic echo /my_topic
```

**Test node without writing subscriber**:
```bash
ros2 topic echo /my_topic
```

---

## ros2 service - Service Operations

### List Available Services

```bash
# List all services
ros2 service list

# List with service types
ros2 service list -t
```

**Output**:
```
/clear [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
```

### Service Type

```bash
ros2 service type /spawn
```

**Output**:
```
turtlesim/srv/Spawn
```

### Find Services by Type

```bash
ros2 service find std_srvs/srv/Empty
```

**Output**:
```
/clear
/reset
```

### Call a Service

```bash
# Service with no arguments
ros2 service call /clear std_srvs/srv/Empty

# Service with arguments
ros2 service call /spawn turtlesim/srv/Spawn \
  "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

**Output**:
```
requester: making request: turtlesim.srv.Spawn_Request(...)
response:
turtlesim.srv.Spawn_Response(name='turtle2')
```

### Common Use Cases

**Test service without writing client**:
```bash
ros2 service call /my_service pkg/srv/MyService "{param: value}"
```

**Check if service is available**:
```bash
ros2 service list | grep my_service
```

**Debug service not responding**:
```bash
# Check service exists
ros2 service list

# Try calling with timeout
timeout 5 ros2 service call /my_service ...
```

---

## ros2 action - Action Operations

### List Active Actions

```bash
# List all actions
ros2 action list

# List with action types
ros2 action list -t
```

**Output**:
```
/fibonacci [action_tutorials_interfaces/action/Fibonacci]
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```

### Action Information

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

### Send Action Goal

```bash
# Send goal without feedback
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci \
  "{order: 5}"

# Send goal with feedback
ros2 action send_goal --feedback /fibonacci \
  action_tutorials_interfaces/action/Fibonacci "{order: 10}"
```

**Output (with feedback)**:
```
Waiting for an action server to become available...
Sending goal:
     order: 10

Goal accepted with ID: ...

Feedback:
    partial_sequence: [0, 1]
Feedback:
    partial_sequence: [0, 1, 1]
... (more feedback) ...

Result:
    sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]

Goal finished with status: SUCCEEDED
```

### Common Use Cases

**Test action server**:
```bash
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}}}}"
```

---

## ros2 param - Parameter Management

Parameters are node configuration values that can be changed at runtime.

### List Parameters

```bash
# List all parameters for all nodes
ros2 param list

# List parameters for specific node
ros2 param list /turtlesim
```

**Output**:
```
/turtlesim:
  background_b
  background_g
  background_r
  use_sim_time
```

### Get Parameter Value

```bash
ros2 param get /turtlesim background_r
```

**Output**:
```
Integer value is: 69
```

### Set Parameter Value

```bash
ros2 param set /turtlesim background_r 255
```

**Output**:
```
Set parameter successful
```

The turtlesim background changes color!

### Dump Parameters to File

```bash
ros2 param dump /turtlesim > turtlesim_params.yaml
```

**turtlesim_params.yaml**:
```yaml
/turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 69
    use_sim_time: false
```

### Load Parameters from File

```bash
ros2 param load /turtlesim turtlesim_params.yaml
```

---

## ros2 run - Run Nodes

### Basic Usage

```bash
ros2 run <package> <executable>
```

**Examples**:
```bash
ros2 run turtlesim turtlesim_node
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

### Run with Remapping

Remap topics or node names:

```bash
# Remap topic
ros2 run demo_nodes_cpp talker --ros-args --remap chatter:=my_topic

# Remap node name
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=my_talker

# Set parameter
ros2 run demo_nodes_cpp talker --ros-args -p my_param:=value
```

### Run with Custom Parameters

```bash
ros2 run my_pkg my_node --ros-args --params-file config.yaml
```

---

## ros2 pkg - Package Management

### List Installed Packages

```bash
ros2 pkg list
```

**Output**:
```
action_tutorials_cpp
demo_nodes_cpp
geometry_msgs
... (hundreds of packages)
```

### Find Package Executables

```bash
ros2 pkg executables turtlesim
```

**Output**:
```
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

### Get Package Path

```bash
ros2 pkg prefix turtlesim
```

**Output**:
```
/opt/ros/humble
```

### Create New Package

```bash
ros2 pkg create --build-type ament_cmake my_robot_pkg
ros2 pkg create --build-type ament_python my_python_pkg
```

---

## ros2 interface - Message/Service/Action Inspection

### List All Interfaces

```bash
ros2 interface list
```

**Output**:
```
action_tutorials_interfaces/action/Fibonacci
std_msgs/msg/String
geometry_msgs/msg/Twist
...
```

### Show Interface Definition

```bash
# Message
ros2 interface show std_msgs/msg/String

# Service
ros2 interface show std_srvs/srv/SetBool

# Action
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```

**Output (service example)**:
```
bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```

### Find Packages with Interface

```bash
ros2 interface package geometry_msgs
```

**Output**:
```
geometry_msgs/msg/Point
geometry_msgs/msg/Pose
geometry_msgs/msg/Twist
...
```

---

## ros2 doctor - System Diagnostics

Check for ROS 2 configuration issues:

```bash
ros2 doctor
```

**Output** (healthy system):
```
UserWarning: Publisher without subscribers:
  /rosout [rcl_interfaces/msg/Log]

Check passed!
```

**Output** (unhealthy system):
```
ERROR: Multiple ROS_DOMAIN_IDs detected
  Node 1: ROS_DOMAIN_ID=0
  Node 2: ROS_DOMAIN_ID=5

RECOMMENDATION: Set consistent ROS_DOMAIN_ID
```

### Common Checks

```bash
# Full diagnostic report
ros2 doctor --report

# Include network configuration
ros2 doctor --include-warnings
```

---

## ros2 bag - Record and Replay Data

### Record Topics

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /chatter /turtle1/pose

# Record to specific file
ros2 bag record -o my_bag /chatter
```

**Output**:
```
[INFO] [rosbag2_storage]: Opened database 'rosbag2_2024_01_01-12_00_00'.
[INFO] [rosbag2_recorder]: Recording topics: /chatter, /turtle1/pose
[INFO] [rosbag2_recorder]: Listening for topics...
```

Press `Ctrl+C` to stop recording.

### Bag Information

```bash
ros2 bag info my_bag
```

**Output**:
```
Files:             my_bag_0.db3
Bag size:          228.5 KiB
Storage id:        sqlite3
Duration:          44.272s
Start:             Jan  1 2024 12:00:00.0  (1704110400.0)
End:               Jan  1 2024 12:00:44.2  (1704110444.2)
Messages:          889
Topic information: Topic: /chatter | Type: std_msgs/msg/String | Count: 44 | Serialization Format: cdr
                   Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 845 | Serialization Format: cdr
```

### Replay Recorded Data

```bash
ros2 bag play my_bag
```

**Options**:
```bash
# Replay at 2x speed
ros2 bag play --rate 2.0 my_bag

# Replay in loop
ros2 bag play --loop my_bag

# Start from specific time
ros2 bag play --start-offset 10 my_bag
```

---

## ros2 daemon - Daemon Management

The ROS 2 daemon caches discovery information for faster CLI commands.

### Restart Daemon

```bash
ros2 daemon stop
ros2 daemon start
```

### Check Daemon Status

```bash
ros2 daemon status
```

**Output**:
```
The daemon is running
```

**When to restart daemon**:
- CLI commands are slow or hanging
- Nodes/topics not appearing in lists
- After network configuration changes

---

## Debugging Workflow

### Issue: Node Not Receiving Messages

**Steps**:
1. Check node is running:
   ```bash
   ros2 node list
   ```

2. Check topic exists:
   ```bash
   ros2 topic list | grep /my_topic
   ```

3. Check publishers:
   ```bash
   ros2 topic info /my_topic
   ```

4. Verify messages are being published:
   ```bash
   ros2 topic hz /my_topic
   ```

5. Check message content:
   ```bash
   ros2 topic echo /my_topic
   ```

6. Check node subscriptions:
   ```bash
   ros2 node info /my_node
   ```

### Issue: Service Not Responding

**Steps**:
1. Check service exists:
   ```bash
   ros2 service list
   ```

2. Try calling service:
   ```bash
   ros2 service call /my_service ...
   ```

3. Check service server is running:
   ```bash
   ros2 node list
   ros2 node info /server_node
   ```

### Issue: Different Machines Can't Communicate

**Steps**:
1. Check ROS_DOMAIN_ID matches:
   ```bash
   echo $ROS_DOMAIN_ID
   ```

2. Check network discovery:
   ```bash
   ros2 doctor
   ```

3. Test multicast:
   ```bash
   ros2 multicast receive  # On machine A
   ros2 multicast send     # On machine B
   ```

---

## Quick Reference Cheat Sheet

### Most Used Commands

```bash
# Nodes
ros2 node list
ros2 node info /node_name

# Topics
ros2 topic list
ros2 topic echo /topic_name
ros2 topic hz /topic_name
ros2 topic pub /topic_name msg_type "data"

# Services
ros2 service list
ros2 service call /service_name srv_type "request"

# Actions
ros2 action list
ros2 action send_goal --feedback /action_name action_type "goal"

# Run
ros2 run package executable

# Diagnostics
ros2 doctor
ros2 daemon restart

# Record/Replay
ros2 bag record -a
ros2 bag play bag_name
```

---

## Practice Exercises

### Exercise 1: Complete Inspection

1. Run turtlesim and teleop_key
2. Use CLI tools to answer:
   - How many nodes are running?
   - What topics does turtlesim subscribe to?
   - What is the publishing rate of `/turtle1/pose`?
   - What message type is `/turtle1/cmd_vel`?

### Exercise 2: Manual Control

1. Run turtlesim (no teleop)
2. Use `ros2 topic pub` to make the turtle draw a square
3. Use `ros2 service call` to spawn a second turtle

### Exercise 3: Data Recording

1. Run turtlesim and teleop
2. Record all topics for 30 seconds while moving turtle
3. Kill turtlesim
4. Replay the recording - turtle should repeat movements!

---

## Key Takeaways

- `ros2` CLI provides tools for inspection, testing, and debugging
- Use `ros2 node/topic/service/action` to inspect running systems
- `ros2 topic pub` and `ros2 service call` enable testing without code
- `ros2 bag` records and replays data for debugging and development
- `ros2 doctor` diagnoses configuration issues

## What's Next?

You've mastered ROS 2 fundamentals! Now learn to write code:

- **Next Chapter**: [Python Integration (rclpy)](../ch2-python-rclpy/first-node.md)
- **Related**: [Debugging Best Practices](../ch4-packages/debugging.md)

## Further Reading

- [Official ROS 2 CLI Tools Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
- [ROS 2 Command Line Cheat Sheet](https://github.com/ubuntu-robotics/ros2_cheats_sheet)

---

**Checkpoint**: You can now inspect, test, and debug ROS 2 systems using CLI tools!
