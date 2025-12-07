# ROS 2 CLI Tools

## Introduction

The ROS 2 command-line interface (CLI) provides powerful tools for interacting with your robot system. These tools allow you to inspect, debug, and control ROS 2 nodes, topics, services, and more without writing any code. Mastering the CLI is essential for effective development and troubleshooting.

## Learning Objectives

By the end of this lesson, you will be able to:
- Use `ros2 topic` commands to inspect and interact with topics
- Use `ros2 node` commands to view and manage nodes
- Use `ros2 service` commands to call services
- Use `ros2 param` commands to get and set parameters
- Debug ROS 2 systems using CLI tools

## Prerequisites

- ROS 2 Humble installed and sourced
- Understanding of nodes, topics, and services
- Basic terminal navigation skills

---

## Core ROS 2 CLI Tools

### 1. ros2 topic

The `ros2 topic` command is used to interact with ROS 2 topics.

#### List All Topics

```bash
ros2 topic list
```

**Output Example:**
```
/parameter_events
/rosout
/chatter
/robot/velocity
```

#### List Topics with Types

```bash
ros2 topic list -t
```

**Output Example:**
```
/chatter [std_msgs/msg/String]
/robot/velocity [geometry_msgs/msg/Twist]
```

#### Echo Topic Data

View messages published to a topic in real-time:

```bash
ros2 topic echo /chatter
```

**Output Example:**
```
data: 'Hello World: 0'
---
data: 'Hello World: 1'
---
```

#### Get Topic Information

```bash
ros2 topic info /chatter
```

**Output Example:**
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

#### Publish to a Topic

Manually publish a message to a topic:

```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from CLI'"
```

**Publish Once:**
```bash
ros2 topic pub --once /chatter std_msgs/msg/String "data: 'One-time message'"
```

**Publish at Specific Rate (10 Hz):**
```bash
ros2 topic pub --rate 10 /chatter std_msgs/msg/String "data: 'Hello at 10Hz'"
```

#### Check Topic Frequency

```bash
ros2 topic hz /chatter
```

**Output Example:**
```
average rate: 10.002
  min: 0.099s max: 0.101s std dev: 0.00095s window: 10
```

#### Check Message Bandwidth

```bash
ros2 topic bw /chatter
```

**Output Example:**
```
average: 156B/s
  mean: 52B min: 52B max: 52B window: 3
```

---

### 2. ros2 node

The `ros2 node` command manages and inspects ROS 2 nodes.

#### List All Nodes

```bash
ros2 node list
```

**Output Example:**
```
/talker
/listener
/robot_controller
```

#### Get Node Information

```bash
ros2 node info /talker
```

**Output Example:**
```
/talker
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /chatter: std_msgs/msg/String
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /talker/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /talker/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /talker/get_parameters: rcl_interfaces/srv/GetParameters
    /talker/list_parameters: rcl_interfaces/srv/ListParameters
    /talker/set_parameters: rcl_interfaces/srv/SetParameters
  Service Clients:

  Action Servers:

  Action Clients:
```

---

### 3. ros2 service

The `ros2 service` command interacts with ROS 2 services.

#### List All Services

```bash
ros2 service list
```

**Output Example:**
```
/add_two_ints
/robot/reset_position
/talker/describe_parameters
```

#### List Services with Types

```bash
ros2 service list -t
```

**Output Example:**
```
/add_two_ints [example_interfaces/srv/AddTwoInts]
/robot/reset_position [std_srvs/srv/Empty]
```

#### Call a Service

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

**Output Example:**
```
waiting for service to become available...
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=5, b=3)

response:
example_interfaces.srv.AddTwoInts_Response(sum=8)
```

#### Call Empty Service

```bash
ros2 service call /robot/reset_position std_srvs/srv/Empty
```

#### Get Service Type

```bash
ros2 service type /add_two_ints
```

**Output Example:**
```
example_interfaces/srv/AddTwoInts
```

---

### 4. ros2 param

The `ros2 param` command manages node parameters.

#### List Parameters for a Node

```bash
ros2 param list /talker
```

**Output Example:**
```
use_sim_time
publish_rate
message_prefix
```

#### Get Parameter Value

```bash
ros2 param get /talker publish_rate
```

**Output Example:**
```
Integer value is: 10
```

#### Set Parameter Value

```bash
ros2 param set /talker publish_rate 20
```

**Output Example:**
```
Set parameter successful
```

#### Dump Parameters to YAML File

```bash
ros2 param dump /talker > talker_params.yaml
```

#### Load Parameters from YAML File

```bash
ros2 param load /talker talker_params.yaml
```

---

### 5. ros2 interface

The `ros2 interface` command shows message, service, and action definitions.

#### Show Message Definition

```bash
ros2 interface show std_msgs/msg/String
```

**Output Example:**
```
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_interfaces

string data
```

#### Show Service Definition

```bash
ros2 interface show example_interfaces/srv/AddTwoInts
```

**Output Example:**
```
int64 a
int64 b
---
int64 sum
```

#### List All Interfaces

```bash
ros2 interface list
```

#### List Interfaces by Package

```bash
ros2 interface package std_msgs
```

---

### 6. ros2 bag

The `ros2 bag` command records and plays back ROS 2 data.

#### Record Topics

```bash
ros2 bag record /chatter /robot/velocity
```

**Record All Topics:**
```bash
ros2 bag record -a
```

**Record with Custom Name:**
```bash
ros2 bag record -o my_experiment /chatter
```

#### Play Back Recorded Data

```bash
ros2 bag play my_experiment
```

**Play in Loop:**
```bash
ros2 bag play -l my_experiment
```

**Play at Different Speed (2x):**
```bash
ros2 bag play -r 2.0 my_experiment
```

#### Get Bag Information

```bash
ros2 bag info my_experiment
```

**Output Example:**
```
Files:             my_experiment_0.db3
Bag size:          1.2 MiB
Storage id:        sqlite3
Duration:          15.5s
Start:             Dec 5 2025 10:30:00.123 (1733390400.123)
End:               Dec 5 2025 10:30:15.623 (1733390415.623)
Messages:          156
Topic information: Topic: /chatter | Type: std_msgs/msg/String | Count: 156 | Serialization Format: cdr
```

---

### 7. ros2 run and ros2 launch

#### Run a Node

```bash
ros2 run <package_name> <executable_name>
```

**Example:**
```bash
ros2 run demo_nodes_cpp talker
```

**Run with Remapping:**
```bash
ros2 run demo_nodes_cpp talker --ros-args -r chatter:=/my_topic
```

**Run with Parameter:**
```bash
ros2 run demo_nodes_cpp talker --ros-args -p publish_rate:=5
```

#### Launch a Launch File

```bash
ros2 launch <package_name> <launch_file>
```

**Example:**
```bash
ros2 launch my_robot_package robot_launch.py
```

---

### 8. rqt_graph - Visual Debugging

Visualize the ROS 2 computation graph:

```bash
rqt_graph
```

This opens a GUI showing:
- All active nodes
- Topics connecting them
- Direction of data flow

**Use Cases:**
- Verify node connections
- Debug communication issues
- Understand system architecture

---

### 9. ros2 doctor

Diagnose ROS 2 system issues:

```bash
ros2 doctor
```

**Output Example:**
```
Checking ROS 2 environment...
All checks passed!
```

**Detailed Report:**
```bash
ros2 doctor --report
```

---

## Practical Workflow Examples

### Example 1: Debug Publisher-Subscriber Communication

```bash
# Terminal 1: Start publisher
ros2 run demo_nodes_cpp talker

# Terminal 2: List topics
ros2 topic list

# Terminal 3: Check if messages are flowing
ros2 topic echo /chatter

# Terminal 4: Check publish rate
ros2 topic hz /chatter

# Terminal 5: Visualize graph
rqt_graph
```

### Example 2: Test a Service

```bash
# Terminal 1: Start service server
ros2 run demo_nodes_cpp add_two_ints_server

# Terminal 2: List available services
ros2 service list

# Terminal 3: Call the service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"
```

### Example 3: Record and Replay Experiment

```bash
# Terminal 1: Start your robot system
ros2 launch my_robot_package robot.launch.py

# Terminal 2: Record experiment data
ros2 bag record -o experiment_1 /robot/velocity /robot/odometry /camera/image

# Later: Replay the experiment
ros2 bag play experiment_1
```

---

## Command Cheat Sheet

| Command | Purpose |
|---------|---------|
| `ros2 topic list` | List all topics |
| `ros2 topic echo <topic>` | Show topic messages |
| `ros2 topic pub <topic> <type> <data>` | Publish to topic |
| `ros2 topic hz <topic>` | Check publish frequency |
| `ros2 node list` | List all nodes |
| `ros2 node info <node>` | Show node details |
| `ros2 service list` | List all services |
| `ros2 service call <service> <type> <data>` | Call a service |
| `ros2 param list <node>` | List node parameters |
| `ros2 param get <node> <param>` | Get parameter value |
| `ros2 param set <node> <param> <value>` | Set parameter value |
| `ros2 interface show <type>` | Show message definition |
| `ros2 bag record <topics>` | Record topics |
| `ros2 bag play <bag>` | Play recorded data |
| `ros2 run <pkg> <node>` | Run a node |
| `ros2 launch <pkg> <file>` | Launch a launch file |
| `rqt_graph` | Visualize node graph |
| `ros2 doctor` | Diagnose system issues |

---

## Common Patterns and Best Practices

### 1. Always Source Your Workspace

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 2. Use Tab Completion

The ROS 2 CLI supports tab completion. Press `Tab` twice to see available options:

```bash
ros2 topic <TAB><TAB>
# Shows: bw  delay  echo  find  hz  info  list  pub  type
```

### 3. Combine with Standard Linux Tools

```bash
# Filter topics by name
ros2 topic list | grep robot

# Count number of nodes
ros2 node list | wc -l

# Save topic output to file
ros2 topic echo /chatter > output.txt
```

### 4. Use Remapping for Testing

```bash
# Run same node with different topic
ros2 run demo_nodes_cpp talker --ros-args -r chatter:=/test_topic
```

---

## Troubleshooting with CLI Tools

### Problem: "No topics available"

```bash
# Check if any nodes are running
ros2 node list

# Check ROS domain ID
echo $ROS_DOMAIN_ID

# Try ros2 doctor
ros2 doctor
```

### Problem: "Service not available"

```bash
# List all services
ros2 service list

# Check node info to see which services it provides
ros2 node info /my_node

# Wait for service
ros2 service call /my_service std_srvs/srv/Empty --timeout 10.0
```

### Problem: "Messages not received"

```bash
# Check topic exists
ros2 topic list

# Check message type matches
ros2 topic info /my_topic

# Echo to verify data flow
ros2 topic echo /my_topic

# Check publish rate
ros2 topic hz /my_topic
```

---

## Hands-On Exercise

### Exercise: Complete CLI Workflow

1. **Start a talker node:**
   ```bash
   ros2 run demo_nodes_cpp talker
   ```

2. **In a new terminal, list all active nodes:**
   ```bash
   ros2 node list
   ```

3. **List all topics:**
   ```bash
   ros2 topic list -t
   ```

4. **Echo the chatter topic:**
   ```bash
   ros2 topic echo /chatter
   ```

5. **Check the publish rate:**
   ```bash
   ros2 topic hz /chatter
   ```

6. **In another terminal, manually publish a message:**
   ```bash
   ros2 topic pub --once /chatter std_msgs/msg/String "data: 'CLI Test Message'"
   ```

7. **Record 10 seconds of data:**
   ```bash
   ros2 bag record -o cli_test /chatter
   # Wait 10 seconds, then Ctrl+C
   ```

8. **Stop the talker and play back the recording:**
   ```bash
   ros2 bag play cli_test
   ```

9. **Visualize the node graph:**
   ```bash
   rqt_graph
   ```

---

## Summary

The ROS 2 CLI tools are essential for:
- **Inspection**: Understanding what's happening in your system
- **Debugging**: Finding and fixing communication issues
- **Testing**: Verifying functionality without writing code
- **Recording**: Capturing data for analysis and replay

Mastering these tools will significantly speed up your development and debugging workflow.

---

## Next Steps

- [First ROS 2 Node](../ch2-python-rclpy/first-node.md) - Create your first Python node
- [Message Types](../ch2-python-rclpy/message-types.md) - Learn about different message types
- [Debugging Guide](../ch4-packages/debugging.md) - Advanced debugging techniques

---

## Additional Resources

- [ROS 2 CLI Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
- [ROS 2 Command Cheat Sheet](https://github.com/ubuntu-robotics/ros2_cheats_sheet)
- [rqt Tools](https://docs.ros.org/en/humble/Concepts/About-RQt.html)
