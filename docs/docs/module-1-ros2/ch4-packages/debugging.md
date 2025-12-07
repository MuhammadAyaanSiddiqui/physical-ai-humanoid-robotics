# Debugging ROS 2 Systems

**Module**: The Robotic Nervous System
**Chapter**: Package Development
**Estimated Time**: 2-3 hours
**Difficulty**: Intermediate

## Prerequisites

- Understanding of ROS 2 nodes, topics, and packages
- Experience with launch files
- Basic command line proficiency
- Familiarity with Python or C++ debugging

## Learning Objectives

By the end of this lesson, you will be able to:

- Identify and fix common ROS 2 errors
- Use `ros2 doctor` to diagnose system issues
- Analyze ROS 2 logs effectively
- Debug node communication problems
- Troubleshoot build and runtime errors
- Use debugging tools (rqt, command line)
- Apply systematic debugging workflows
- Prevent common mistakes

## Common ROS 2 Errors

### 1. Package Not Found

**Error**:
```
Package 'my_pkg' not found
```

**Causes & Fixes**:

**Cause 1**: Package not built
```bash
# Fix: Build the package
cd ~/robot_ws
colcon build --packages-select my_pkg
source install/setup.bash
```

**Cause 2**: Workspace not sourced
```bash
# Fix: Source workspace
source ~/robot_ws/install/setup.bash
```

**Cause 3**: Wrong terminal
```bash
# Fix: Make sure you're in the right terminal with sourced workspace
# Add to ~/.bashrc for automatic sourcing:
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
```

**Verification**:
```bash
ros2 pkg list | grep my_pkg
```

### 2. Executable Not Found

**Error**:
```
Executable 'my_node' not found in package 'my_pkg'
```

**Python Package Fixes**:

**Check 1**: Entry point in `setup.py`
```python
entry_points={
    'console_scripts': [
        'my_node = my_pkg.my_node:main',  # Must match!
    ],
},
```

**Check 2**: File is executable
```bash
chmod +x src/my_pkg/my_pkg/my_node.py
```

**Check 3**: Rebuild and source
```bash
colcon build --packages-select my_pkg --symlink-install
source install/setup.bash
```

**C++ Package Fixes**:

**Check 1**: Executable added in `CMakeLists.txt`
```cmake
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp)

install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME})
```

**Check 2**: Rebuild
```bash
colcon build --packages-select my_pkg
source install/setup.bash
```

### 3. Import Errors (Python)

**Error**:
```
ModuleNotFoundError: No module named 'my_pkg'
```

**Fix 1**: Check `__init__.py` exists
```bash
ls src/my_pkg/my_pkg/__init__.py
# If missing:
touch src/my_pkg/my_pkg/__init__.py
```

**Fix 2**: Rebuild with symlink install
```bash
colcon build --packages-select my_pkg --symlink-install
source install/setup.bash
```

**Fix 3**: Check package structure
```
my_pkg/
├── package.xml
├── setup.py
└── my_pkg/              # Package name must match!
    ├── __init__.py      # Required!
    └── my_node.py
```

### 4. No Transform Available

**Error**:
```
Lookup would require extrapolation into the future.
Could not transform from 'base_link' to 'camera_link'
```

**Cause**: TF frames not being published

**Diagnosis**:
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Monitor specific transform
ros2 run tf2_ros tf2_echo base_link camera_link

# List available frames
ros2 topic echo /tf_static
```

**Fixes**:

**Fix 1**: Start robot_state_publisher
```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat robot.urdf)"
```

**Fix 2**: Publish joint states
```bash
ros2 run joint_state_publisher joint_state_publisher
```

**Fix 3**: Check URDF links are connected
```bash
check_urdf robot.urdf
# Look for "link [X] is not connected to any other link"
```

### 5. Topic Not Found

**Error**:
```
Could not find topic '/my_topic'
```

**Diagnosis**:
```bash
# List all topics
ros2 topic list

# Check if node is running
ros2 node list

# Check node's topics
ros2 node info /my_node
```

**Common Causes**:

**Cause 1**: Node not started
```bash
# Fix: Start the node
ros2 run my_pkg my_node
```

**Cause 2**: Topic name mismatch
```bash
# Publisher uses: /robot/cmd_vel
# Subscriber uses: /cmd_vel
# Fix: Use remapping
ros2 run my_pkg subscriber --ros-args -r cmd_vel:=robot/cmd_vel
```

**Cause 3**: Namespace mismatch
```bash
# Node in namespace 'robot1' publishes to '/robot1/cmd_vel'
# Check namespaces:
ros2 node list
```

### 6. Parameter Not Declared

**Error**:
```
Parameter 'my_param' has not been declared
```

**Fix**: Declare parameter before using
```python
# Wrong:
self.get_parameter('my_param')  # Not declared!

# Correct:
self.declare_parameter('my_param', 'default_value')
param = self.get_parameter('my_param').value
```

### 7. Message Type Mismatch

**Error**:
```
Incompatible type for message: expected std_msgs/msg/String but got geometry_msgs/msg/Twist
```

**Diagnosis**:
```bash
# Check topic type
ros2 topic info /my_topic

# Check message definition
ros2 interface show std_msgs/msg/String
```

**Fix**: Use correct message type in code
```python
# Wrong:
from std_msgs.msg import String
self.pub = self.create_publisher(String, '/cmd_vel', 10)

# Correct:
from geometry_msgs.msg import Twist
self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
```

## Using ros2 doctor

`ros2 doctor` checks your ROS 2 setup for common issues.

### Basic Check

```bash
ros2 doctor
```

**Example Output**:
```
Checking ROS 2 installation...
✓ ROS 2 Humble is installed
✓ ROS_DOMAIN_ID is set to 0
✓ ROS_LOCALHOST_ONLY is not set
✗ Multiple RMW implementations found (fastrtps, cyclonedds)
✓ Network connectivity OK

Checking running nodes...
✓ 3 nodes running
✗ Node '/my_node' not responding to echo

Warnings: 2
Errors: 0
```

### Detailed Report

```bash
ros2 doctor --report
```

Generates comprehensive system information:
- ROS 2 version and distribution
- RMW implementation
- Network configuration
- Running nodes and topics
- Package list
- Environment variables

**Save report to file**:
```bash
ros2 doctor --report > ros2_diagnostics.txt
```

### Checking Specific Components

```bash
# Check network setup
ros2 doctor --check-network

# Include platform info
ros2 doctor --include-platform
```

## Log Analysis

### Finding Logs

ROS 2 logs are stored in:
```bash
~/.ros/log/
```

**Structure**:
```
~/.ros/log/
└── 2024-01-15-14-30-45-123456/  # Timestamp
    ├── launch.log               # Launch file output
    ├── my_node-1.log            # Node stderr
    └── my_node-2.log            # Second instance
```

**Find latest log**:
```bash
ls -lt ~/.ros/log/ | head -n 5
```

### Reading Logs

```bash
# View node log
cat ~/.ros/log/latest/my_node-1.log

# Follow log in real-time
tail -f ~/.ros/log/latest/my_node-1.log

# Search logs for errors
grep -r "ERROR" ~/.ros/log/latest/
```

### Log Levels

ROS 2 log levels (severity increasing):

| Level | Use Case |
|-------|----------|
| DEBUG | Detailed debugging information |
| INFO  | General informational messages |
| WARN  | Warning messages (non-critical) |
| ERROR | Error messages (operation failed) |
| FATAL | Fatal errors (node cannot continue) |

### Setting Log Levels

**Per node** (command line):
```bash
ros2 run my_pkg my_node --ros-args --log-level debug
```

**Per logger** (in code):
```python
# Python
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

```cpp
// C++
auto logger = this->get_logger();
logger.set_level(rclcpp::Logger::Level::Debug);
```

**Environment variable** (all nodes):
```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1
ros2 run my_pkg my_node
```

### Logging in Code

**Python**:
```python
self.get_logger().debug('Debug message')
self.get_logger().info('Info message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal message')
```

**C++**:
```cpp
RCLCPP_DEBUG(this->get_logger(), "Debug message");
RCLCPP_INFO(this->get_logger(), "Info message");
RCLCPP_WARN(this->get_logger(), "Warning message");
RCLCPP_ERROR(this->get_logger(), "Error message");
RCLCPP_FATAL(this->get_logger(), "Fatal message");
```

## Debugging Tools

### 1. rqt_console

Real-time log viewer with filtering.

```bash
ros2 run rqt_console rqt_console
```

**Features**:
- Filter by severity, node, message content
- Color-coded log levels
- Pause/resume log stream
- Save logs to file

### 2. rqt_graph

Visualize node and topic connections.

```bash
ros2 run rqt_graph rqt_graph
```

**Use Cases**:
- Verify publishers/subscribers are connected
- Find topic name mismatches
- Identify orphaned nodes
- Visualize system architecture

**Options**:
- **Nodes only**: Show only nodes
- **Nodes/Topics (active)**: Show active connections
- **Nodes/Topics (all)**: Show all possible connections

### 3. rqt_topic

Monitor topic data.

```bash
ros2 run rqt_topic rqt_topic
```

**Features**:
- Real-time topic data visualization
- Plot numeric values over time
- Inspect message contents
- Publish test messages

### 4. rqt_tf_tree

Visualize TF tree.

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

Shows transform relationships between frames.

### 5. ros2 topic echo

Monitor topic messages:

```bash
# Echo topic
ros2 topic echo /my_topic

# Echo with message type
ros2 topic echo /scan sensor_msgs/msg/LaserScan

# Show only first message
ros2 topic echo /my_topic --once

# Show specific field
ros2 topic echo /odom --field pose.pose.position
```

### 6. ros2 topic hz

Check topic publish rate:

```bash
ros2 topic hz /my_topic
```

**Output**:
```
average rate: 10.023
	min: 0.099s max: 0.101s std dev: 0.00032s window: 100
```

**Use Case**: Verify nodes are publishing at expected rate.

### 7. ros2 topic info

Get topic information:

```bash
ros2 topic info /my_topic
```

**Output**:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 2
```

### 8. ros2 node info

Get node information:

```bash
ros2 node info /my_node
```

**Output**:
```
/my_node
  Subscribers:
    /input_topic: std_msgs/msg/String
  Publishers:
    /output_topic: std_msgs/msg/String
  Service Servers:
    /my_node/describe_parameters: rcl_interfaces/srv/DescribeParameters
  Service Clients:
  Action Servers:
  Action Clients:
```

## Build Errors

### Python Build Errors

**Error**: `setup.py not found`
```bash
# Check package structure
ls src/my_pkg/
# Should have: package.xml, setup.py, setup.cfg

# Verify setup.py exists and is correct
cat src/my_pkg/setup.py
```

**Error**: `Package 'my_pkg' not found in workspace`
```bash
# Make sure package.xml <name> matches directory
cat src/my_pkg/package.xml | grep "<name>"
```

### C++ Build Errors

**Error**: `CMakeLists.txt not found`
```bash
# Check package structure
ls src/my_cpp_pkg/
# Should have: package.xml, CMakeLists.txt, src/
```

**Error**: `Could not find package 'rclcpp'`
```bash
# Add to CMakeLists.txt
find_package(rclcpp REQUIRED)

# Add to package.xml
<depend>rclcpp</depend>

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

**Error**: Compilation errors
```bash
# Build with verbose output
colcon build --packages-select my_pkg --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

# Check for:
# - Missing includes
# - Undefined references
# - Type mismatches
```

### Dependency Errors

**Error**: `Package 'X' not found`
```bash
# Check package.xml has dependency
cat src/my_pkg/package.xml | grep "<depend>X</depend>"

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Update rosdep database
rosdep update
```

## Runtime Debugging Workflows

### Workflow 1: Node Not Communicating

**Symptom**: Subscriber not receiving messages

**Debug Steps**:

1. **Verify publisher is running**:
```bash
ros2 node list | grep publisher_node
```

2. **Check topic is being published**:
```bash
ros2 topic list | grep /my_topic
ros2 topic hz /my_topic
```

3. **Verify subscriber is running**:
```bash
ros2 node list | grep subscriber_node
```

4. **Check topic names match exactly**:
```bash
# Publisher's topics
ros2 node info /publisher_node

# Subscriber's topics
ros2 node info /subscriber_node
```

5. **Check message types match**:
```bash
ros2 topic info /my_topic
```

6. **Visualize connections**:
```bash
ros2 run rqt_graph rqt_graph
```

7. **Test with ros2 topic echo**:
```bash
ros2 topic echo /my_topic
```

### Workflow 2: Slow or Frozen Node

**Symptom**: Node appears to hang or runs slowly

**Debug Steps**:

1. **Check if node is alive**:
```bash
ros2 node list
ros2 node info /my_node
```

2. **Check CPU usage**:
```bash
top -p $(pgrep -f my_node)
```

3. **Check for deadlocks** (C++):
   - Add log statements before/after locks
   - Use thread sanitizer: `-DCMAKE_BUILD_TYPE=Debug`

4. **Profile callback execution**:
```python
import time

def callback(self, msg):
    start = time.time()
    # ... processing ...
    elapsed = time.time() - start
    self.get_logger().info(f'Callback took {elapsed:.3f}s')
```

5. **Check for blocking operations**:
   - Network calls
   - File I/O
   - Heavy computation
   - Use timers instead of sleep in callbacks

### Workflow 3: Memory Leak

**Symptom**: Memory usage grows over time

**Debug Steps**:

1. **Monitor memory**:
```bash
# Watch memory usage
watch -n 1 'ps aux | grep my_node'
```

2. **Use valgrind** (C++):
```bash
valgrind --leak-check=full ros2 run my_pkg my_node
```

3. **Check for Python memory leaks**:
```python
import tracemalloc

tracemalloc.start()

# ... run code ...

snapshot = tracemalloc.take_snapshot()
top_stats = snapshot.statistics('lineno')

for stat in top_stats[:10]:
    print(stat)
```

4. **Common causes**:
   - Not destroying publishers/subscribers
   - Not clearing message buffers
   - Circular references (Python)
   - Unfreed pointers (C++)

### Workflow 4: Launch File Issues

**Symptom**: Nodes don't start or crash immediately

**Debug Steps**:

1. **Check launch file syntax**:
```bash
ros2 launch my_pkg my_launch.py --show-args
```

2. **Run with debug output**:
```bash
ros2 launch my_pkg my_launch.py --debug
```

3. **Check individual nodes work**:
```bash
# Test each node separately
ros2 run my_pkg node1
ros2 run my_pkg node2
```

4. **Check file paths**:
```python
# Add logging to launch file
from launch.actions import LogInfo

return LaunchDescription([
    LogInfo(msg=f'Config file: {config_file}'),
    # ...
])
```

5. **Verify parameters**:
```bash
# After launch, check parameters were set
ros2 param list /my_node
ros2 param get /my_node my_param
```

## Debugging Strategies

### 1. Divide and Conquer

**Break down the problem**:
- Test each component separately
- Use simple test cases first
- Add complexity incrementally

### 2. Print Debugging

**Add log statements**:
```python
self.get_logger().info(f'Variable x = {x}')
self.get_logger().info(f'Entering function foo()')
self.get_logger().info(f'Message received: {msg.data}')
```

### 3. Binary Search

**Comment out code sections**:
```python
def callback(self, msg):
    # self.process_part1(msg)  # Comment out
    self.process_part2(msg)
    # self.process_part3(msg)  # Comment out
```

Find which part causes the issue.

### 4. Minimal Reproducible Example

**Create simple test case**:
```python
# Minimal publisher
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_pub')
        self.pub = self.create_publisher(String, 'test', 10)
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):
        msg = String()
        msg.data = 'test'
        self.pub.publish(msg)
        self.get_logger().info('Published!')
```

If this works, gradually add complexity until it breaks.

### 5. Check Assumptions

**Verify your assumptions**:
```python
# Don't assume:
value = self.get_parameter('my_param').value

# Verify:
if self.has_parameter('my_param'):
    value = self.get_parameter('my_param').value
    self.get_logger().info(f'Parameter value: {value}')
else:
    self.get_logger().error('Parameter not set!')
```

## Common Mistakes to Avoid

### 1. Not Sourcing Workspace

```bash
# Wrong: Run node without sourcing
ros2 run my_pkg my_node  # Package not found!

# Correct:
source ~/robot_ws/install/setup.bash
ros2 run my_pkg my_node
```

### 2. Hardcoded Paths

```python
# Wrong: Hardcoded path
urdf_file = '/home/user/my_robot/robot.urdf'  # Breaks on other machines!

# Correct: Use package share directory
from ament_index_python.packages import get_package_share_directory
urdf_file = os.path.join(get_package_share_directory('my_pkg'), 'urdf', 'robot.urdf')
```

### 3. Ignoring Warnings

```
Warning: QoS mismatch detected
```

**Don't ignore warnings!** They often indicate future problems.

### 4. Not Checking Return Values

```python
# Wrong: Assume success
self.call_service(request)

# Correct: Check response
response = self.call_service(request)
if response.success:
    self.get_logger().info('Service succeeded')
else:
    self.get_logger().error(f'Service failed: {response.message}')
```

### 5. Blocking in Callbacks

```python
# Wrong: Blocking operation in callback
def callback(self, msg):
    time.sleep(5)  # Blocks executor!
    self.process(msg)

# Correct: Use timer or async
def callback(self, msg):
    self.queue.put(msg)  # Non-blocking

def timer_callback(self):
    if not self.queue.empty():
        msg = self.queue.get()
        self.process(msg)
```

## Debugging Checklist

When facing an issue, go through this checklist:

- [ ] Is ROS 2 sourced? (`source /opt/ros/humble/setup.bash`)
- [ ] Is workspace sourced? (`source ~/robot_ws/install/setup.bash`)
- [ ] Is the package built? (`colcon build --packages-select my_pkg`)
- [ ] Are all nodes running? (`ros2 node list`)
- [ ] Do topic names match exactly? (`ros2 topic list`)
- [ ] Do message types match? (`ros2 topic info /topic`)
- [ ] Are parameters declared? (`ros2 param list /node`)
- [ ] Are TF frames being published? (`ros2 run tf2_tools view_frames`)
- [ ] Any errors in logs? (`tail -f ~/.ros/log/latest/*.log`)
- [ ] Does `ros2 doctor` show issues? (`ros2 doctor`)

## Practice Exercises

### Exercise 1: Debug Communication Issue

Given two nodes that should communicate but don't:
1. Use `ros2 node list` and `ros2 topic list` to diagnose
2. Identify the mismatch (topic name, namespace, or type)
3. Fix using remapping or code changes

### Exercise 2: Debug Build Error

Given a package that won't build:
1. Identify missing dependencies in package.xml
2. Use `rosdep` to install them
3. Fix CMakeLists.txt or setup.py issues
4. Successfully build the package

### Exercise 3: Performance Debug

Given a slow node:
1. Add timing logs to identify bottleneck
2. Profile callback execution times
3. Optimize the slowest part
4. Verify improvement with `ros2 topic hz`

## Key Takeaways

- **`ros2 doctor`** quickly diagnoses common ROS 2 issues
- **Always check**: sourced workspace, built packages, running nodes
- **Logs** are in `~/.ros/log/` - check them for errors
- **rqt tools** (rqt_graph, rqt_console) visualize system state
- **Topic echo** and **node info** verify communication
- **Systematic workflow**: isolate, test, verify, fix
- **Common mistakes**: not sourcing, hardcoded paths, blocking callbacks
- **Prevention**: good logging, error checking, testing

## What's Next?

Now that you can debug ROS 2 systems:

- **Next Lesson**: [Assessment](./assessment.md) - Apply your skills to a complete project
- **Related**: [Launch Files](./launch-files.md) - Advanced launch debugging

## Further Reading

- [ROS 2 Logging Documentation](https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html)
- [ROS 2 Troubleshooting Guide](https://docs.ros.org/en/humble/How-To-Guides/Installation-Troubleshooting.html)
- [rqt User Guide](https://docs.ros.org/en/humble/Concepts/About-RQt.html)

---

**Checkpoint**: You can now systematically debug and troubleshoot ROS 2 systems!
