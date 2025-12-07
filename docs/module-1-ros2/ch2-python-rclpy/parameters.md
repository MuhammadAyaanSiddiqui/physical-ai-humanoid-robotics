# ROS 2 Parameters

**Module**: The Robotic Nervous System
**Chapter**: Python Integration (rclpy)
**Estimated Time**: 2-3 hours
**Difficulty**: Intermediate

## Prerequisites

- Understanding of ROS 2 nodes and topics
- Ability to create Python nodes with rclpy
- Basic Python knowledge (dictionaries, types)

## Learning Objectives

By the end of this lesson, you will be able to:

- Understand what ROS 2 parameters are and when to use them
- Declare and use parameters in Python nodes
- Get and set parameters at runtime via CLI
- Load parameters from YAML files
- Implement parameter callbacks for dynamic reconfiguration
- Use parameters for node configuration

## What are Parameters?

**Parameters** are configuration values that can be set when a node starts or changed during runtime. They allow you to:
- Configure node behavior without recompiling code
- Tune algorithms (PID gains, thresholds, timeouts)
- Switch between modes (simulation vs. real robot)
- Share configuration across multiple nodes

### Parameters vs. Topics

| Feature | Parameters | Topics |
|---------|-----------|--------|
| **Purpose** | Configuration | Data streaming |
| **Update Rate** | Occasional (on change) | Continuous |
| **Direction** | Bi-directional (get/set) | Publisher → Subscriber |
| **Persistence** | Can load from file | Transient |
| **Use Case** | PID gains, timeouts | Sensor data, commands |

**Example**:
- **Parameter**: Maximum speed limit (set once, rarely changes)
- **Topic**: Current velocity command (updates 10-100 Hz)

## Parameter Types

ROS 2 supports these parameter types:

| Type | Python Type | Example |
|------|-------------|---------|
| `bool` | `bool` | `True`, `False` |
| `integer` | `int` | `42`, `-10` |
| `double` | `float` | `3.14`, `-0.5` |
| `string` | `str` | `"hello"`, `"config.yaml"` |
| `byte_array` | `bytes` | `b'\x01\x02\x03'` |
| `bool_array` | `list[bool]` | `[True, False, True]` |
| `integer_array` | `list[int]` | `[1, 2, 3]` |
| `double_array` | `list[float]` | `[1.0, 2.5, 3.7]` |
| `string_array` | `list[str]` | `["a", "b", "c"]` |

## Declaring Parameters

### Basic Declaration

```python
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Declare a parameter with a default value
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('enable_safety', True)

        # Get parameter values
        robot_name = self.get_parameter('robot_name').value
        max_speed = self.get_parameter('max_speed').value
        enable_safety = self.get_parameter('enable_safety').value

        self.get_logger().info(f'Robot: {robot_name}')
        self.get_logger().info(f'Max speed: {max_speed} m/s')
        self.get_logger().info(f'Safety: {enable_safety}')
```

### Why Declare Parameters?

In ROS 2, parameters **must be declared** before use:

```python
# Good: Declare first
self.declare_parameter('speed', 1.0)
speed = self.get_parameter('speed').value

# Bad: Will raise error if parameter not declared
speed = self.get_parameter('speed').value  # Error!
```

**Benefits of declaration**:
- Lists valid parameters for the node
- Provides default values
- Enables type checking
- Prevents typos (undeclared parameters raise errors)

### Declaring with Descriptors

Add descriptions and constraints:

```python
from rcl_interfaces.msg import ParameterDescriptor

self.declare_parameter(
    'max_speed',
    1.0,  # Default value
    ParameterDescriptor(
        description='Maximum robot speed in m/s',
        additional_constraints='Must be positive',
        read_only=False
    )
)
```

### Read-Only Parameters

```python
self.declare_parameter(
    'robot_id',
    'robot_001',
    ParameterDescriptor(read_only=True)
)
# Cannot be changed after node starts
```

## Getting Parameters

### Get Single Parameter

```python
# Get parameter value
speed = self.get_parameter('max_speed').value

# Get parameter object (has .value, .type, .name)
param = self.get_parameter('max_speed')
print(f'Name: {param.name}')
print(f'Value: {param.value}')
print(f'Type: {param.type_}')
```

### Get with Default (No Declaration)

```python
# Get parameter or use default if not set
speed = self.get_parameter_or('max_speed', 1.0).value
```

**Note**: Still requires declaration for strict parameter checking.

### Get All Parameters

```python
params = self.get_parameters([
    'robot_name',
    'max_speed',
    'enable_safety'
])

for param in params:
    self.get_logger().info(f'{param.name} = {param.value}')
```

## Setting Parameters

### Set During Initialization

Parameters can be set when creating the node:

**Via Command Line**:
```bash
ros2 run my_pkg my_node --ros-args -p max_speed:=2.0 -p robot_name:=robot_2
```

**Via Launch File** (covered in later lesson):
```python
Node(
    package='my_pkg',
    executable='my_node',
    parameters=[{
        'max_speed': 2.0,
        'robot_name': 'robot_2'
    }]
)
```

### Set at Runtime

```bash
# Using CLI
ros2 param set /my_node max_speed 2.5
```

**Response**:
```
Set parameter successful
```

### Set from Code

```python
from rclpy.parameter import Parameter

# Set parameter from within node
self.set_parameters([
    Parameter('max_speed', Parameter.Type.DOUBLE, 2.5)
])
```

## Parameter Callbacks

**Parameter callbacks** are called when parameters change:

```python
class DynamicNode(Node):
    def __init__(self):
        super().__init__('dynamic_node')

        # Declare parameter
        self.declare_parameter('speed', 1.0)

        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.current_speed = self.get_parameter('speed').value

    def parameter_callback(self, params):
        """Called when parameters are changed"""
        for param in params:
            if param.name == 'speed':
                self.current_speed = param.value
                self.get_logger().info(f'Speed updated to {param.value}')

        # Return success
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)
```

**Usage**:
```bash
# Terminal 1: Run node
ros2 run my_pkg dynamic_node

# Terminal 2: Change parameter
ros2 param set /dynamic_node speed 2.5
```

**Output**:
```
[INFO] [dynamic_node]: Speed updated to 2.5
```

### Validating Parameters

```python
def parameter_callback(self, params):
    from rcl_interfaces.msg import SetParametersResult

    for param in params:
        if param.name == 'speed':
            # Validate: must be positive
            if param.value <= 0.0:
                self.get_logger().error(f'Invalid speed: {param.value}')
                return SetParametersResult(successful=False)

            # Validate: must be below maximum
            if param.value > 5.0:
                self.get_logger().error(f'Speed too high: {param.value}')
                return SetParametersResult(successful=False)

            # Update if valid
            self.current_speed = param.value

    return SetParametersResult(successful=True)
```

## Loading Parameters from YAML Files

### Creating a YAML Config File

Create `config/params.yaml`:

```yaml
/**:
  ros__parameters:
    robot_name: "robot_1"
    max_speed: 1.5
    enable_safety: true
    sensor_topics:
      - "/scan"
      - "/camera"
      - "/imu"
```

**Wildcard Notation**:
- `/**:` applies to all nodes
- `/my_node:` applies only to `my_node`

### Loading via Command Line

```bash
ros2 run my_pkg my_node --ros-args --params-file config/params.yaml
```

### Loading via Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_pkg',
            executable='my_node',
            parameters=['/path/to/params.yaml']
        )
    ])
```

### Node-Specific Parameters

```yaml
/robot_controller:
  ros__parameters:
    max_speed: 2.0
    controller_gain: 1.5

/sensor_processor:
  ros__parameters:
    publish_rate: 30.0
    filter_size: 5
```

## CLI Parameter Operations

### List Parameters

```bash
ros2 param list /my_node
```

**Output**:
```
enable_safety
max_speed
robot_name
use_sim_time
```

### Get Parameter Value

```bash
ros2 param get /my_node max_speed
```

**Output**:
```
Double value is: 1.5
```

### Set Parameter Value

```bash
ros2 param set /my_node max_speed 2.0
```

### Dump Parameters to File

```bash
ros2 param dump /my_node > saved_params.yaml
```

**saved_params.yaml**:
```yaml
/my_node:
  ros__parameters:
    enable_safety: true
    max_speed: 2.0
    robot_name: robot_1
    use_sim_time: false
```

### Load Parameters from File

```bash
ros2 param load /my_node saved_params.yaml
```

## Practical Examples

### Example 1: PID Controller

```python
class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Declare PID gains
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.05)

        # Get initial values
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        # Allow runtime tuning
        self.add_on_set_parameters_callback(self.parameter_callback)

        # PID state
        self.integral = 0.0
        self.last_error = 0.0

        self.get_logger().info(
            f'PID Gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}'
        )

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'kp':
                self.kp = param.value
            elif param.name == 'ki':
                self.ki = param.value
            elif param.name == 'kd':
                self.kd = param.value

            self.get_logger().info(f'Updated {param.name} = {param.value}')

        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    def compute_control(self, error, dt):
        """Compute PID control output"""
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output
```

**Tuning**:
```bash
# Increase proportional gain
ros2 param set /pid_controller kp 1.5

# Reduce derivative gain
ros2 param set /pid_controller kd 0.03
```

### Example 2: Configurable Publisher

```python
class ConfigurablePublisher(Node):
    def __init__(self):
        super().__init__('configurable_publisher')

        # Declare configuration parameters
        self.declare_parameter('topic_name', 'chatter')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('message_prefix', 'Hello')

        # Get parameters
        topic = self.get_parameter('topic_name').value
        rate = self.get_parameter('publish_rate').value
        self.prefix = self.get_parameter('message_prefix').value

        # Create publisher with configured topic
        self.publisher_ = self.create_publisher(String, topic, 10)

        # Create timer with configured rate
        period = 1.0 / rate
        self.timer = self.create_timer(period, self.timer_callback)

        # Enable parameter updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.prefix} {self.counter}'
        self.publisher_.publish(msg)
        self.counter += 1

    def parameter_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == 'message_prefix':
                self.prefix = param.value
                self.get_logger().info(f'Prefix changed to: {self.prefix}')

        return SetParametersResult(successful=True)
```

**Usage**:
```bash
# Run with custom configuration
ros2 run my_pkg configurable_publisher --ros-args \
  -p topic_name:=custom_topic \
  -p publish_rate:=5.0 \
  -p message_prefix:=Greetings

# Change prefix at runtime
ros2 param set /configurable_publisher message_prefix "Hi"
```

### Example 3: Multi-Mode Robot

```python
class MultiModeRobot(Node):
    def __init__(self):
        super().__init__('multi_mode_robot')

        # Declare mode parameter
        self.declare_parameter('mode', 'manual')

        # Declare mode-specific parameters
        self.declare_parameter('auto_speed', 0.5)
        self.declare_parameter('manual_speed', 1.0)

        # Get current mode
        self.mode = self.get_parameter('mode').value
        self.update_mode()

        # Enable parameter updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Control timer
        self.create_timer(0.1, self.control_callback)

    def update_mode(self):
        """Update behavior based on mode"""
        if self.mode == 'auto':
            self.speed = self.get_parameter('auto_speed').value
            self.get_logger().info(f'Autonomous mode: {self.speed} m/s')
        elif self.mode == 'manual':
            self.speed = self.get_parameter('manual_speed').value
            self.get_logger().info(f'Manual mode: {self.speed} m/s')
        else:
            self.get_logger().error(f'Unknown mode: {self.mode}')

    def parameter_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == 'mode':
                if param.value not in ['auto', 'manual']:
                    return SetParametersResult(successful=False)
                self.mode = param.value
                self.update_mode()

        return SetParametersResult(successful=True)

    def control_callback(self):
        # Use self.speed which updates based on mode
        self.get_logger().debug(f'Running at {self.speed} m/s')
```

**Switching Modes**:
```bash
# Switch to autonomous mode
ros2 param set /multi_mode_robot mode auto

# Switch to manual mode
ros2 param set /multi_mode_robot mode manual
```

## Parameter Namespaces

Parameters can be organized hierarchically:

```python
# Declare nested parameters
self.declare_parameter('motors.left.max_speed', 1.0)
self.declare_parameter('motors.right.max_speed', 1.0)
self.declare_parameter('sensors.lidar.range_max', 10.0)
self.declare_parameter('sensors.camera.fps', 30)

# Get nested parameters
left_speed = self.get_parameter('motors.left.max_speed').value
lidar_range = self.get_parameter('sensors.lidar.range_max').value
```

**YAML**:
```yaml
/my_node:
  ros__parameters:
    motors:
      left:
        max_speed: 1.0
      right:
        max_speed: 1.0
    sensors:
      lidar:
        range_max: 10.0
      camera:
        fps: 30
```

## Best Practices

### 1. Declare All Parameters in __init__

```python
def __init__(self):
    super().__init__('my_node')

    # Good: Declare all parameters upfront
    self.declare_parameter('speed', 1.0)
    self.declare_parameter('timeout', 5.0)
    self.declare_parameter('enable', True)

    # Then use them
    self.speed = self.get_parameter('speed').value
```

### 2. Provide Sensible Defaults

```python
# Good: Reasonable defaults
self.declare_parameter('max_speed', 1.0)      # Safe speed
self.declare_parameter('timeout', 10.0)       # Reasonable timeout
self.declare_parameter('log_level', 'info')   # Standard logging

# Bad: Defaults that might cause issues
self.declare_parameter('max_speed', 100.0)    # Dangerously fast!
self.declare_parameter('timeout', 0.0)        # No timeout = infinite wait
```

### 3. Validate Parameter Changes

```python
def parameter_callback(self, params):
    from rcl_interfaces.msg import SetParametersResult

    for param in params:
        if param.name == 'speed':
            # Validate range
            if not (0.0 <= param.value <= 5.0):
                self.get_logger().error(
                    f'Speed {param.value} out of range [0, 5]'
                )
                return SetParametersResult(successful=False)

    return SetParametersResult(successful=True)
```

### 4. Use Descriptive Parameter Names

```python
# Good
self.declare_parameter('max_linear_velocity', 1.0)
self.declare_parameter('min_obstacle_distance', 0.5)

# Bad
self.declare_parameter('speed', 1.0)     # Which speed?
self.declare_parameter('dist', 0.5)      # Distance to what?
```

### 5. Document Parameters

```python
from rcl_interfaces.msg import ParameterDescriptor

self.declare_parameter(
    'pid_kp',
    1.0,
    ParameterDescriptor(
        description='Proportional gain for PID controller',
        additional_constraints='Range: 0.1 to 10.0'
    )
)
```

## Common Pitfalls

### Pitfall 1: Forgetting to Declare

```python
# Error: Parameter not declared
speed = self.get_parameter('speed').value  # Raises exception!

# Fix: Declare first
self.declare_parameter('speed', 1.0)
speed = self.get_parameter('speed').value
```

### Pitfall 2: Not Handling Parameter Callbacks

```python
# Bad: Parameter changes don't affect behavior
self.speed = self.get_parameter('speed').value
# Even if user changes parameter, self.speed stays the same

# Good: Update on parameter change
def parameter_callback(self, params):
    for param in params:
        if param.name == 'speed':
            self.speed = param.value
```

### Pitfall 3: Using Parameters for High-Frequency Data

```python
# Bad: Trying to use parameters for streaming data
self.set_parameters([Parameter('sensor_value', Parameter.Type.DOUBLE, reading)])

# Good: Use topics for streaming data
msg = Float64()
msg.data = reading
self.publisher_.publish(msg)
```

## Practice Exercises

### Exercise 1: Temperature Monitor

Create a node with parameters for `temp_min` and `temp_max`. Subscribe to a temperature topic and log warnings when temperature is out of range. Allow users to adjust thresholds at runtime.

### Exercise 2: Configurable Timer

Create a node with a `publish_rate` parameter. Use a parameter callback to recreate the timer with the new rate when the parameter changes.

### Exercise 3: Parameter File

Create a YAML file with parameters for a robot (name, max_speed, sensor_rate). Write a node that loads these parameters and logs them.

## Key Takeaways

- **Parameters** are configuration values for nodes
- **Declare parameters** in `__init__` with default values
- **Get parameters** with `get_parameter().value`
- **Set parameters** via CLI or parameter callbacks
- **Parameter callbacks** enable dynamic reconfiguration
- **Load from YAML** for complex configurations
- Parameters are for **configuration**, topics for **data streams**

## What's Next?

You've completed Chapter 2: Python Integration! Next:

- **Next Chapter**: [Robot Modeling (URDF)](../ch3-urdf/urdf-syntax.md)
- **Related**: [Launch Files](../ch4-packages/launch-files.md) - Using parameters in launch files

## Further Reading

- [ROS 2 Parameters Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html)
- [Using Parameters in Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [Parameter YAML Files](https://docs.ros.org/en/humble/How-To-Guides/Using-ros2-param.html)

---

**Checkpoint**: You can now configure ROS 2 nodes using parameters!
