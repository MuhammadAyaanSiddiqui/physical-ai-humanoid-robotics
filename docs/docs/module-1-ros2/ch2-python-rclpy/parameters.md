# ROS 2 Parameters

## Introduction

Parameters allow you to configure ROS 2 nodes at runtime without changing code. They enable flexibility and reusability, letting you tune behavior, adjust settings, and switch between different configurations. This lesson covers how to declare, use, and modify parameters in your nodes.

## Learning Objectives

By the end of this lesson, you will be able to:
- Declare and use parameters in ROS 2 nodes
- Get and set parameter values from Python code
- Modify parameters using command-line tools
- Load parameters from YAML files
- Implement parameter callbacks for dynamic reconfiguration
- Use parameters for robot configuration

## Prerequisites

- Understanding of ROS 2 nodes and topics
- Completed previous Python/rclpy lessons
- Basic Python knowledge

---

## Why Parameters?

**Without Parameters:**
```python
# Hardcoded values - requires code change
PUBLISH_RATE = 10.0
ROBOT_NAME = "atlas"
MAX_SPEED = 1.5
```

**With Parameters:**
```python
# Configurable at runtime
self.publish_rate = self.declare_parameter('publish_rate', 10.0).value
self.robot_name = self.declare_parameter('robot_name', 'atlas').value
self.max_speed = self.declare_parameter('max_speed', 1.5).value
```

**Benefits:**
- Change behavior without recompiling
- Reuse same node with different configurations
- Tune parameters during development
- Store configurations in files
- Different settings for different robots/environments

---

## Declaring Parameters

### Basic Parameter Declaration

```python
import rclpy
from rclpy.node import Node


class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('debug_mode', False)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.max_speed = self.get_parameter('max_speed').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # Use parameters
        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Max speed: {self.max_speed} m/s')
        self.get_logger().info(f'Debug mode: {self.debug_mode}')

        # Create timer with parameter-based rate
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f'{self.robot_name} timer callback')


def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### One-Line Declaration and Retrieval

```python
# Declare and get in one line
self.robot_name = self.declare_parameter('robot_name', 'atlas').value
self.max_speed = self.declare_parameter('max_speed', 1.5).value
```

---

## Parameter Types

ROS 2 supports several parameter types:

| Type | Python Type | Example |
|------|-------------|---------|
| Boolean | `bool` | `True`, `False` |
| Integer | `int` | `42`, `-10` |
| Double | `float` | `3.14`, `1.5` |
| String | `str` | `"hello"`, `"robot_1"` |
| Byte Array | `bytes` | `b'data'` |
| Boolean Array | `list[bool]` | `[True, False, True]` |
| Integer Array | `list[int]` | `[1, 2, 3, 4]` |
| Double Array | `list[float]` | `[1.0, 2.5, 3.7]` |
| String Array | `list[str]` | `["a", "b", "c"]` |

### Example with Different Types

```python
class MultiTypeParameterNode(Node):
    def __init__(self):
        super().__init__('multi_type_parameter_node')

        # Boolean
        self.declare_parameter('enable_debug', False)

        # Integer
        self.declare_parameter('queue_size', 10)

        # Double
        self.declare_parameter('rate_hz', 10.0)

        # String
        self.declare_parameter('frame_id', 'base_link')

        # Arrays
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3'])
        self.declare_parameter('joint_limits', [1.57, 3.14, 0.785])
        self.declare_parameter('sensor_ids', [1, 2, 3, 4])

        # Get values
        self.enable_debug = self.get_parameter('enable_debug').value
        self.queue_size = self.get_parameter('queue_size').value
        self.rate_hz = self.get_parameter('rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value
        self.joint_names = self.get_parameter('joint_names').value
        self.joint_limits = self.get_parameter('joint_limits').value
        self.sensor_ids = self.get_parameter('sensor_ids').value

        # Log all parameters
        self.log_parameters()

    def log_parameters(self):
        self.get_logger().info(f'Debug: {self.enable_debug}')
        self.get_logger().info(f'Queue size: {self.queue_size}')
        self.get_logger().info(f'Rate: {self.rate_hz} Hz')
        self.get_logger().info(f'Frame ID: {self.frame_id}')
        self.get_logger().info(f'Joint names: {self.joint_names}')
        self.get_logger().info(f'Joint limits: {self.joint_limits}')
        self.get_logger().info(f'Sensor IDs: {self.sensor_ids}')
```

---

## Setting Parameters from Command Line

### Using ros2 run

```bash
# Set single parameter
ros2 run my_package parameter_node --ros-args -p robot_name:=atlas

# Set multiple parameters
ros2 run my_package parameter_node --ros-args \
    -p robot_name:=atlas \
    -p publish_rate:=5.0 \
    -p max_speed:=2.0 \
    -p debug_mode:=true
```

### Using ros2 param

While node is running:

```bash
# Get parameter value
ros2 param get /parameter_node robot_name

# Set parameter value
ros2 param set /parameter_node publish_rate 20.0

# List all parameters
ros2 param list /parameter_node

# Describe parameter
ros2 param describe /parameter_node robot_name

# Dump all parameters to file
ros2 param dump /parameter_node > node_params.yaml

# Load parameters from file
ros2 param load /parameter_node node_params.yaml
```

---

## Loading Parameters from YAML Files

### Creating a YAML Parameter File

Create `params.yaml`:

```yaml
parameter_node:
  ros__parameters:
    robot_name: "atlas_v2"
    publish_rate: 15.0
    max_speed: 2.5
    debug_mode: true
    joint_names: ["shoulder", "elbow", "wrist"]
    joint_limits: [1.57, 3.14, 0.785]
```

### Loading in Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get path to parameter file
    params_file = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_package',
            executable='parameter_node',
            name='parameter_node',
            parameters=[params_file]
        )
    ])
```

### Loading from Command Line

```bash
ros2 run my_package parameter_node --ros-args --params-file params.yaml
```

---

## Parameter Callbacks (Dynamic Reconfiguration)

Monitor and respond to parameter changes:

```python
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult


class DynamicParameterNode(Node):
    def __init__(self):
        super().__init__('dynamic_parameter_node')

        # Declare parameters with descriptors
        speed_descriptor = ParameterDescriptor(
            description='Maximum robot speed in m/s',
            additional_constraints='Must be between 0.0 and 5.0'
        )

        self.declare_parameter('max_speed', 1.5, speed_descriptor)
        self.declare_parameter('robot_name', 'atlas')

        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.max_speed = self.get_parameter('max_speed').value
        self.robot_name = self.get_parameter('robot_name').value

        self.timer = self.create_timer(1.0, self.timer_callback)

    def parameters_callback(self, params):
        """Called when parameters are changed"""
        result = SetParametersResult(successful=True)

        for param in params:
            if param.name == 'max_speed':
                if 0.0 <= param.value <= 5.0:
                    self.max_speed = param.value
                    self.get_logger().info(f'Max speed updated to: {self.max_speed}')
                else:
                    result.successful = False
                    result.reason = 'max_speed must be between 0.0 and 5.0'

            elif param.name == 'robot_name':
                self.robot_name = param.value
                self.get_logger().info(f'Robot name updated to: {self.robot_name}')

        return result

    def timer_callback(self):
        self.get_logger().info(
            f'Robot: {self.robot_name}, Max Speed: {self.max_speed} m/s'
        )
```

**Testing Dynamic Parameters:**

```bash
# Terminal 1: Run node
ros2 run my_package dynamic_parameter_node

# Terminal 2: Change parameters
ros2 param set /dynamic_parameter_node max_speed 3.0
# Success! Logs show: "Max speed updated to: 3.0"

ros2 param set /dynamic_parameter_node max_speed 10.0
# Rejected! "max_speed must be between 0.0 and 5.0"
```

---

## Practical Example: Configurable Robot Controller

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult


class ConfigurableRobotController(Node):
    def __init__(self):
        super().__init__('configurable_robot_controller')

        # Declare parameters with descriptors
        self.declare_parameter(
            'max_linear_speed',
            1.0,
            ParameterDescriptor(description='Max linear speed (m/s)')
        )
        self.declare_parameter(
            'max_angular_speed',
            1.57,
            ParameterDescriptor(description='Max angular speed (rad/s)')
        )
        self.declare_parameter(
            'control_rate',
            10.0,
            ParameterDescriptor(description='Control loop rate (Hz)')
        )
        self.declare_parameter(
            'enable_safety_limits',
            True,
            ParameterDescriptor(description='Enable velocity safety limits')
        )

        # Get initial parameter values
        self.update_parameters()

        # Setup callback for parameter changes
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.battery_sub = self.create_subscription(
            Float32,
            'battery_voltage',
            self.battery_callback,
            10
        )

        # Create control timer based on control_rate parameter
        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.battery_voltage = 24.0
        self.target_linear = 0.0
        self.target_angular = 0.0

        self.get_logger().info('Configurable Robot Controller initialized')
        self.log_configuration()

    def update_parameters(self):
        """Get current parameter values"""
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.control_rate = self.get_parameter('control_rate').value
        self.enable_safety_limits = self.get_parameter('enable_safety_limits').value

    def parameters_callback(self, params):
        """Handle parameter changes"""
        result = SetParametersResult(successful=True)

        for param in params:
            if param.name == 'max_linear_speed':
                if param.value > 0.0:
                    self.max_linear_speed = param.value
                else:
                    result.successful = False
                    result.reason = 'max_linear_speed must be positive'

            elif param.name == 'max_angular_speed':
                if param.value > 0.0:
                    self.max_angular_speed = param.value
                else:
                    result.successful = False
                    result.reason = 'max_angular_speed must be positive'

            elif param.name == 'control_rate':
                if param.value > 0.0:
                    self.control_rate = param.value
                    # Recreate timer with new rate
                    self.timer.cancel()
                    timer_period = 1.0 / self.control_rate
                    self.timer = self.create_timer(timer_period, self.control_loop)
                else:
                    result.successful = False
                    result.reason = 'control_rate must be positive'

            elif param.name == 'enable_safety_limits':
                self.enable_safety_limits = param.value

        if result.successful:
            self.log_configuration()

        return result

    def battery_callback(self, msg):
        """Monitor battery voltage"""
        self.battery_voltage = msg.data

    def control_loop(self):
        """Main control loop"""
        cmd = Twist()

        # Get desired velocities (from some control algorithm)
        desired_linear = self.compute_desired_linear_velocity()
        desired_angular = self.compute_desired_angular_velocity()

        # Apply safety limits if enabled
        if self.enable_safety_limits:
            cmd.linear.x = self.clamp(
                desired_linear,
                -self.max_linear_speed,
                self.max_linear_speed
            )
            cmd.angular.z = self.clamp(
                desired_angular,
                -self.max_angular_speed,
                self.max_angular_speed
            )

            # Reduce speed if battery is low
            if self.battery_voltage < 20.0:
                cmd.linear.x *= 0.5
                cmd.angular.z *= 0.5
                self.get_logger().warn('Low battery! Reducing speed.')
        else:
            cmd.linear.x = desired_linear
            cmd.angular.z = desired_angular

        self.cmd_vel_pub.publish(cmd)

    def compute_desired_linear_velocity(self):
        """Placeholder for actual control logic"""
        return 0.5

    def compute_desired_angular_velocity(self):
        """Placeholder for actual control logic"""
        return 0.0

    def clamp(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min_val, min(value, max_val))

    def log_configuration(self):
        """Log current configuration"""
        self.get_logger().info('=== Configuration ===')
        self.get_logger().info(f'Max Linear Speed: {self.max_linear_speed} m/s')
        self.get_logger().info(f'Max Angular Speed: {self.max_angular_speed} rad/s')
        self.get_logger().info(f'Control Rate: {self.control_rate} Hz')
        self.get_logger().info(f'Safety Limits: {self.enable_safety_limits}')


def main(args=None):
    rclpy.init(args=args)
    controller = ConfigurableRobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Parameter File for Robot Controller

`robot_config.yaml`:

```yaml
configurable_robot_controller:
  ros__parameters:
    max_linear_speed: 2.0
    max_angular_speed: 2.0
    control_rate: 20.0
    enable_safety_limits: true
```

---

## Parameter Best Practices

### 1. Provide Sensible Defaults

```python
# Good: Reasonable defaults
self.declare_parameter('timeout', 5.0)
self.declare_parameter('retry_count', 3)

# Avoid: No context for defaults
self.declare_parameter('value', 0)
```

### 2. Use Descriptive Names

```python
# Good
self.declare_parameter('max_velocity_mps', 1.5)
self.declare_parameter('battery_warning_threshold_volts', 20.0)

# Avoid
self.declare_parameter('vel', 1.5)
self.declare_parameter('thresh', 20.0)
```

### 3. Add Parameter Descriptors

```python
descriptor = ParameterDescriptor(
    description='Maximum velocity in meters per second',
    additional_constraints='Range: 0.0 to 5.0',
    read_only=False
)
self.declare_parameter('max_velocity', 1.5, descriptor)
```

### 4. Validate Parameter Values

```python
def parameters_callback(self, params):
    result = SetParametersResult(successful=True)

    for param in params:
        if param.name == 'temperature_limit':
            if not (0.0 <= param.value <= 100.0):
                result.successful = False
                result.reason = 'temperature_limit must be 0-100'

    return result
```

### 5. Group Related Parameters

```yaml
robot_controller:
  ros__parameters:
    # Motion parameters
    max_linear_velocity: 1.5
    max_angular_velocity: 2.0
    acceleration_limit: 0.5

    # Safety parameters
    enable_collision_avoidance: true
    safety_distance: 0.5

    # Communication parameters
    publish_rate: 10.0
    queue_size: 10
```

---

## Hands-On Exercise

### Exercise 1: Configurable Publisher

Create a node that:
1. Publishes messages at a configurable rate
2. Allows changing the message content via parameter
3. Can enable/disable publishing via boolean parameter

```python
class ConfigurablePublisher(Node):
    def __init__(self):
        super().__init__('configurable_publisher')

        # TODO: Declare parameters
        # - publish_rate (float, default: 1.0)
        # - message_prefix (string, default: "Hello")
        # - enable_publishing (bool, default: True)

        # TODO: Create publisher and timer

        # TODO: Add parameter callback

    def timer_callback(self):
        # TODO: Publish only if enabled
        pass

    def parameters_callback(self, params):
        # TODO: Handle parameter changes
        pass
```

### Exercise 2: Robot Configuration

Create a configuration file for a humanoid robot with:
- Joint position limits (arrays)
- Control rates for different systems
- Safety thresholds
- Debug flags

---

## Summary

You've learned to:
- Declare and use parameters in nodes
- Load parameters from YAML files
- Modify parameters at runtime
- Implement parameter validation
- Use parameters for robot configuration

**Key Takeaways:**
- Parameters enable runtime configuration
- Use descriptive names and provide defaults
- Validate parameter changes with callbacks
- Store configurations in YAML files
- Parameters make nodes reusable

---

## Next Steps

- [URDF Syntax](../ch3-urdf/urdf-syntax.md) - Model robots with URDF
- [Workspace Setup](../ch4-packages/workspace-setup.md) - Create ROS 2 packages
- [Launch Files](../ch4-packages/launch-files.md) - Launch with parameters

---

## Additional Resources

- [ROS 2 Parameters Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [Parameter YAML Format](https://docs.ros.org/en/humble/How-To-Guides/Node-arguments.html)
- [rcl_interfaces/msg/ParameterDescriptor](https://docs.ros2.org/latest/api/rcl_interfaces/msg/ParameterDescriptor.html)
