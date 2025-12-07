# ROS 2 Launch Files

**Module**: The Robotic Nervous System
**Chapter**: Package Development
**Estimated Time**: 2-3 hours
**Difficulty**: Intermediate

## Prerequisites

- Understanding of ROS 2 nodes and packages
- Colcon workspace setup completed
- Basic Python programming knowledge
- Familiarity with ROS 2 CLI tools

## Learning Objectives

By the end of this lesson, you will be able to:

- Understand the purpose and benefits of launch files
- Write Python launch files for ROS 2
- Launch multiple nodes with a single command
- Pass parameters and arguments to nodes
- Use launch file actions and events
- Include other launch files
- Debug launch file issues
- Create reusable launch configurations

## What is a Launch File?

A **launch file** is a script that automates starting multiple ROS 2 nodes, setting parameters, and configuring a complete robot system with a single command.

### Why Use Launch Files?

**Without Launch Files** (manual startup):
```bash
# Terminal 1
ros2 run robot_state_publisher robot_state_publisher

# Terminal 2
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3
ros2 run rviz2 rviz2

# Terminal 4
ros2 run my_pkg robot_controller

# Terminal 5
ros2 run my_pkg sensor_driver
```

**With Launch File** (automated):
```bash
ros2 launch my_pkg robot.launch.py
```

**Benefits**:
- Start entire system with one command
- Reproducible robot configurations
- Easy parameter management
- Conditional node launching
- Event-based coordination
- Version-controlled system setup

## Launch File Format

ROS 2 supports three launch file formats:

| Format | Extension | Language | Recommended |
|--------|-----------|----------|-------------|
| Python | `.launch.py` | Python | ✅ Yes (most flexible) |
| XML | `.launch.xml` | XML | Limited use |
| YAML | `.launch.yaml` | YAML | Limited use |

**We'll focus on Python launch files** as they offer the most flexibility and are the ROS 2 standard.

## Basic Launch File Structure

### Minimal Example

Create `my_first_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='listener'
        ),
    ])
```

**Key Components**:
1. **`generate_launch_description()`**: Entry point function (required)
2. **`LaunchDescription`**: Container for launch actions
3. **`Node`**: Action to start a ROS 2 node

**Run it**:
```bash
ros2 launch my_first_launch.py
```

## Node Actions

### Basic Node Configuration

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',        # Package name
            executable='controller',        # Executable name (from setup.py)
            name='robot_controller',        # Node name (overrides default)
            namespace='robot1',             # Node namespace
            output='screen',                # Show output in console
            parameters=[{
                'update_rate': 50.0,
                'use_sim_time': True,
            }],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[
                ('input_topic', 'sensors/camera'),
                ('output_topic', 'control/cmd_vel'),
            ],
        ),
    ])
```

### Node Parameters

**Parameter Dictionary**:
```python
Node(
    package='my_pkg',
    executable='my_node',
    parameters=[{
        'param1': 'value1',
        'param2': 42,
        'param3': 3.14,
        'param4': True,
    }]
)
```

**Parameter File (YAML)**:
```python
import os
from ament_index_python.packages import get_package_share_directory

config_file = os.path.join(
    get_package_share_directory('my_pkg'),
    'config',
    'params.yaml'
)

Node(
    package='my_pkg',
    executable='my_node',
    parameters=[config_file]
)
```

**params.yaml** example:
```yaml
/**:
  ros__parameters:
    update_rate: 50.0
    use_sim_time: true
    max_velocity: 1.5
```

### Topic Remapping

```python
Node(
    package='my_pkg',
    executable='my_node',
    remappings=[
        ('original_topic', 'new_topic'),
        ('cmd_vel', 'robot1/cmd_vel'),
        ('scan', 'sensors/lidar'),
    ]
)
```

### Output Handling

```python
# Show output in console (recommended for debugging)
Node(
    package='my_pkg',
    executable='my_node',
    output='screen'
)

# Log to file
Node(
    package='my_pkg',
    executable='my_node',
    output='log'
)

# Suppress output
Node(
    package='my_pkg',
    executable='my_node',
    output='none'
)
```

## Launch Arguments

### Declaring Arguments

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='my_robot',
        description='Name of the robot'
    )

    # Use arguments in nodes
    my_node = Node(
        package='my_pkg',
        executable='my_node',
        name=LaunchConfiguration('robot_name'),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        my_node,
    ])
```

**Running with Arguments**:
```bash
# Use defaults
ros2 launch my_pkg my_launch.py

# Override arguments
ros2 launch my_pkg my_launch.py use_sim_time:=true robot_name:=robot1
```

### Argument Choices

Restrict argument values:

```python
DeclareLaunchArgument(
    'log_level',
    default_value='info',
    description='Logging level',
    choices=['debug', 'info', 'warn', 'error', 'fatal']
)
```

## Including Other Launch Files

### Basic Include

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to other launch file
    other_launch_file = os.path.join(
        get_package_share_directory('other_pkg'),
        'launch',
        'other.launch.py'
    )

    # Include it
    include_other = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file)
    )

    return LaunchDescription([
        include_other,
    ])
```

### Include with Arguments

```python
include_with_args = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(other_launch_file),
    launch_arguments={
        'use_sim_time': 'true',
        'robot_name': 'robot1',
    }.items()
)
```

## Conditional Launching

### Launch Based on Argument

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz if true'
    )

    # Only launch if use_rviz is true
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        rviz_node,
    ])
```

**Usage**:
```bash
# With RViz (default)
ros2 launch my_pkg demo.launch.py

# Without RViz
ros2 launch my_pkg demo.launch.py use_rviz:=false
```

### Unless Condition

```python
from launch.conditions import UnlessCondition

# Launch only if simulation is false
real_robot_node = Node(
    package='my_pkg',
    executable='real_robot_driver',
    condition=UnlessCondition(LaunchConfiguration('use_sim'))
)
```

## Complete Example: Robot Visualization

### Launch File

`display_robot.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package paths
    pkg_share = get_package_share_directory('my_robot_description')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(pkg_share, 'urdf', 'robot.urdf'),
        description='Path to robot URDF file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'rviz', 'robot.rviz'),
        description='Path to RViz config file'
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Start joint_state_publisher_gui'
    )

    # Get configurations
    urdf_file = LaunchConfiguration('urdf_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')

    # Read URDF file
    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(use_gui),
        output='screen'
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        urdf_file_arg,
        rviz_config_arg,
        use_gui_arg,
        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
```

**Usage**:
```bash
# Default (with GUI)
ros2 launch my_robot_description display_robot.launch.py

# Without GUI
ros2 launch my_robot_description display_robot.launch.py use_gui:=false

# Custom URDF
ros2 launch my_robot_description display_robot.launch.py urdf_file:=/path/to/my.urdf
```

## Launch File Organization in Packages

### Python Package Structure

```
my_robot_pkg/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
├── my_robot_pkg/
│   └── __init__.py
├── launch/                 # Launch files here
│   ├── robot.launch.py
│   ├── simulation.launch.py
│   └── navigation.launch.py
└── config/                 # Parameter files
    └── params.yaml
```

### setup.py Configuration

Add launch files to installation:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='My robot package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_pkg.my_node:main',
        ],
    },
)
```

### CMake Package Structure (C++)

```
my_robot_cpp_pkg/
├── package.xml
├── CMakeLists.txt
├── launch/
│   └── robot.launch.py
├── config/
│   └── params.yaml
└── src/
    └── my_node.cpp
```

### CMakeLists.txt Configuration

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_cpp_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add executable
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp)

# Install executables
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

## Advanced Launch Actions

### ExecuteProcess

Run external commands:

```python
from launch.actions import ExecuteProcess

# Start Gazebo simulator
gazebo = ExecuteProcess(
    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
    output='screen'
)
```

### SetEnvironmentVariable

```python
from launch.actions import SetEnvironmentVariable

set_env = SetEnvironmentVariable(
    'GAZEBO_MODEL_PATH',
    '/home/user/my_models'
)
```

### TimerAction

Delay launching a node:

```python
from launch.actions import TimerAction

# Start camera node after 5 seconds
delayed_camera = TimerAction(
    period=5.0,
    actions=[
        Node(
            package='camera_driver',
            executable='camera_node'
        )
    ]
)
```

### GroupAction

Group related actions:

```python
from launch.actions import GroupAction

sensor_group = GroupAction([
    Node(package='lidar_driver', executable='lidar'),
    Node(package='camera_driver', executable='camera'),
    Node(package='imu_driver', executable='imu'),
])
```

## Event Handlers

### OnProcessExit

Launch action when a node exits:

```python
from launch.event_handlers import OnProcessExit

calibration_node = Node(
    package='my_pkg',
    executable='calibration'
)

# Start main controller after calibration exits
controller_on_exit = OnProcessExit(
    target_action=calibration_node,
    on_exit=[
        Node(
            package='my_pkg',
            executable='controller'
        )
    ]
)

return LaunchDescription([
    calibration_node,
    controller_on_exit,
])
```

### OnProcessStart

```python
from launch.event_handlers import OnProcessStart

main_node = Node(
    package='my_pkg',
    executable='main'
)

# Log message when node starts
on_start_handler = OnProcessStart(
    target_action=main_node,
    on_start=[
        ExecuteProcess(
            cmd=['echo', 'Main node started!']
        )
    ]
)
```

## Substitutions

### Common Substitutions

```python
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.substitutions import FindPackageShare

# Launch configuration (from arguments)
robot_name = LaunchConfiguration('robot_name')

# Find package share directory
pkg_share = FindPackageShare('my_pkg')

# Join paths
config_file = PathJoinSubstitution([
    pkg_share,
    'config',
    'params.yaml'
])

# Text substitution
greeting = TextSubstitution(text='Hello, ROS 2!')

# Execute command and use output
robot_description = Command(['cat ', urdf_file])
```

### Using Substitutions

```python
Node(
    package='my_pkg',
    executable='my_node',
    name=[LaunchConfiguration('robot_name'), '_controller'],
    parameters=[{
        'config_file': PathJoinSubstitution([
            FindPackageShare('my_pkg'),
            'config',
            LaunchConfiguration('config_name')
        ])
    }]
)
```

## Debugging Launch Files

### Print to Console

```python
from launch.actions import LogInfo

return LaunchDescription([
    LogInfo(msg='Starting robot launch file...'),
    my_node,
    LogInfo(msg='Robot launch complete!'),
])
```

### Check Arguments

```python
ros2 launch my_pkg robot.launch.py --show-args
```

**Output**:
```
Arguments (pass arguments as '<name>:=<value>'):

    'use_sim_time':
        Use simulation time if true
        (default: 'false')

    'robot_name':
        Name of the robot
        (default: 'my_robot')
```

### Verbose Launch

```python
ros2 launch my_pkg robot.launch.py --debug
```

Shows detailed launch process information.

### Dry Run

```python
ros2 launch my_pkg robot.launch.py --print-description
```

Shows what would be launched without actually launching.

## Best Practices

### 1. Use Descriptive Argument Names

**Good**:
```python
DeclareLaunchArgument('robot_urdf_file', ...)
```

**Bad**:
```python
DeclareLaunchArgument('file', ...)
```

### 2. Provide Default Values and Descriptions

```python
DeclareLaunchArgument(
    'update_rate',
    default_value='50.0',
    description='Control loop update rate in Hz'
)
```

### 3. Use Package Share Directory

**Good** (works anywhere):
```python
urdf_file = os.path.join(
    get_package_share_directory('my_pkg'),
    'urdf',
    'robot.urdf'
)
```

**Bad** (hardcoded path):
```python
urdf_file = '/home/user/my_robot/robot.urdf'  # Don't do this!
```

### 4. Separate Configuration from Launch Logic

Use YAML files for parameters:

```python
# Launch file
config = PathJoinSubstitution([pkg_share, 'config', 'robot.yaml'])
Node(parameters=[config], ...)

# robot.yaml
/**:
  ros__parameters:
    max_speed: 1.5
    timeout: 5.0
```

### 5. Use Conditional Launching

```python
# Enable/disable components
use_lidar_arg = DeclareLaunchArgument('use_lidar', default_value='true')
lidar_node = Node(..., condition=IfCondition(LaunchConfiguration('use_lidar')))
```

### 6. Group Related Nodes

```python
sensors_group = GroupAction(
    actions=[lidar_node, camera_node, imu_node],
    condition=IfCondition(LaunchConfiguration('use_sensors'))
)
```

## Common Patterns

### Multi-Robot Launch

```python
def generate_launch_description():
    robots = []
    for i in range(3):
        robot = Node(
            package='my_pkg',
            executable='robot_controller',
            namespace=f'robot{i}',
            parameters=[{
                'robot_id': i,
            }]
        )
        robots.append(robot)

    return LaunchDescription(robots)
```

### Simulation vs Real Robot

```python
use_sim_arg = DeclareLaunchArgument('use_sim', default_value='true')

sim_node = Node(
    package='gazebo_ros',
    executable='gazebo',
    condition=IfCondition(LaunchConfiguration('use_sim'))
)

real_robot_node = Node(
    package='robot_driver',
    executable='driver',
    condition=UnlessCondition(LaunchConfiguration('use_sim'))
)
```

## Practice Exercises

### Exercise 1: Multi-Node Launch

Create a launch file that starts:
1. A publisher node
2. A subscriber node
3. RViz
4. rqt_graph

Add arguments to enable/disable each component.

### Exercise 2: Parameter Configuration

Create a launch file that:
1. Loads parameters from a YAML file
2. Allows overriding specific parameters via launch arguments
3. Passes parameters to multiple nodes

### Exercise 3: Robot Bringup

Create a complete robot bringup launch file:
1. Load robot URDF
2. Start robot_state_publisher
3. Start sensor drivers (lidar, camera, IMU)
4. Start control nodes
5. Optionally start RViz

## Troubleshooting

### Issue 1: Launch File Not Found

**Error**: `Package 'my_pkg' not found`

**Fix**:
1. Build package: `colcon build --packages-select my_pkg`
2. Source workspace: `source install/setup.bash`
3. Check setup.py includes launch files

### Issue 2: Arguments Not Working

**Error**: Argument value not applied

**Check**:
```python
# Make sure you're using LaunchConfiguration, not the string
parameters=[{
    'param': LaunchConfiguration('my_arg'),  # Correct
    # 'param': 'my_arg',  # Wrong - uses literal string!
}]
```

### Issue 3: Nodes Start in Wrong Order

**Solution**: Use event handlers:
```python
OnProcessExit(
    target_action=first_node,
    on_exit=[second_node]
)
```

### Issue 4: Path Not Found

**Error**: `FileNotFoundError: [Errno 2] No such file or directory`

**Fix**: Use `get_package_share_directory`:
```python
from ament_index_python.packages import get_package_share_directory
pkg_share = get_package_share_directory('my_pkg')
file_path = os.path.join(pkg_share, 'config', 'my_file.yaml')
```

## Key Takeaways

- **Launch files** automate starting complex multi-node systems
- **Python launch files** (`.launch.py`) are the most flexible format
- **`generate_launch_description()`** is the entry point function
- **Node actions** start ROS 2 nodes with parameters and remappings
- **Arguments** make launch files configurable and reusable
- **Conditions** enable/disable components based on arguments
- **Include actions** compose launch files from smaller pieces
- **Event handlers** coordinate node startup sequencing
- **Always use `get_package_share_directory`** for portable paths

## What's Next?

Now that you can create launch files:

- **Next Lesson**: [Debugging](./debugging.md) - Troubleshoot ROS 2 systems
- **Related**: [Workspace Setup](./workspace-setup.md) - Review colcon workspace basics

## Further Reading

- [ROS 2 Launch Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [Launch File Examples](https://github.com/ros2/launch_ros/tree/humble/launch_ros/examples)
- [Launch File Architecture](https://design.ros2.org/articles/roslaunch.html)

---

**Checkpoint**: You can now create and use launch files to automate robot system startup!
