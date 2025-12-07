---
sidebar_position: 1
---

# Module 1: ROS 2 Robot Control Mastery

## Module Overview

This module teaches you to build robot control systems using ROS 2 Humble. Over 16 lessons across 4 chapters, you'll master nodes, topics, services, URDF modeling, and package development.

**Duration**: Weeks 3-5 (3 weeks)
**Priority**: P1 (Core Foundation)
**Prerequisites**: ROS 2 Humble installed, Python basics

## Learning Outcomes

By completing this module, you will:

- ✅ Create ROS 2 nodes in Python (rclpy)
- ✅ Implement publish-subscribe communication with topics
- ✅ Use services and actions for request-response patterns
- ✅ Model humanoid robots in URDF/XML
- ✅ Visualize robots in RViz
- ✅ Build complete ROS 2 packages with colcon
- ✅ Write Python launch files
- ✅ Debug ROS 2 systems with CLI tools

## Module Structure

### Chapter 1: ROS 2 Fundamentals
**Goal**: Understand ROS 2 architecture and core concepts

1. **Installation** (45 min)
   - Ubuntu 22.04 + ROS 2 Humble setup
   - Verify with talker/listener demo
   - Install development tools (colcon, rosdep)

2. **Nodes and Topics** (45 min)
   - Publisher-subscriber pattern
   - Message types and QoS
   - Visualizing with rqt_graph

3. **Services and Actions** (45 min)
   - Request-response with services
   - Goal-based async actions
   - When to use each pattern

4. **CLI Tools** (30 min)
   - `ros2 topic`, `ros2 node`, `ros2 service`
   - Debugging with `ros2 doctor`
   - Introspection commands

### Chapter 2: Python Integration (rclpy)
**Goal**: Write production-ready ROS 2 nodes in Python

1. **First Node** (30 min)
   - Minimal publisher example
   - Node lifecycle (init, spin, shutdown)
   - Logging best practices

2. **Message Types** (45 min)
   - Standard messages (`std_msgs`, `sensor_msgs`, `geometry_msgs`)
   - Custom message definitions
   - Message serialization

3. **Timers and Callbacks** (45 min)
   - Event-driven node design
   - Timer-based publishing
   - Callback groups for parallelism

4. **Parameters** (30 min)
   - Runtime configuration
   - Parameter types and validation
   - Dynamic reconfiguration

### Chapter 3: Robot Modeling (URDF)
**Goal**: Model humanoid robots with URDF for simulation and visualization

1. **URDF Syntax** (45 min)
   - XML structure (links, joints, collisions)
   - Visual vs collision geometry
   - Inertial properties

2. **Humanoid Description** (60 min)
   - 10-joint humanoid model
   - Kinematic chains (torso → legs, arms)
   - Joint types (revolute, prismatic, fixed)

3. **RViz Visualization** (30 min)
   - Loading URDF in RViz
   - Joint State Publisher
   - Interactive markers

4. **Joint Controllers** (45 min)
   - `joint_state_publisher` GUI
   - Publishing joint commands
   - Reading joint feedback

### Chapter 4: Package Development
**Goal**: Build, test, and deploy ROS 2 packages

1. **Workspace Setup** (30 min)
   - Colcon workspace structure
   - `package.xml` and `setup.py`
   - Building with colcon

2. **Launch Files** (45 min)
   - Python launch files
   - Node parameters and remapping
   - Launching multiple nodes

3. **Debugging** (30 min)
   - Common ROS 2 errors
   - Using `ros2 doctor`
   - Logging levels

4. **Assessment Project** (90 min)
   - Multi-node robot control package
   - Integration test
   - Documentation

## Key Code Examples

### Minimal Publisher

```python
# File: static/code/ros2-packages/minimal_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run**: `python3 minimal_publisher.py`

### Minimal Subscriber

```python
# File: static/code/ros2-packages/minimal_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String, 'chatter', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run**: `python3 minimal_subscriber.py` (in separate terminal)

### Simple Humanoid URDF

```xml
<!-- File: static/code/ros2-packages/simple_humanoid.urdf -->
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link (Torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Head Link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right Arm -->
  <joint name="right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="2.0"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Arm (mirror of right) -->
  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="2.0"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Leg -->
  <joint name="right_hip" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="-0.1 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="2.0"/>
  </joint>

  <link name="right_thigh">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Leg (mirror of right) -->
  <joint name="left_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="0.1 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="2.0"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>

</robot>
```

**Visualize**: `ros2 launch urdf_tutorial display.launch.py model:=simple_humanoid.urdf`

## Package Structure Template

```
my_robot_package/
├── package.xml              # Package metadata
├── setup.py                 # Python package setup
├── setup.cfg               # Install configuration
├── resource/               # Package marker
│   └── my_robot_package
├── my_robot_package/       # Python source code
│   ├── __init__.py
│   ├── publisher_node.py
│   ├── subscriber_node.py
│   └── service_node.py
├── launch/                 # Launch files
│   └── robot_system.launch.py
├── urdf/                   # Robot models
│   └── my_robot.urdf
└── config/                 # Configuration files
    └── params.yaml
```

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.1.0</version>
  <description>Physical AI Robot Control Package</description>
  <maintainer email="student@example.com">Student</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>urdf</depend>
  <depend>robot_state_publisher</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Physical AI Robot Control Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = my_robot_package.publisher_node:main',
            'subscriber_node = my_robot_package.subscriber_node:main',
            'service_node = my_robot_package.service_node:main',
        ],
    },
)
```

## Assessment Project

### Multi-Node Robot Control System

**Objective**: Build a simulated temperature monitoring system for a humanoid robot

**Requirements**:
1. **Temperature Sensor Node**: Publishes random temperature data (Float64) every 1 second
2. **Monitor Node**: Subscribes to temperature, logs warnings if > 80°C
3. **Control Service**: Provides a service to set temperature thresholds
4. **URDF Model**: 5-joint humanoid (torso, head, 2 arms, 2 legs)
5. **Launch File**: Starts all nodes and loads URDF into RViz

**Success Criteria**:
- ✅ All nodes communicate via topics/services
- ✅ URDF visualizes correctly in RViz
- ✅ Package builds with `colcon build`
- ✅ System runs with single launch file

## Next Steps

After completing Module 1:
1. Proceed to [Module 2: Digital Twin Simulation](../module-2-simulation/ch5-gazebo/gazebo-architecture.md)
2. Apply ROS 2 skills in Gazebo and Unity
3. Add perception with NVIDIA Isaac (Module 3)

## Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [rclpy API](https://docs.ros.org/en/humble/p/rclpy/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [Colcon Documentation](https://colcon.readthedocs.io/)

---

**Module Duration**: 3 weeks
**Total Lessons**: 16
**Hands-On Projects**: 4
**Assessment**: Multi-node robot control system
