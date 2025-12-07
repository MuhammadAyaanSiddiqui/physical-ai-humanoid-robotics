---
---

# Spawning Robots in Gazebo

Now that you understand the architecture of a Gazebo world, it's time for the exciting part: adding a robot to it! This process is called "spawning." You can spawn a robot from a URDF or SDF file using several methods, including ROS 2 launch files, command-line tools, and services.

## From URDF to SDF: A Necessary Conversion

A key point to understand is that **Gazebo natively uses the Simulation Description Format (SDF)**. While ROS heavily relies on the Unified Robot Description Format (URDF) for robot modeling, Gazebo's physics engine requires SDF for full functionality.

When you load a URDF file into Gazebo, it's **automatically converted to SDF** behind the scenes. However, this conversion has limitations:
- URDFs cannot specify many of the rich simulation parameters that SDFs can (e.g., friction coefficients, damping, sensor plugins).
- To add Gazebo-specific features, you must add special `<gazebo>` tags to your URDF file, which are then parsed during the conversion.

For this reason, you will often see both URDF and SDF files used in ROS 2 projects.
- **URDF**: Great for general robot structure, kinematics, and visualization in RViz.
- **SDF**: Best for high-fidelity simulation in Gazebo.

## Method 1: Spawning with a ROS 2 Launch File (Recommended)

The most common and robust method for spawning a robot is using a ROS 2 launch file. This approach automates the entire process, making it repeatable and easy to manage. The key is the `spawn_entity.py` script provided by the `ros_gz_sim` package.

Here’s a typical workflow within a launch file:

1.  **Get the path to your robot's URDF/SDF file.**
2.  **Launch the Gazebo simulation** with your desired world file.
3.  **Launch a `robot_state_publisher` node.** This node reads your URDF, finds the joint states, and publishes the robot's `tf2` transforms. This is essential for visualizing the robot in RViz and for many other ROS 2 nodes.
4.  **Launch the `spawn_entity.py` node.** This node takes your robot description file and calls the Gazebo service `/spawn_entity` to add it to the simulation.

### Example: `humanoid_spawn_launch.py`

Let's break down a launch file that spawns a humanoid robot from a URDF file.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Get the path to the URDF file
    urdf_file_name = 'simple_humanoid.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('your_robot_package'),
        'urdf',
        urdf_file_name)

    # 2. Read the URDF file content
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 3. Launch Gazebo
    # We use 'gz sim' which is the new command for Gazebo Fortress/Garden
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.world'],
        output='screen'
    )

    # 4. Robot State Publisher
    # This node publishes the robot's state to TF2
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}]
    )

    # 5. Spawn Entity Node
    # This node spawns the robot model in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-name', 'humanoid',
            '-allow_renaming', 'true'
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

**To run this launch file:**
```bash
ros2 launch your_robot_package humanoid_spawn_launch.py
```

## Method 2: Spawning from the Command Line

You can also spawn a robot manually using the `ros2 service call` command. This is useful for debugging or for adding objects to an already-running simulation.

The service we need to call is `/spawn_entity`.

**Steps:**

1.  **Start Gazebo:**
    ```bash
    gz sim -r empty.world
    ```

2.  **Get your robot's XML content:**
    You can either pass the file path directly or read the file content into a string.

3.  **Call the service:**

    ```bash
    # Make sure your URDF/SDF file exists at this path
    ros2 service call /spawn_entity ros_gz_interfaces/srv/SpawnEntity "{
        name: 'my_robot',
        xml: '$(cat /path/to/your/robot.urdf)',
        robot_namespace: 'my_robot',
        initial_pose: {
            position: {x: 0, y: 0, z: 1.0},
            orientation: {x: 0, y: 0, z: 0, w: 1}
        }
    }"
    ```

**Key Parameters for the `SpawnEntity` service:**
- `name`: A unique name for your robot in the simulation.
- `xml`: The full XML content of your URDF or SDF file as a string.
- `robot_namespace`: The ROS 2 namespace for the robot's topics and services.
- `initial_pose`: Where to spawn the robot in the world.

## Method 3: Spawning from a Python Script

You can achieve the same result as the command-line method from within a Python script by creating a ROS 2 service client. This gives you programmatic control over when and how entities are spawned.

### Example: `spawn_script.py`

```python
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity

class Spawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /spawn_entity not available, waiting...')
        
        # Path to your robot description file
        urdf_path = '/path/to/your/robot.urdf'
        with open(urdf_path, 'r') as f:
            self.robot_xml = f.read()

    def send_request(self):
        req = SpawnEntity.Request()
        req.name = 'script_spawned_robot'
        req.xml = self.robot_xml
        req.initial_pose.position.z = 1.5 # Spawn slightly higher

        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        
        if self.future.result() is not None:
            self.get_logger().info(f'Spawn successful: {self.future.result().success}')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    spawner = Spawner()
    spawner.send_request()
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting Common Spawning Issues

1.  **"Service /spawn_entity not available"**:
    - **Cause**: Gazebo isn't running, or the `ros_gz_bridge` hasn't been started correctly.
    - **Solution**: Ensure `gz sim` is running and that the `ros_gz_sim` package is installed and sourced.

2.  **Robot falls through the ground plane**:
    - **Cause**: Missing `<collision>` tags in your URDF/SDF, or the collision geometry is incorrect.
    - **Solution**: Verify that every `<link>` has a corresponding `<collision>` tag. Check that the collision shapes are valid.

3.  **Robot appears "exploded" or parts are in the wrong place**:
    - **Cause**: Incorrect joint origins or axes in your URDF.
    - **Solution**: Use `rviz2` and a `joint_state_publisher` to debug your URDF separately from Gazebo. Ensure your joint kinematics are correct.

4.  **"Could not find package..."**:
    - **Cause**: Your ROS 2 workspace isn't sourced, or the package containing your robot description isn't installed.
    - **Solution**: Run `source install/setup.bash` in your workspace and make sure you've built your robot's package with `colcon build`.

With these methods, you have a complete toolkit for getting your robots into the simulation. The launch file method is preferred for its robustness, but the others are excellent for debugging and dynamic control.
