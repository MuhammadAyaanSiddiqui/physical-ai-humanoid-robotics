---
---

# ROS 2 Integration: The `ros_gz_bridge`

You have a simulation running in Gazebo and a set of ROS 2 nodes ready to control a robot. How do you connect them? The answer is the `ros_gz_bridge`, a powerful and essential tool for communication between the two systems.

Gazebo and ROS 2 are separate frameworks with their own transport layers.
- **Gazebo** uses its own publish-subscribe system, **Gazebo Transport**.
- **ROS 2** uses **DDS (Data Distribution Service)** as its middleware.

The `ros_gz_bridge` acts as a bi-directional translator, converting messages from one system to the other.

## How the Bridge Works

The bridge is a standalone ROS 2 package that you run as a node. It allows you to specify which topics you want to bridge and in which direction.

```mermaid
graph TD
    subgraph "Gazebo Environment"
        A[Gazebo Sensor Plugin<br>(e.g., Camera)]
        B[Gazebo Topic<br>/camera]
        A -->|Publishes Gazebo Msg| B
    end

    subgraph "ROS 2 Environment"
        D[ROS 2 Node<br>(e.g., Image Processor)]
        E[ROS 2 Topic<br>/camera_ros]
        D -->|Subscribes to ROS 2 Msg| E
    end

    subgraph "ros_gz_bridge"
        C[Bridge Node]
    end

    B -->|Gazebo Msg| C
    C -->|Translates to ROS 2 Msg| E

    style C fill:#ccf,stroke:#333,stroke-width:2px
```

The bridge can translate messages in three ways:
1.  **Gazebo to ROS 2**: For sensor data. A camera plugin in Gazebo publishes an `gz.msgs.Image`, and the bridge translates it to a `sensor_msgs/msg/Image` for your ROS 2 nodes.
2.  **ROS 2 to Gazebo**: For control commands. A ROS 2 node publishes a `geometry_msgs/msg/Twist`, and the bridge translates it to an `gz.msgs.Twist` to control a robot's velocity in Gazebo.
3.  **Bi-directional**: For state synchronization, like clock or TF messages.

## Running the Bridge

There are two main ways to run the bridge: from the command line or from a launch file.

### Method 1: Command Line

You can run the bridge as an executable, providing the topics and message types as arguments. The syntax is:

```bash
ros2 run ros_gz_bridge parameter_bridge <gazebo_topic>@<gazebo_msg_type>[<ros2_topic>@<ros2_msg_type>
```

The direction is specified by the brackets:
-   `[`: Gazebo to ROS 2 (subscribes to Gazebo, publishes to ROS 2)
-   `]`: ROS 2 to Gazebo (subscribes to ROS 2, publishes to Gazebo)
-   `@`: Separates the topic name from the message type.

**Example: Bridging a Camera (Gazebo to ROS 2)**

If a Gazebo camera is publishing to the `/camera` topic with message type `gz.msgs.Image`, you can bridge it to a ROS 2 topic named `/camera_ros` with message type `sensor_msgs/msg/Image`.

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /camera@gz.msgs.Image[sensor_msgs/msg/Image@/camera_ros
```

**Example: Bridging Velocity Commands (ROS 2 to Gazebo)**

To send velocity commands from ROS 2 to a Gazebo differential drive plugin:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist@/model/my_robot/cmd_vel
```
*Note: The Gazebo topic for model control is often namespaced under `/model/<model_name>/`.*

### Method 2: Launch File (Recommended)

For any real project, you'll want to include the bridge in your launch file. This ensures all communication channels are set up automatically when you start your simulation.

You can create a `Node` that executes the `parameter_bridge`.

**Example: Launch file snippet for bridging camera and commands**

```python
from launch_ros.actions import Node

# ... inside your generate_launch_description function

bridge_node = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        # Bridge Camera Data (Gazebo -> ROS 2)
        '/camera@gz.msgs.Image[sensor_msgs/msg/Image@/camera_ros',
        
        # Bridge Velocity Commands (ROS 2 -> Gazebo)
        '/model/my_robot/cmd_vel@gz.msgs.Twist]geometry_msgs/msg/Twist@/cmd_vel'
    ],
    output='screen'
)

# Add bridge_node to your LaunchDescription
return LaunchDescription([
    # ... other nodes
    bridge_node
])
```

## Built-in Bridges in Sensor Plugins

As we saw in the previous lesson, modern `ros_gz` sensor plugins often handle bridging automatically. When you include the `<ros>` block in your plugin definition, you are configuring a built-in bridge.

```xml
<plugin name="camera_plugin" filename="libgz-sensors-camera.so">
  <ros>
    <namespace>my_camera</namespace>
    <remapping>~/out:=image_raw</remapping>
  </ros>
</plugin>
```

This is effectively doing the work of the `parameter_bridge` for you. The sensor plugin itself starts a bridge for its output topic.

**When to use a standalone bridge vs. a built-in one?**
-   **Use built-in bridges (`<ros>` tag)** whenever possible. It's cleaner and couples the sensor directly with its ROS 2 interface. This is the standard for cameras, LiDAR, and IMUs.
-   **Use a standalone `parameter_bridge`** when:
    -   You need to bridge topics not associated with a specific sensor (e.g., clock, world state).
    -   You need to send control commands from ROS 2 to a Gazebo plugin (like a differential drive or gripper controller).
    -   You are working with older plugins that don't have the `<ros>` block integration.

## Common Topics to Bridge

Here is a list of common topics and their message types that you will frequently bridge:

| Purpose | Gazebo Topic | Gazebo Type | ROS 2 Type | Direction |
| :--- | :--- | :--- | :--- | :--- |
| **Clock** | `/clock` | `gz.msgs.Clock` | `rosgraph_msgs/msg/Clock` | Gz -> ROS |
| **Transforms**| `/tf` | `gz.msgs.Pose_V` | `tf2_msgs/msg/TFMessage` | Gz -> ROS |
| **Camera** | `/camera` | `gz.msgs.Image` | `sensor_msgs/msg/Image` | Gz -> ROS |
| **LiDAR (3D)** | `/lidar` | `gz.msgs.PointCloudPacked`|`sensor_msgs/msg/PointCloud2`| Gz -> ROS |
| **IMU** | `/imu` | `gz.msgs.IMU` | `sensor_msgs/msg/Imu` | Gz -> ROS |
| **Velocity**|`/model/X/cmd_vel`|`gz.msgs.Twist`|`geometry_msgs/msg/Twist`| ROS -> Gz |
| **Joint State**| `/world/X/model/Y/joint_state` |`gz.msgs.Model`| `sensor_msgs/msg/JointState`| Gz -> ROS|

## Debugging the Bridge

1.  **Check Topics**: Use `gz topic -l` to see all available Gazebo topics and `ros2 topic list` to see all ROS 2 topics. Confirm that the bridge is subscribing and publishing as expected.
2.  **Check Message Types**: Make sure you are using compatible message types. The `ros_gz` documentation provides a table of all supported message pairings.
3.  **Use Verbose Output**: Run the bridge with `-v` to get more detailed logging about which topics are being bridged and whether the mappings are successful.
4.  **Isolate the Problem**: Test each side independently. Can you `gz topic -e -t /camera` to see data in Gazebo? Can you `ros2 topic pub` to your command topic? This will tell you if the problem is with the bridge or with the publisher/subscriber.

By mastering the `ros_gz_bridge`, you unlock the full power of ROS 2 for controlling and sensing in a high-fidelity Gazebo simulation.
