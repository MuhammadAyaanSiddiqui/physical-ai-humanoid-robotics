---
sidebar_position: 6
---

# Mermaid Diagram: Gazebo to ROS 2 Data Flow

This diagram illustrates the data flow from a Gazebo simulation to the ROS 2 ecosystem, facilitated by the `ros_gz_bridge`.

```mermaid
graph TD
    subgraph "Gazebo Simulation Environment"
        A[Gazebo World] --> B{Robot Model};
        B --> C[Sensor Plugin: Camera];
        B --> D[Sensor Plugin: LiDAR];
        B --> E[Actuator Plugin: Motor];

        C --"Raw Image Data"--> F[Gz Topic: /camera/image];
        D --"Raw Point Cloud"--> G[Gz Topic: /lidar/points];
        E <-- "Velocity Command" -- H[Gz Topic: /cmd_vel];
    end

    subgraph "ROS 2 Ecosystem"
        K[ROS 2 Node: Perception] -- "Subscribes" --> L[ROS 2 Topic: /camera/image_raw];
        M[ROS 2 Node: SLAM] -- "Subscribes" --> N[ROS 2 Topic: /lidar/points];
        O[ROS 2 Node: Teleop/Nav] -- "Publishes" --> P[ROS 2 Topic: /cmd_vel];
    end

    subgraph "ros_gz_bridge"
        I[Bridge Node]
    end

    F --"gz.msgs.Image"--> I;
    G --"gz.msgs.PointCloudPacked"--> I;
    I --"sensor_msgs/msg/Image"--> L;
    I --"sensor_msgs/msg/PointCloud2"--> N;
    
    P --"geometry_msgs/msg/Twist"--> I;
    I --"gz.msgs.Twist"--> H;

    style I fill:#ccf,stroke:#333,stroke-width:2px
```

### Explanation

1.  **Gazebo Simulation**:
    -   The Gazebo world contains a robot model.
    -   This model has sensor plugins (Camera, LiDAR) and actuator plugins (Motors).
    -   Sensor plugins publish their raw data to **Gazebo topics** (e.g., `/camera/image`, `/lidar/points`).
    -   Actuator plugins subscribe to Gazebo topics to receive control commands (e.g., `/cmd_vel`).

2.  **`ros_gz_bridge`**:
    -   The bridge node is the intermediary.
    -   It subscribes to specific Gazebo topics (like `/camera/image`).
    -   It translates the Gazebo message type (e.g., `gz.msgs.Image`) into the corresponding ROS 2 message type (e.g., `sensor_msgs/msg/Image`).
    -   It then publishes the translated message to a **ROS 2 topic** (e.g., `/camera/image_raw`).
    -   The process also works in reverse for control commands, translating ROS 2 messages into Gazebo messages.

3.  **ROS 2 Ecosystem**:
    -   Your ROS 2 nodes (Perception, SLAM, Navigation, etc.) operate purely in the ROS 2 domain.
    -   They subscribe to the ROS 2 topics published by the bridge to get sensor data.
    -   They publish to ROS 2 topics to send commands, which the bridge then forwards to Gazebo.

This decoupling is powerful. It means your ROS 2 nodes don't need to know (or care) that the data is coming from a simulation. The same perception code can work on a physical robot just by having its drivers publish to the same ROS 2 topics.
