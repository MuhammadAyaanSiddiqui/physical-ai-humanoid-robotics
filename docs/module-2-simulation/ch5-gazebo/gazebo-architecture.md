---
sidebar_position: 1
---

# Gazebo Architecture: Worlds, Models, and Plugins

Welcome to the world of simulation! Before we start spawning robots, it's crucial to understand the fundamental building blocks of Gazebo. Gazebo simulates a 3D environment by composing three key elements: **Worlds**, **Models**, and **Plugins**.

Think of it like building a movie set:
- The **World** is the entire soundstage, including the lighting, the ground, and the overall physics.
- **Models** are the actors, props, and scenery you place within the stage.
- **Plugins** are the special effects and scripts that make the actors and props behave in specific ways.

Let's break down each component.

## 1. The World (`.sdf` or `.world`)

A Gazebo World file defines the entire simulation environment. It's an XML file written in the **Simulation Description Format (SDF)**. This file specifies everything needed to set up the scene, including:

- **Scene Properties**: Ambient light, background color, and shadows.
- **Physics Engine**: Configuration for gravity, friction, and the physics engine (like ODE or Bullet).
- **Static Objects**: Ground planes, buildings, tables, and other non-moving scenery.
- **Included Models**: References to other SDF files for robots, objects, and sensors.
- **Plugins**: World-level plugins that might affect the entire simulation (e.g., a wind effect).

### Example: A Simple World File

Here’s what a basic `empty_world.world` file might look like:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <!-- 1. Scene Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- 2. Static Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- 3. Physics Engine Configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- 4. Graphical User Interface (GUI) settings -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-6.0 -6.0 6.0 0 0.5 0.7</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>
```

**Key Takeaways**:
- The `<world>` tag is the root element.
- We `<include>` pre-defined models like `sun` and `ground_plane` from Gazebo's model database.
- The `<physics>` tag sets the simulation's "laws of nature."
- The `<gui>` tag configures the default camera view when you open the simulation.

## 2. Models

Models are the objects within your world. A model can be anything from a simple shape (like a box or a sphere) to a complex, multi-jointed humanoid robot. Each model is also defined using an SDF file (`.sdf`) or can be converted from a URDF file.

A model is composed of three primary components:
- **Links**: The physical parts of the model (e.g., the torso, arm, or leg of a robot). Each link has physical properties like mass, inertia, and a collision shape.
- **Joints**: The connections between links. Joints define how links can move relative to each other (e.g., `revolute` for a rotating elbow joint or `prismatic` for a sliding joint).
- **Plugins**: Model-specific plugins that give the model its behavior (e.g., a sensor plugin for a camera or a controller plugin for a motor).

### Example: A Simple Box Model

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_box">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="box_link">
      <!-- Physical Properties -->
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.083</ixx>
          <iyy>0.083</iyy>
          <izz>0.083</izz>
        </inertia>
      </inertial>

      <!-- Visual Appearance -->
      <visual name="visual">
        <geometry><box><size>1 1 1</size></box></geometry>
        <material><ambient>0 0 1 1</ambient></material>
      </visual>

      <!-- Collision Shape -->
      <collision name="collision">
        <geometry><box><size>1 1 1</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
```

**Key Takeaways**:
- The `<model>` tag is the root element for a model file.
- A model must have at least one `<link>`.
- **`<inertial>`**: Defines the model's mass and resistance to rotation. Critical for realistic physics.
- **`<visual>`**: What the model looks like. This is what you see in the simulation.
- **`<collision>`**: The shape used by the physics engine to calculate collisions. This is often simpler than the visual geometry to save computation.

## 3. Plugins

Plugins are the brains of a Gazebo simulation. They are shared libraries, typically written in C++, that can be attached to Worlds, Models, or even individual sensors to add custom behavior.

Without plugins, your simulation would be a static collection of objects that just fall to the ground. Plugins allow you to:
- **Control Robots**: Implement motor controllers that respond to ROS 2 messages.
- **Simulate Sensors**: Generate realistic sensor data (e.g., LiDAR point clouds, camera images, IMU readings).
- **Create Complex Scenarios**: Animate objects, apply external forces (like wind), or change lighting conditions.
- **Integrate with ROS 2**: The `ros_gz_bridge` is a specialized set of plugins that seamlessly passes messages between Gazebo and ROS 2.

### How Plugins Work

Gazebo loads plugins at runtime and executes their code. A plugin can listen to Gazebo's internal events (e.g., the physics update loop) and can interact with the simulation by:
- Applying forces or torques to links.
- Setting joint velocities or positions.
- Publishing data to Gazebo topics (which can then be bridged to ROS 2).

For example, a **Camera Sensor Plugin** would:
1. Attach to a camera sensor on a robot model.
2. During each simulation update, render an image from the camera's perspective.
3. Publish that image to a Gazebo topic.
4. The `ros_gz_bridge` would then relay this image to a corresponding ROS 2 topic, making it available to your perception nodes.

## Summary Diagram

Here’s how it all fits together:

```mermaid
graph TD
    A[World (.sdf)] --> B(Scene: Lighting, Physics)
    A --> C{Include Models}
    
    subgraph "Model: my_robot.sdf"
        D[Links: Physical Bodies]
        E[Joints: Connections]
        F[Plugins: Behavior]
    end

    C --> D
    D --> E
    D --> F

    subgraph "Plugins"
        G[World Plugins: e.g., Wind]
        H[Model Plugins: e.g., Motor Controller]
        I[Sensor Plugins: e.g., Camera, LiDAR]
    end

    A --> G
    F --> H
    F --> I

    subgraph "ROS 2 Integration"
        J[ros_gz_bridge]
    end

    I --> J
    H --> J

    J -- ROS 2 Topics --> K[Your ROS 2 Nodes]
    K -- ROS 2 Topics --> J

    style A fill:#f9f,stroke:#333,stroke-width:2px
    style C fill:#ccf,stroke:#333,stroke-width:2px
    style J fill:#cfc,stroke:#333,stroke-width:2px
```

With this foundational understanding, you're now ready to move on to the next step: spawning your first robot into a Gazebo world!
