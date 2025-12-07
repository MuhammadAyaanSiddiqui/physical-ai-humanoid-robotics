---
---

# Attaching and Configuring Sensor Plugins in Gazebo

A robot is only as smart as its sensors. In Gazebo, you can add a wide variety of sensors to your robot models by using **sensor plugins**. These plugins generate realistic sensor data based on the simulation environment, which is then published to Gazebo topics and bridged to ROS 2 for your nodes to consume.

The most common sensors used in robotics are cameras, LiDAR, and IMUs. Let's cover how to add each of these to a URDF.

## The `<gazebo>` Tag: Your Bridge to SDF

As we've learned, URDFs are simpler than SDFs and don't natively support simulation-specific tags. To add Gazebo plugins and parameters to a URDF, we use a special `<gazebo>` tag. This tag is ignored by ROS 2 tools like `robot_state_publisher` but is parsed by Gazebo during the URDF-to-SDF conversion process.

A `<gazebo>` tag can be associated with a specific `<link>` or `<joint>`. When defining a sensor, you'll attach it to a link and give it a reference frame.

The general structure for a sensor plugin looks like this:

```xml
<gazebo reference="link_where_sensor_is_attached">
  <sensor name="my_sensor_name" type="sensor_type">
    <!-- Sensor-specific configuration -->
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
    
    <!-- Plugin configuration -->
    <plugin name="my_plugin_name" filename="libplugin_name.so">
      <!-- Plugin-specific parameters -->
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>~/out:=/sensor_topic</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

-   **`reference`**: The name of the `<link>` the sensor is rigidly attached to.
-   **`type`**: The type of sensor (e.g., `camera`, `gpu_lidar`, `imu`).
-   **`always_on`**, **`update_rate`**, **`visualize`**: Common sensor parameters controlling its behavior and visibility in the GUI.
-   **`plugin`**: Specifies the shared library (`.so` file) to load and its configuration. The `gz-sensors` project provides a rich set of these libraries.
-   **`<ros>` tag**: This is a custom block used by `ros_gz` plugins to configure ROS 2 specific settings like namespace and topic remapping.

## 1. Camera Sensor Plugin

A camera plugin simulates a standard camera, generating `sensor_msgs/Image` messages.

### URDF Example:

This snippet attaches a camera to a `camera_link` on the robot.

```xml
<link name="camera_link" />
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor name="camera_sensor" type="camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_plugin" filename="libgz-sensors-camera.so">
      <ros>
        <namespace>demo_cam</namespace>
        <!-- <remapping>~/out:=image_raw</remapping> -->
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

**Key Parameters**:
-   **`type="camera"`**: Specifies the camera sensor.
-   **`update_rate`**: The frame rate of the camera (in Hz).
-   **`<horizontal_fov>`**: The camera's horizontal field of view in radians.
-   **`<image>`**: Defines the resolution and pixel format.
-   **`<clip>`**: The near and far clipping planes. Objects closer than `near` or farther than `far` will not be rendered.
-   **`filename="libgz-sensors-camera.so"`**: The shared library for the camera sensor system.
-   **`<ros>` block**:
    - The `ros_gz_bridge` will automatically create a bridge for this sensor.
    - By default, the image topic will be `/demo_cam/camera_sensor`. You can use remapping to change it.

**To see the output in ROS 2:**
```bash
# Bridge the topic if not automatically done
ros2 run ros_gz_image image_bridge /demo_cam/camera_sensor

# View the image
ros2 run image_view image_view --ros-args -r image:=/demo_cam/camera_sensor
```

## 2. LiDAR Sensor Plugin

A LiDAR plugin simulates a laser scanner, generating `sensor_msgs/LaserScan` (for 2D) or `sensor_msgs/PointCloud2` (for 3D) messages. The `gpu_lidar` is highly recommended for performance.

### URDF Example:

This attaches a 3D LiDAR to a `lidar_link`.

```xml
<link name="lidar_link" />
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
</joint>

<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="gpu_lidar">
    <update_rate>10</update_rate>
    <visualize>true</visualize>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>
        <max_angle>0.261799</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
    <plugin name="lidar_plugin" filename="libgz-sensors-gpu-lidar.so">
      <ros>
        <namespace>demo_lidar</namespace>
        <remapping>~/out:=points</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

**Key Parameters**:
-   **`type="gpu_lidar"`**: Uses the GPU for faster point cloud generation.
-   **`<scan>`**: Defines the scanner's properties.
    -   **`<horizontal>`**: Settings for the horizontal sweep (number of samples, resolution, and angle range).
    -   **`<vertical>`**: Settings for the vertical sweep (for 3D LiDAR). For a 2D LiDAR, you would set `<samples>` to 1.
-   **`<range>`**: The minimum and maximum detection distances.
-   **`<noise>`**: Adds noise to the measurements to make the simulation more realistic.
-   **`filename="libgz-sensors-gpu-lidar.so"`**: The shared library for the GPU LiDAR system.
-   **`<remapping>~/out:=points</remapping>`**: The default output topic is `~/out`, which resolves to `/demo_lidar/lidar_sensor`. This remaps it to `/demo_lidar/points`.

**To see the output in ROS 2:**
You can visualize the point cloud in RViz2 by adding a `PointCloud2` display and subscribing to the `/demo_lidar/points` topic.

## 3. IMU Sensor Plugin

An Inertial Measurement Unit (IMU) plugin simulates an accelerometer and a gyroscope, providing data on linear acceleration and angular velocity. This is crucial for robot localization and balance.

### URDF Example:

```xml
<link name="imu_link" />
<joint name="imu_joint" type="fixed">
    <parent link="base_link" />
    <child link="imu_link" />
    <origin xyz="0 0 0.1" rpy="0 0 0" />
</joint>

<gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <plugin name="imu_plugin" filename="libgz-sensors-imu.so">
        <ros>
          <namespace>demo_imu</namespace>
          <remapping>~/out:=data</remapping>
        </ros>
      </plugin>
    </sensor>
</gazebo>
```

**Key Parameters**:
-   **`type="imu"`**: Specifies the IMU sensor.
-   **`update_rate`**: A high update rate (100 Hz or more) is common for IMUs.
-   **`filename="libgz-sensors-imu.so"`**: The shared library for the IMU system.
- The plugin can also be configured with noise parameters for the accelerometer and gyroscope to simulate real-world sensor drift and error.

**To see the output in ROS 2:**
```bash
ros2 topic echo /demo_imu/data
```
This will print `sensor_msgs/Imu` messages containing orientation, angular velocity, and linear acceleration.

## Putting It All Together

To create a well-equipped robot, you will combine these sensor definitions into your main robot URDF file, attaching each sensor to the appropriate link. Remember to create the physical links and fixed joints that place the sensors correctly on your robot's body.

By mastering sensor plugins, you can create rich, realistic data streams that will power your perception, navigation, and control algorithms, just as they would on a physical robot.
