---
---

# Module 2 Assessment: Simulation Project

## Objective

The goal of this assessment is to apply the concepts you've learned in Module 2 to create a complete simulation setup for a robot of your choice. You will be evaluated on your ability to create a stable Gazebo simulation, add sensors to your robot, bridge the data to ROS 2, and document your process.

This project will test your skills in:
-   Creating and modifying URDF and SDF files.
-   Configuring Gazebo worlds and physics.
-   Adding and configuring sensor plugins (Camera, LiDAR, IMU).
-   Using the `ros_gz_bridge` to establish communication.
-   Creating a ROS 2 launch file to bring up the entire simulation stack.

## Project Requirements

You must create a ROS 2 package for a simple robot that can be fully simulated in Gazebo.

### 1. Robot Model (URDF)

-   Create a new URDF file for your robot. The robot must have:
    -   At least **four links** and **three joints**.
    -   A combination of `revolute` and `fixed` joints.
    -   Reasonable `<inertial>` properties for all links.
    -   `<collision>` and `<visual>` tags for all links.
-   The robot must have the following sensors attached:
    -   A **camera** sensor.
    -   A **2D or 3D LiDAR** sensor.
    -   An **IMU** sensor.
-   Each sensor must be configured with a `ros_gz` plugin to publish data.

### 2. Gazebo World

-   Create a new Gazebo world file (`.sdf`).
-   The world must contain:
    -   A ground plane and a light source.
    -   At least **three static models** (e.g., boxes, cylinders, spheres) arranged as obstacles.
    -   Custom gravity settings (i.e., not Earth gravity, `0 0 -9.81`).

### 3. ROS 2 Integration

-   Create a single ROS 2 **launch file** that:
    1.  Launches the Gazebo simulation with your custom world.
    2.  Spawns your robot model from the URDF file.
    3.  Starts a `robot_state_publisher` to publish the robot's TFs.
    4.  Starts a `ros_gz_bridge` to bridge the sensor data from Gazebo to ROS 2.
        -   The camera image should be available on a ROS 2 topic.
        -   The LiDAR data should be available on a ROS 2 topic.
        -   The IMU data should be available on a ROS 2 topic.

### 4. Documentation

-   Create a `README.md` file in the root of your package that includes:
    -   A brief description of your robot.
    -   Instructions on how to build and launch the simulation.
    -   A list of the ROS 2 topics that the bridged sensor data is published to.
    -   A screenshot of your robot in the Gazebo world.

## Submission

-   Push your completed ROS 2 package to a public GitHub repository.
-   Submit a link to the repository.
-   Ensure your `README.md` is clear and that your launch file works as expected in a standard ROS 2 Humble + Gazebo Fortress environment.

## Grading Rubric (100 Points)

| Criteria | Unsatisfactory (0-9 pts) | Satisfactory (10-17 pts) | Excellent (18-25 pts) | Score |
| :--- | :--- | :--- | :--- | :--- |
| **Robot URDF** (25) | Robot model is missing, has significant errors, or does not meet link/joint requirements. | Robot model exists but is missing some components (inertial, collision) or has minor errors. | Robot model is well-formed, meets all requirements, and has realistic inertial properties. All sensors are correctly attached and configured. | |
| **Gazebo World** (25) | World file is missing or does not meet requirements for static objects and physics. | World file is functional but is missing some required elements (e.g., custom gravity, enough obstacles). | World file is well-designed, contains all required elements, and demonstrates an understanding of SDF properties. | |
| **ROS 2 Launch File** (25)| Launch file is missing, fails to run, or does not launch all required nodes. | Launch file runs but is missing one or more required components (e.g., bridge, robot_state_publisher), or bridging is incomplete. | Launch file correctly and reliably launches the entire simulation stack. All sensor topics are correctly bridged and available in ROS 2. | |
| **Documentation** (25) | `README.md` is missing or lacks critical information. | `README.md` exists but is incomplete. Instructions may be unclear or missing steps. | `README.md` is clear, comprehensive, and provides all necessary information to build, launch, and test the simulation. Includes a screenshot. | |
| **Total** | | | | **/ 100** |

### Example of an "Excellent" Project

-   A simple four-wheeled rover with a "head" link on a revolute joint.
-   A camera is attached to the head, a LiDAR is on the main body, and an IMU is at the center.
-   The Gazebo world is a "Martian landscape" with red-tinted lighting, low gravity (`0 0 -3.7`), and several "rock" obstacles.
-   A single `ros2 launch` command brings up Gazebo, spawns the rover, and makes `/camera/image_raw`, `/lidar/points`, and `/imu/data` available in ROS 2.
-   The `README.md` clearly explains the robot and the simulation, and a new user can run it without issues.

Good luck! This assessment is your chance to bring together everything from this module into a cohesive project.
