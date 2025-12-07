---
---

# Getting Started with Unity and the Robotics Hub

While Gazebo is fantastic for physics simulation, Unity offers a world-class rendering engine, making it an excellent choice for high-fidelity visualization, creating synthetic data, and developing interactive robotic applications.

To bridge the gap between Unity's powerful creative engine and the ROS 2 ecosystem, Unity provides the **Unity Robotics Hub**. This is a collection of packages that enable you to import, control, and test your robot in a visually rich Unity environment.

## Key Packages in the Unity Robotics Hub

The hub consists of several key packages you will manage through the Unity Package Manager:

1.  **URDF Importer**: A crucial tool that allows you to import a URDF file and automatically convert it into a fully articulated Unity prefab. It correctly configures the hierarchy of links and the properties of joints.
2.  **ROS TCP Connector**: This package provides the low-level TCP/IP communication that connects Unity to the ROS 2 network. It allows Unity scripts to publish and subscribe to ROS 2 topics.
3.  **ROS Visualizations**: A set of pre-built components for visualizing common ROS 2 messages directly within the Unity scene (e.g., TF frames, point clouds, laser scans). This is great for debugging.

## Step 1: Installing Unity Hub and the Unity Editor

First, you need the Unity development environment.

1.  **Download Unity Hub**: Go to the [Unity Download page](https://unity.com/download) and download Unity Hub. The Hub is an application that manages your Unity Editor versions and projects.
2.  **Install the Unity Editor**:
    -   Open Unity Hub and go to the "Installs" tab.
    -   Click "Install Editor" and choose a recent **LTS (Long-Term Support)** version (e.g., 2022.3.x).
    -   During installation, make sure to add the **"Linux Build Support (IL2CPP)"** module, as you will likely be connecting to a ROS 2 instance running on Linux.

## Step 2: Creating a New Unity Project

1.  In Unity Hub, go to the "Projects" tab.
2.  Click "New project".
3.  Select the **3D** template.
4.  Give your project a name (e.g., `PhysicalAI-UnitySim`) and choose a location.
5.  Click "Create project". This will open the Unity Editor.

## Step 3: Installing the Robotics Packages

Now, we'll add the necessary robotics packages to your project using the Unity Package Manager.

1.  In the Unity Editor, go to **Window > Package Manager**.
2.  Click the **"+"** icon in the top-left corner and select **"Add package from git URL..."**.
3.  Add the following packages one by one using their Git URLs. It's recommended to add them in this order, as they have dependencies on each other.

    -   **ROS TCP Connector**:
        ```
        com.unity.robotics.ros-tcp-connector
        ```
        *Note: Unity now hosts this on their own registry, so you may be able to find it in the "Unity Registry" section of the Package Manager directly.* If adding by Git URL, you may need to use the full URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git`

    -   **URDF Importer**:
        `https://github.com/Unity-Technologies/URDF-Importer.git`

    -   **ROS Visualizations (Optional but Recommended)**:
        `https://github.com/Unity-Technologies/ros-visualizations.git`

The Package Manager will automatically handle the dependencies and import the assets into your project.

## Step 4: Configuring the ROS 2 Connection

For Unity to communicate with ROS 2, you need to configure the IP addresses for the TCP connection.

1.  In the Unity Editor, go to **Robotics > ROS Settings**. This menu will appear after the Robotics Hub packages are installed.

2.  A configuration window will pop up. Here are the key settings:
    -   **ROS IP Address**: The IP address of the machine running the ROS 2 master (your Ubuntu machine).
    -   **ROS Port**: The port for the TCP connection. The default is `10000`.
    -   **Override Unity IP Address**: The IP address of the machine running Unity. If you are running ROS 2 and Unity on different machines, you must set this manually. If they are on the same machine, `localhost` (or `127.0.0.1`) is fine.

    ![Unity ROS Settings](https://user-images.githubusercontent.com/25339203/110582424-5d51e700-812e-11eb-9895-6b5d53b5b63b.png)
    *(Image credit: Unity Technologies)*

3.  **Firewall Configuration**:
    -   The most common issue is a firewall blocking the connection.
    -   Ensure that the firewall on both the ROS 2 machine and the Unity machine allows traffic on the specified TCP port (e.g., port `10000`).
    -   On Ubuntu, you might run: `sudo ufw allow 10000/tcp`.
    -   On Windows, you may need to add an inbound rule to Windows Defender Firewall.

## Step 5: Setting up the ROS 2 Side

The ROS TCP Connector requires a corresponding server endpoint on the ROS 2 side to complete the connection. This server is provided by the `ros-tcp-endpoint` package.

1.  **Install the ROS 2 package**:
    On your ROS 2 machine, install the endpoint package.
    ```bash
    # For Humble
    sudo apt-get install ros-humble-ros-tcp-endpoint
    ```
    Or build it from source if you need the latest version:
    ```bash
    git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
    cd ROS-TCP-Endpoint
    colcon build --symlink-install
    source install/setup.bash
    ```

2.  **Run the endpoint server**:
    You need to run this server whenever you want to connect Unity to your ROS 2 network.
    ```bash
    # Pass your ROS_IP as an argument
    ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<your_ros_ip>
    ```
    Replace `<your_ros_ip>` with the actual IP address of your ROS 2 machine.

## Verifying the Connection

1.  Ensure the `ros_tcp_endpoint` is running on your ROS 2 machine.
2.  Configure the IP addresses in the Unity ROS Settings.
3.  Enter Play mode in the Unity Editor by clicking the "Play" button at the top.

Check the console in both Unity and your ROS 2 terminal.
-   The ROS 2 terminal should show a message like `[INFO] [default_server_endpoint]: Client connected.`
-   The Unity console will show a confirmation that the connection was successful.

If you see these messages, congratulations! Your Unity project is now successfully connected to your ROS 2 graph. You are ready to import your robot and start controlling it.
