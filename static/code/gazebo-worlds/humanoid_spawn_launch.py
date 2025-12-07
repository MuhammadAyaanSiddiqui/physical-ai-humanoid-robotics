import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- Paths ---
    # Path to the Gazebo world file
    world_path = os.path.join(
        get_package_share_directory('your_robot_package'), # Replace with your package name
        'worlds', 
        'empty_world.world'
    )
    
    # Path to the robot's URDF file
    urdf_path = os.path.join(
        get_package_share_directory('your_robot_package'), # Replace with your package name
        'urdf', 
        'simple_humanoid.urdf'
    )

    # --- Nodes and Processes ---
    # 1. Launch Gazebo simulation
    # Using 'gz sim' for Gazebo Fortress/Garden+
    # '-r' flag runs the simulation upon startup
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # 2. Read the URDF file to a string
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # 3. Robot State Publisher
    # This node reads the URDF and publishes the robot's transformations (TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True # IMPORTANT: Use simulation time
        }]
    )

    # 4. Spawn Entity Node
    # This node spawns the robot model into the Gazebo simulation
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description,
            '-name', 'humanoid',
            '-allow_renaming', 'true',
            '-z', '1.0' # Spawn the robot 1 meter high
        ],
    )
    
    # 5. ROS-Gazebo Bridge
    # This bridges topics between ROS 2 and Gazebo
    # We bridge the clock and joint states
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS 2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint States (Gazebo -> ROS 2)
            '/world/default/model/humanoid/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/default/model/humanoid/joint_state', 'joint_states'),
        ],
        output='screen'
    )

    # 6. Joint State Publisher GUI
    # This provides a GUI to manually control the robot's joints
    # Helpful for testing and debugging the model
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )


    # --- Launch Description ---
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        # joint_state_publisher_gui, # Uncomment to use the GUI for joint control
    ])

# To Run This Launch File:
# 1. Make sure you have a ROS 2 package named 'your_robot_package'
# 2. Inside the package, you should have:
#    - worlds/empty_world.world
#    - urdf/simple_humanoid.urdf
#    - launch/humanoid_spawn_launch.py (this file)
# 3. Build your workspace: colcon build --packages-select your_robot_package
# 4. Source your workspace: source install/setup.bash
# 5. Run the launch file: ros2 launch your_robot_package humanoid_spawn_launch.py
