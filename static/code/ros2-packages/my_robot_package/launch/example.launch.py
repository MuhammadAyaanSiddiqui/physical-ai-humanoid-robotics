"""
Example launch file for my_robot_package

Demonstrates:
- Launching nodes
- Setting parameters
- Launch arguments
- Conditional launching
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""

    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='2.0',
        description='Rate at which to publish messages (Hz)'
    )

    message_prefix_arg = DeclareLaunchArgument(
        'message_prefix',
        default_value='Hello ROS 2',
        description='Prefix for published messages'
    )

    use_example_node_arg = DeclareLaunchArgument(
        'use_example_node',
        default_value='true',
        description='Whether to launch example node'
    )

    # Node configuration
    example_node = Node(
        package='my_robot_package',
        executable='example_node',
        name='example_node',
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'message_prefix': LaunchConfiguration('message_prefix'),
        }],
        condition=IfCondition(LaunchConfiguration('use_example_node'))
    )

    return LaunchDescription([
        # Arguments
        publish_rate_arg,
        message_prefix_arg,
        use_example_node_arg,
        # Nodes
        example_node,
    ])
