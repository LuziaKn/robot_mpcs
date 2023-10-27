import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'robotmpcs_ros2'

    ld = LaunchDescription()

    # Calculate the absolute path to the YAML file
    config = os.path.join(
        get_package_share_directory('robotmpcs_ros2'),
        'config',
        'jackal_mpc_config.yaml'
        )

    node= Node(
        package=package_name,
        executable='mpc_planner_node',  # Replace with your node's executable name
        name='mpc_planner_node',
        output='screen',
        parameters=[config],
    )

    ld.add_action(node)
    return ld

