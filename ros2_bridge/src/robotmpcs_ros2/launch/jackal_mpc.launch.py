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
    current_path = os.path.dirname(os.path.abspath(__file__))
    package_path = os.path.join(
        current_path,
        "..")
    config = os.path.join(
        current_path,
        "..",
        'config',
        'jackal_mpc_config.yaml'
        )

    node= Node(
        package=package_name,
        executable='mpc_planner_node',  # Replace with your node's executable name
        name='mpc_planner_node',
        output='screen',
        parameters=[config, {"package_path": package_path}],
    )

    ld.add_action(node)
    return ld

