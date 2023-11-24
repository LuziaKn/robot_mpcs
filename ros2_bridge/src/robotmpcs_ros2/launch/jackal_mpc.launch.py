import os
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    package_name = 'robotmpcs_ros2'

    ld = LaunchDescription()


    # Calculate the absolute path to the YAML file
    current_path = os.path.dirname(os.path.abspath(__file__))
    package_path = os.path.join(
        current_path,
        "..")
    ws_path = os.path.join(
        package_path,
        "..", "..", "..")
    config = os.path.join(
        package_path,
        'config',
        'jackal_mpc_config.yaml'
        )
    
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    harmony_nmcl = LaunchConfiguration('harmony_nmcl')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    param_substitutions = {
         'use_sim_time': use_sim_time,
         'yaml_filename': map_yaml_file}
    
    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace))
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    declare_harmony_nmcl_cmd = DeclareLaunchArgument(
        'harmony_nmcl', default_value='True', description='Whether run the H2020 harmony nmcl algorithm')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
         'map',
         default_value=os.path.join(package_path, 'maps', 'GMap.yaml'),
         description='Full path to map yaml file to load')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_path, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    


    
    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(current_path,
                                                       'localization.launch.py')),
            condition=IfCondition(PythonExpression(['not ', harmony_nmcl])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
        
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(ws_path,'harmony_nmcl', 'ros2_ws','src', 'nmcl_ros', 'launch',
                                                       'abb_lab_nmcl.launch')),
            condition=IfCondition(PythonExpression(harmony_nmcl)),
            launch_arguments={'dataFolder': "/home/ws/ros2_bridge/src/harmony_nmcl/ncore/data/ABB_lab/",
                              'configFolder': "/home/ws/ros2_bridge/src/harmony_nmcl/ncore/data/ABB_lab/config",
                              'modelPath': "/home/ws/ros2_bridge/src/harmony_nmcl/ros2_ws/src/yolov5_ros/",
                              'robot_name': "luzia_Alienware_m15_R4"}.items()),
    ])

    ld.add_action(declare_harmony_nmcl_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_log_level_cmd)
    
    #ld.add_action(bringup_cmd_group)

    node= Node(
        package=package_name,
        executable='mpc_planner_node',  # Replace with your node's executable name
        name='mpc_planner_node',
        output='screen',
        parameters=[config, {"package_path": package_path}],
    )
    ld.add_action(node)
    
    # node = Node(package='rviz2',
    #            executable='rviz2',
    #            output='log',
    #            parameters=[{'rviz.default_panel_visibility': False}],
    #            arguments=['-d', os.path.join(package_path, 'rviz', 'jackal_experiment.rviz')] +  ['>', '/dev/null', '2>&1'], )
    # ld.add_action(node)
    
    return ld

