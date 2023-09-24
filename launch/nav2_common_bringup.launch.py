import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros.actions


def generate_launch_description():

    ld = LaunchDescription()
    params_file = os.path.join(
            get_package_share_directory('nav2_map_server_start'),
            'configs/', 'map_server_params.yaml')
    print("looking for params file in", params_file)

    map_file = os.path.join(
            get_package_share_directory('nav2_map_server_start'),
            'maps/', 'map.yaml')

    parameters =  LaunchConfiguration('params_file', default=params_file)
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')


    lifecycle_nodes = ['map_server']

    configured_params = RewrittenYaml(
        source_file=parameters, # note source_file is the simple yaml file
        root_key=namespace,
        param_rewrites={'use_sim_time' : use_sim_time, 'yaml_filename': map_file}, # note yaml_filename is the one associated with the .pgm
        convert_types=True)

    # remapppings = ('what the node expects', 'what it should expect in reality')
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    load_nodes = GroupAction(
        condition=True,
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    ld.add_action(load_nodes)

    return ld
