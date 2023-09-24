import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
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

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)


    map_lifecycle_node = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])




    ld.add_action(map_server_node)
    ld.add_action(map_lifecycle_node)

    return ld
