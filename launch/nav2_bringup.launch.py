import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions


def generate_launch_description():

    ld = LaunchDescription()
    default_params_file = os.path.join(
            get_package_share_directory('nav2_map_server_start'),
            'configs/', 'nav2_common_params.yaml')
    default_map_file = os.path.join(
            get_package_share_directory('nav2_map_server_start'),
            'maps/', 'map.yaml')

    parameters =  LaunchConfiguration('params_file', default=default_params_file)
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration('map', default=default_map_file)

    # remapppings = ('what the node expects', 'what it should expect in reality')
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': parameters,
                "map": map_yaml_path
            }.items()
        )

    ld.add_action(nav2_bringup_launch)

    return ld
