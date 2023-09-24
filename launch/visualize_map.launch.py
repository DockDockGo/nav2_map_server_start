import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    rviz_default_path = os.path.join(get_package_share_directory('nav2_map_server_start'),
            'configs/', 'map_viz.rviz')
    rviz_config_path = LaunchConfiguration('rviz_config', default=rviz_default_path)
    rviz_config_path_str = rviz_config_path.perform(context=LaunchContext())

    # get map path
    map_default_path = os.path.join(
            get_package_share_directory('nav2_map_server_start'),
            'maps/', 'map.yaml')
    map_file_path = LaunchConfiguration('map_path', default=map_default_path)
    map_file = map_file_path.perform(context=LaunchContext())

    nav2_map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('nav2_map_server_start'),
        'launch',
        'nav2_map_server_start.launch.py'
        ))
    )

    # Launch RViz
    rviz_launch = launch.actions.ExecuteProcess(
                    cmd=['rviz2', '-d', rviz_config_path_str],
                    output='screen')

    # Sleep for a moment then call map_server_service
    map_server_service_call = launch.actions.TimerAction(
            period=LaunchConfiguration('delay_duration', default='5'),  # Adjust the delay as needed
            actions=[
                # Run the service call after a delay
                launch.actions.ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/map_server/load_map',
                        'nav2_msgs/srv/LoadMap',
                        '{map_url: "' + map_file + '"}'
                    ],
                    output='screen'
                ),
            ],
        )

    ld.add_action(nav2_map_server_launch)
    ld.add_action(rviz_launch)
    ld.add_action(map_server_service_call)

    return ld
