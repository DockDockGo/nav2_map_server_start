import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='/home/amr-orin-0/mfi_amr_ws/src/nav2_map_server_start/maps/map.yaml',  # Specify your default path here
            description='Full path to the .yaml file specifying the map',
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False}],
            # remappings=[('/map', 'your_map_namespace/map')],
            arguments=[LaunchConfiguration('map_file')],
        )
    ])
