import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='/home/sush/mfi/ros2_ws/bags/sep_16_data/map.yaml',  # Specify your default path here
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
        ),
        LogInfo(
            condition=launch.conditions.UnlessCondition(LaunchConfiguration('map_file')),
            format="Using default map file path: '/home/sush/mfi/ros2_ws/bags/sep_16_data/map.yaml'.",
        ),
    ])