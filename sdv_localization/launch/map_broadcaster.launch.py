from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='nav2_map_server', executable='map_server',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': "/home/ws/sdv_localization_ws/map_1681870113.yaml"
            }],
            name='map_broadcaster'
        )
    ])
