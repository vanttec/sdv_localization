import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    this_dir = get_package_share_directory('sdv_localization')

    start_map_broadcaster =  Node(
            package='nav2_map_server', executable='map_server',
            parameters=[os.path.join(this_dir, 'config', 'map_server_params.yaml')],
            name='map_server',
            output='screen'
        )


    start_lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        )



    ld = LaunchDescription()

    ld.add_action(start_map_broadcaster)
    ld.add_action(start_lifecycle_manager)

    return ld
