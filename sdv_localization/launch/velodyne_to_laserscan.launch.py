from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/velodyne_points'),
                        ('scan', '/full_scan')],
            parameters=[{
                'target_frame': 'velodyne',
                'transform_tolerance': 0.40,
                'min_height': -0.65,
                'max_height': 2.0,
                'angle_min': -3.1415927410125732,  # -M_PI/2
                'angle_max': 3.1415927410125732,  # M_PI/2
                'angle_increment': 0.007000000216066837,  # M_PI/360.0
                'scan_time': 0.03333,
                'range_min': 0.0,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
