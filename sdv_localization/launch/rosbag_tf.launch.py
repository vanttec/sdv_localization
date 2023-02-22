import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    this_dir = get_package_share_directory('sdv_localization')
    
    # Vectornav odometry and path
    start_odom_pub = Node(
        package='sdv_localization', 
        executable='vn_ned_pose',
        output='screen',
        parameters=[os.path.join(this_dir, 'config', 'VnNedPoseNode.yaml')])
    
    start_transform_odom_base_link = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name="tf_odom_to_base_link",
            arguments = ['0', '0', '-0.41', '0', '0', '0', 'odom', 'base_link'])
    
    start_transform_base_link_velodyne = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name="tf_base_link_to_velodyne",
            arguments = ['-0.2', '0.26', '-0.17', '-1.047191', '3.145926', '0', 'base_link', 'velodyne'])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(start_odom_pub)
    ld.add_action(start_transform_odom_base_link)
    ld.add_action(start_transform_base_link_velodyne)
    return ld
