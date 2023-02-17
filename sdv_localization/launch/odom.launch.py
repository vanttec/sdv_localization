import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    this_dir = get_package_share_directory('sdv_localization')
    
    # Vectornav
    start_odom_pub = Node(
        package='sdv_localization', 
        executable='vn_ned_pose',
        output='screen')
    

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(start_odom_pub)

    return ld
