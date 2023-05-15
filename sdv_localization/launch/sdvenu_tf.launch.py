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
        executable='vn_gps_pose',
        output='screen',
        parameters=[os.path.join(this_dir, 'config', 'VnNedPoseNode.yaml')])
    

    #ODOM is in NED frame since vn measurements are in that configuration, so the rotation from NED to ENU for MAP is necessary
    start_transform_odom_base_link = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name="tf_map_to_odom",
            arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']) #x, y, z, yaw, pitch, roll 
    
    # In yaw, pitch,roll, it translates to -90, 180, -9 degrees to replicate the connection between the velodyne and the vn
    # The velodyne sensor has a slight offset in yaw, so a 1.5 degree offset is applied 
    # First rosbags have a -11 offset to them since it had a angle to the lidar
    # New, full rosbags are planar, so a roll offset is not necessary
    start_transform_base_link_velodyne = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name="tf_base_link_to_velodyne",
            arguments = ['0.0', '0.48', '0.14', '-2.08204', '0.0', '0.0', 'base_link', 'velodyne'])
    
    start_transform_base_link_base_footprint = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name="tf_base_link_to_base_footprint",
                                #Real distance from ground to vn is 1.9m, distance from car base to vn is 1.3m
            arguments = ['0.0', '0.0', '-1.9', '0', '0', '0', 'base_link', 'base_footprint'])
                

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(start_odom_pub)
    ld.add_action(start_transform_odom_base_link)
    ld.add_action(start_transform_base_link_velodyne)
    ld.add_action(start_transform_base_link_base_footprint)
    return ld
