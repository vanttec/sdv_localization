# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import launch
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = launch.LaunchDescription()
    """Launch file to bring up visual slam node standalone."""
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        remappings=[('stereo_camera/left/camera_info', '/multisense/left/image_mono/camera_info'),
                    ('stereo_camera/right/camera_info', '/multisense/right/image_mono/camera_info'),
		    ('stereo_camera/right/image', 'multisense/right/image_mono'),
		    ('stereo_camera/left/image', 'multisense/left/image_mono'),
		    ('visual_slam/imu', 'vectornav/imu') 
		     ],
        parameters=[{
                    'use_sim_time': False,
                    'denoise_input_images': True,
                    'rectified_images': True,
                    'enable_imu': True,
                    'enable_slam_visualization': True,
                    'enable_observations_view': True,
                    'enable_landmarks_view': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/elbrus',
                    'map_frame': 'map',
                    'publish_tf':True,
                    'odom_frame': 'odom',	
                    'base_frame': 'base_link',
                    'input_imu_frame': 'odom'}])

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    node1 = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       name="tf_camera_to_baselink",
                       arguments = ["0", "0", "0", "0", "0", "0", "base_link","multisense/head"])
    
    ld.add_action(visual_slam_launch_container)
    ld.add_action(node1)
    return ld

