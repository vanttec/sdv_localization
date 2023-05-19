# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode



def generate_launch_description():
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
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_slam_visualization': True,
                    'enable_observations_view': True,
                    'enable_landmarks_view': True,
                    'enable_imu' : True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/elbrus',
                    'input_base_frame': 'base_link',
                    'input_imu_frame' : 'base_link',
                    'publish_odom_to_base_tf': 'false',
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link'
                    'input_imu_frame':'odom'
                    }]
    )


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
                       arguments = ["1.20", "0.1", "-1.22", "0", "0", "0", "base_link","multisense/head"])

    ld.add_action(visual_slam_launch_container)
    ld.add_action(node1)
    return ld

