# Copyright 2020 Advanced Remanufacturing and Technology Centre
# Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('run_grasp_planner'),
        'config',
        'params_2f.yaml',
    )

    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/camera/color/image_raw',
        description='RGB image topic consumed by easy_perception_deployment',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth/image_rect_raw',
        description='Depth image topic consumed by easy_perception_deployment',
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/color/camera_info',
        description='Camera info topic consumed by easy_perception_deployment',
    )
    epd_output_topic_arg = DeclareLaunchArgument(
        'epd_output_topic',
        default_value='/easy_perception_deployment/epd_localize_output',
        description='Localized object output topic published by easy_perception_deployment',
    )

    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    epd_output_topic = LaunchConfiguration('epd_output_topic')

    epd_node = Node(
        package='easy_perception_deployment',
        executable='easy_perception_deployment',
        name='easy_perception_deployment',
        output='screen',
        remappings=[
            ('rgb_image', rgb_topic),
            ('depth_image', depth_topic),
            ('camera_info', camera_info_topic),
            ('/camera/color/image_raw', rgb_topic),
            ('/camera/depth/image_rect_raw', depth_topic),
            ('/camera/color/camera_info', camera_info_topic),
            ('/easy_perception_deployment/epd_localize_output', epd_output_topic),
        ],
    )

    grasp_planner_node = Node(
        package='run_grasp_planner',
        name='grasp_planning_node',
        executable='demo_node',
        output='screen',
        parameters=[
            config,
            {
                'easy_perception_deployment.epd_enabled': True,
                'easy_perception_deployment.epd_localization_topic': epd_output_topic,
            },
        ],
    )

    return LaunchDescription([
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        epd_output_topic_arg,
        epd_node,
        grasp_planner_node,
    ])
