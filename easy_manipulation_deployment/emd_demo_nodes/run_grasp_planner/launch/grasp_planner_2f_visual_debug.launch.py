import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('run_grasp_planner'),'config','params_2f.yaml')
    return LaunchDescription([
        Node(
            package='run_grasp_planner',
            name='grasp_planning_node',
            executable='demo_node',
            output='screen',
            parameters=[config, {
                'visualization_params.point_cloud_visualization': True,
                'visualization_params.point_cloud_visualization_blocking': False,
            }],
        )
    ])
