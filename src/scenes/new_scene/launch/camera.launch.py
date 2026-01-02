from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device_type', default_value='d435i'),
        
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'rgb_camera.profile': '640,480,15',
                'depth_module.profile': '640,480,15',
                'pointcloud.enable': True,
                'device_type': LaunchConfiguration('device_type'),
                'enable_gyro': False,
                'enable_accel': False,
                'publish_tf': False,  # IMPORTANT: Disable TF from camera
                'tf_publish_rate': 0.0,
            }]
        ),
    ])
