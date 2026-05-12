from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

DESCRIPTION_PACKAGE = "table_clamp_camera_mount_description"

def generate_launch_description():
    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare(DESCRIPTION_PACKAGE), "urdf", "test_table_clamp_camera_mount.urdf.xacro"]),
    ]), value_type=str)

    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": robot_description}]),
        Node(package="rviz2", executable="rviz2", output="screen", arguments=["-d", PathJoinSubstitution([FindPackageShare(DESCRIPTION_PACKAGE), "rviz", "view_table_clamp_camera_mount.rviz"])]),
    ])
