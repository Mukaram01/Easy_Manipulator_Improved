from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        )
    )

    robot_description = ParameterValue(
        Command(
            [
                'cat',
                PathJoinSubstitution(
                    [FindPackageShare('double_fixture'), 'urdf', 'double_fixture.urdf']
                ),
            ]
        ),
        value_type=str,
    )

    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint', '40'],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'double_fixture', '-param', 'robot_description'],
        parameters=[{'robot_description': robot_description}],
    )

    fake_joint_calibration = ExecuteProcess(
        cmd=[
            'ros2',
            'topic',
            'pub',
            '/calibrated',
            'std_msgs/msg/Bool',
            '{data: true}',
            '--once',
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            gazebo_launch,
            static_transform,
            spawn_entity,
            fake_joint_calibration,
        ]
    )
