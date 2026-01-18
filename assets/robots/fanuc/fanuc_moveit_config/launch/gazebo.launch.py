from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    paused_arg = DeclareLaunchArgument('paused', default_value='false')
    gazebo_gui_arg = DeclareLaunchArgument('gazebo_gui', default_value='true')
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution(
            [FindPackageShare('moveit_resources_fanuc_description'), 'urdf', 'fanuc.urdf']
        ),
    )

    world_path = PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'worlds', 'empty.world'])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'world': world_path,
            'pause': LaunchConfiguration('paused'),
            'gui': LaunchConfiguration('gazebo_gui'),
        }.items(),
    )

    robot_description = ParameterValue(
        Command(['cat', LaunchConfiguration('urdf_path')]),
        value_type=str,
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'robot', '-param', 'robot_description', '-x', '0', '-y', '0', '-z', '0'],
        parameters=[{'robot_description': robot_description}],
    )

    return LaunchDescription(
        [
            paused_arg,
            gazebo_gui_arg,
            urdf_path_arg,
            gazebo_launch,
            spawn_entity,
        ]
    )
