# Copyright 2020 ROS Industrial Consortium Asia Pacific
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
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

import xacro

scene_pkg = 'ur5_2f_test'
robot_base_link = 'base_link'
package_name = 'run_waypoint_execution'


def to_urdf(xacro_path, urdf_path=None, mappings=None):
    """Convert the given xacro file to a URDF file."""
    import atexit

    xacro_path = str(xacro_path)

    if urdf_path is None:
        fd, urdf_path = tempfile.mkstemp(
            prefix=f"{Path(xacro_path).stem}_", suffix=".urdf"
        )
        os.close(fd)
        # Register cleanup so the temp file is removed when the launch process exits.
        atexit.register(lambda p=urdf_path: Path(p).unlink(missing_ok=True))
    else:
        urdf_path = Path(urdf_path)
        if not urdf_path.name or urdf_path.name in {".", ".."}:
            raise ValueError("urdf_path must not be empty")
        urdf_path = str(urdf_path.with_suffix(".urdf"))
        directory = os.path.dirname(urdf_path)
        if directory:
            os.makedirs(directory, exist_ok=True)

    # Use xacro's own toxml() serialiser instead of toprettyxml() to avoid
    # whitespace artefacts (extra text nodes in <joint>/<link> elements) and
    # the mandatory XML declaration that toprettyxml() injects.
    doc = xacro.process_file(xacro_path, mappings=mappings)
    with open(urdf_path, "w", encoding="utf-8") as out:
        out.write(doc.toxml())

    return urdf_path


def load_file(package_name, file_path, mappings=None):
    """Load a robot description file, converting xacro sources to URDF."""

    package_path = Path(get_package_share_directory(package_name))
    target = Path(file_path)
    absolute_file_path = (package_path / target) if not target.is_absolute() else target

    try:
        with tempfile.TemporaryDirectory() as tmpdir:
            if target.suffix == ".xacro":
                temp_urdf_path = Path(tmpdir) / target.with_suffix(".urdf").name
                temp_urdf_path = Path(
                    to_urdf(str(absolute_file_path), str(temp_urdf_path), mappings)
                )
            else:
                temp_urdf_path = absolute_file_path

            with Path(temp_urdf_path).open("r", encoding="utf-8") as file:
                return file.read()
    except Exception:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    return xacro.load_yaml(absolute_file_path)


def generate_launch_description():
    # moveit_cpp.yaml is passed by filename for now since it's node specific
    grasp_execution_yaml_file_name = (
        get_package_share_directory(package_name) + '/config/grasp_execution.yaml'
    )

    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false', description='Launch in debug mode'
    )

    # Initial position mapping
    initial_position_path = (get_package_share_directory(package_name) +
                             '/config/start_positions.yaml')
    initial_position_mappings = {
        'initial_positions_file': initial_position_path}

    # Component yaml files are grouped in separate namespaces
    robot_description_config = load_file(scene_pkg, 'urdf/scene.urdf.xacro',
                                         initial_position_mappings)
    robot_description = {'robot_description': robot_description_config}

    robot_description_semantic_config = load_file(scene_pkg, 'urdf/arm_hand.srdf.xacro')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('ur5_moveit_config', 'config/kinematics.yaml')

    ompl_planning_pipeline_config = {
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ('default_planner_request_adapters/AddTimeOptimalParameterization '
                                 'default_planner_request_adapters/FixWorkspaceBounds '
                                 'default_planner_request_adapters/FixStartStateBounds '
                                 'default_planner_request_adapters/FixStartStateCollision '
                                 'default_planner_request_adapters/FixStartStatePathConstraints'),
            'start_state_max_bounds_error': 0.1}
        }

    ompl_planning_yaml = load_yaml('ur5_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.01}

    controllers_yaml = load_yaml(package_name, 'config/controllers.yaml')
    moveit_controller = {
        'moveit_simple_controller_manager': controllers_yaml,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager'
        }

    joint_limits_yaml = load_yaml('ur5_moveit_config', 'config/joint_limits.yaml')
    joint_limits = {'robot_description_planning': joint_limits_yaml}

    workcell_context_yaml = os.path.join(
        get_package_share_directory(package_name), 'config', 'workcell_context.yaml')
    workcell_context = {'workcell_context': workcell_context_yaml}

    # MoveItCpp demo executable
    grasp_execution_demo_node = Node(
        name='grasp_execution_node',
        package=package_name,
        prefix=PythonExpression(
            [
                "'xterm -e gdb --args' if '",
                LaunchConfiguration('debug'),
                "' == 'true' else ''",
            ]
        ),
        executable='demo_node',
        output='screen',
        parameters=[
            grasp_execution_yaml_file_name,
            robot_description,
            robot_description_semantic,
            joint_limits,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            workcell_context,
            moveit_controller,
        ],
    )

    # RViz
    rviz_config_file = (get_package_share_directory(package_name) +
                        '/config/grasp_execution.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description,
                    robot_description_semantic]
        )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
        )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ur5_moveit_config"),
        "config",
        "ur5_ros_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Load controllers
    load_controllers = []
    for controller in ["ur5_arm_controller", "joint_state_broadcaster"]:
        load_controllers += [
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "controller_manager",
                    "spawner",
                    controller,
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            debug_arg,
            robot_state_publisher,
            rviz_node,
            grasp_execution_demo_node,
            ros2_control_node,
        ]
        + load_controllers
    )
