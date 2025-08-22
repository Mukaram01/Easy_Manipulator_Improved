## Copyright 2020 ROS Industrial Consortium Asia Pacific
##
## Licensed under the Apache License, Version 2.0 (the "License");
## you may not use this file except in compliance with the License.
## You may obtain a copy of the License at
##
##     http://www.apache.org/licenses/LICENSE-2.0
##
## Unless required by applicable law or agreed to in writing, software
## distributed under the License is distributed on an "AS IS" BASIS,
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
## See the License for the specific language governing permissions and
## limitations under the License.

import os
import tempfile
import xacro
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

scene_pkg = 'scene_name'
robot_base_link = 'base_link_name'
robot_moveit_pkg = 'moveit_config_name'

def to_urdf(xacro_path, urdf_path=None):
    """Convert the given xacro file to a URDF file.

    * xacro_path -- the path to the xacro file
    * urdf_path -- the desired path to the URDF file
    """
    xacro_path = str(xacro_path)
    # If no URDF path is given, generate a temporary filename with a proper
    # ``.urdf`` extension.  ``tempfile.mktemp`` is avoided as it is vulnerable
    # to race conditions.  ``mkstemp`` provides a securely created file which we
    # immediately close before re-opening it for writing via xacro.
    if urdf_path is None:
        fd, urdf_path = tempfile.mkstemp(
            prefix=f"{Path(xacro_path).stem}_", suffix=".urdf"
        )
        os.close(fd)
    else:
        # Ensure the output file has a valid name, a ``.urdf`` extension and the directory
        # exists before writing. ``os.path.dirname`` returns an empty string
        # when ``urdf_path`` does not include any directory components (e.g.
        # ``"output"``).  Calling ``os.makedirs('')`` raises a
        # ``FileNotFoundError``.  Guard against this by only attempting to
        # create the directory when a directory path is provided.
        urdf_path = Path(urdf_path)
        # ``Path.name`` returns ``'.'`` for the current directory and ``'..'``
        # for the parent directory.  Both of those indicate that the provided
        # path refers to a directory rather than a file.  ``Path.with_suffix``
        # would raise ``ValueError`` for such paths, so detect them explicitly
        # and raise a clearer error message instead.
        if not urdf_path.name or urdf_path.name in {".", ".."}:
            raise ValueError("urdf_path must not be empty")
        urdf_path = str(urdf_path.with_suffix(".urdf"))
        directory = os.path.dirname(urdf_path)
        if directory:
            os.makedirs(directory, exist_ok=True)

    # open and process file
    doc = xacro.process_file(xacro_path)
    # open the output file using a context manager to ensure it is properly
    # closed and flushed to disk on all platforms
    with xacro.open_output(urdf_path) as out:
        out.write(doc.toprettyxml(indent='  '))

    return urdf_path  # Return path to the urdf file

def load_file(package_name, file_path):
    package_path = Path(get_package_share_directory(package_name))  # get package filepath
    absolute_file_path = package_path / file_path
    try:
        # Use a temporary directory so the generated URDF does not pollute the
        # package path and to avoid double ``.urdf`` extensions when the input
        # file already contains one.
        with tempfile.TemporaryDirectory() as tmpdir:
            temp_urdf_path = Path(tmpdir) / Path(file_path).with_suffix(".urdf").name
            temp_urdf_path = Path(to_urdf(str(absolute_file_path), str(temp_urdf_path)))
            with temp_urdf_path.open('r') as file:
                return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    return xacro.load_yaml(absolute_file_path)


def generate_launch_description():

    # Component yaml files are grouped in separate namespaces
    robot_description_config = load_file(scene_pkg, 'urdf/scene.urdf.xacro')
    robot_description = {'robot_description' : robot_description_config}

    robot_description_semantic_config = load_file(scene_pkg, 'urdf/arm_hand.srdf.xacro')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml(robot_moveit_pkg , 'config/kinematics.yaml')
    robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }

    ompl_planning_pipeline_config = { 'ompl' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }

    ompl_planning_yaml = load_yaml(robot_moveit_pkg, 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # RViz
    rviz_config_file = get_package_share_directory(scene_pkg) + "/launch/demo.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description,
                                 robot_description_semantic])
    # Publish base link TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', robot_base_link])

    return LaunchDescription([ static_tf, rviz_node])
