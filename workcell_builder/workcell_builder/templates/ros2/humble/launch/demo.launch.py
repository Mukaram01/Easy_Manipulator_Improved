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

import logging
import os
import tempfile
from pathlib import Path
from typing import Any, Optional, Union

import xacro
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

scene_pkg = 'scene_name'
robot_base_link = 'base_link_name'
robot_moveit_pkg = 'moveit_config_name'
logger = logging.getLogger(__name__)

def to_urdf(
    xacro_path: Union[str, os.PathLike[str]],
    urdf_path: Optional[Union[str, os.PathLike[str]]] = None,
) -> str:
    """Convert the given xacro file to a URDF file.

    * xacro_path -- the path to the xacro file
    * urdf_path -- the desired path to the URDF file
    """
    # Validate that the source xacro file exists before attempting to convert
    # it. ``xacro`` will eventually raise a somewhat cryptic error if the file
    # is missing; performing an explicit check here gives a clearer
    # ``FileNotFoundError`` to callers.
    # Expand the user home directory (``~``) in the supplied path so that
    # callers can rely on standard shell semantics.  Without this a path such
    # as ``~/file.xacro`` would be interpreted literally and the subsequent
    # ``is_file`` check would fail even though the file exists.
    xacro_path = Path(xacro_path).expanduser()
    if not xacro_path.is_file():
        raise FileNotFoundError(f"xacro file '{xacro_path}' does not exist")
    # ``xacro`` is designed to process files ending with the ``.xacro``
    # extension.  Accepting other extensions would result in confusing error
    # messages downstream when ``xacro`` attempts to parse non-xacro input.
    # Validate the extension early to provide a clear and actionable error to
    # callers.
    if xacro_path.suffix != ".xacro":
        raise ValueError("xacro_path must point to a .xacro file")

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
        orig_urdf_path = str(urdf_path)
        # Allow ``~`` in the output path in the same way as the xacro path
        # above.  ``Path`` would otherwise treat the tilde as a literal
        # character which leads to confusing paths like ``~/out.urdf`` being
        # created in the current working directory.
        urdf_path = Path(urdf_path).expanduser()
        # ``Path.name`` returns ``'.'`` for the current directory and ``'..'``
        # for the parent directory.  Both of those indicate that the provided
        # path refers to a directory rather than a file.  Trailing path
        # separators or references to an existing directory should likewise be
        # rejected to avoid writing ``*.urdf`` files with unexpected names such
        # as ``dir.urdf`` when a directory was intended.
        if (
            not urdf_path.name
            or urdf_path.name in {".", ".."}
            or orig_urdf_path.endswith(os.path.sep)
            or urdf_path.is_dir()
        ):
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


def load_file(package_name: str, file_path: str) -> Optional[str]:
    """Load a file from a package, converting xacro files to URDF as needed.

    Returns the file contents as a string or ``None`` if the package could not
    be located or the conversion fails.
    """

    try:
        package_path = Path(get_package_share_directory(package_name))
        file_path = Path(file_path).expanduser()
        if not file_path.is_absolute():
            absolute_file_path = package_path / file_path
        else:
            absolute_file_path = file_path

        # Use a temporary directory so the generated URDF does not pollute the
        # package path and to avoid double ``.urdf`` extensions when the input
        # file already contains one.
        with tempfile.TemporaryDirectory() as tmpdir:
            filename = Path(file_path)

            # Only invoke ``to_urdf`` when the source file is a xacro file.
            # ``to_urdf`` raises a ``ValueError`` for non-xacro input which
            # previously caused ``load_file`` to return ``None`` even for valid
            # URDF files.  By checking the extension up-front we can simply
            # read URDF files directly.
            if filename.suffix == ".xacro":
                filename = filename.with_suffix("")
                if filename.suffix != ".urdf":
                    filename = filename.with_suffix(".urdf")
                temp_urdf_path = Path(tmpdir) / filename.name
                temp_urdf_path = Path(
                    to_urdf(str(absolute_file_path), str(temp_urdf_path))
                )
            else:
                temp_urdf_path = absolute_file_path

            # Explicitly specify UTF-8 encoding to avoid platform-dependent
            # defaults which may not handle non-ASCII characters correctly.
            with Path(temp_urdf_path).open("r", encoding="utf-8") as file:
                return file.read()
    except Exception:
        # ``to_urdf`` may raise a variety of exceptions when the conversion
        # fails (e.g. malformed xacro, missing files or packages).  Swallow
        # any such error and indicate the failure by returning ``None`` to the
        # caller.
        return None


def load_yaml(package_name: str, file_path: str) -> Any | None:
    """Load a YAML file from the given package.

    Returns the parsed YAML content or ``None`` if parsing fails or the package
    cannot be located.  Raises ``FileNotFoundError`` when the requested YAML
    file does not exist.
    """

    try:
        package_path = Path(get_package_share_directory(package_name))
    except Exception:
        # ``get_package_share_directory`` raises a package specific
        # exception when the package cannot be located.  For parity with
        # :func:`load_file` catch any such error and return ``None`` so
        # callers can handle the missing package gracefully instead of
        # the exception bubbling up.
        return None

    file_path = Path(file_path).expanduser()
    if not file_path.is_absolute():
        yaml_file = package_path / file_path
    else:
        yaml_file = file_path
    if not yaml_file.is_file():
        raise FileNotFoundError(f"YAML file '{yaml_file}' does not exist")
    try:
        # Use UTF-8 to ensure consistent behaviour across locales.
        with yaml_file.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except yaml.YAMLError:
        return None


def generate_launch_description() -> LaunchDescription:

    # Component yaml files are grouped in separate namespaces
    robot_description_config = load_file(scene_pkg, 'urdf/scene.urdf.xacro')
    if robot_description_config is None:
        logger.error(
            "Failed to load robot description for '%s' from %s",
            scene_pkg,
            'urdf/scene.urdf.xacro',
        )
        raise RuntimeError("robot_description is unavailable")
    robot_description = {'robot_description' : robot_description_config}

    robot_description_semantic_config = load_file(scene_pkg, 'urdf/arm_hand.srdf.xacro')
    if robot_description_semantic_config is None:
        logger.error(
            "Failed to load semantic robot description for '%s' from %s",
            scene_pkg,
            'urdf/arm_hand.srdf.xacro',
        )
        raise RuntimeError("robot_description_semantic is unavailable")
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    try:
        kinematics_yaml = load_yaml(robot_moveit_pkg , 'config/kinematics.yaml')
    except FileNotFoundError as exc:
        logger.warning(
            "Kinematics YAML not found for '%s': %s",
            robot_moveit_pkg,
            exc,
        )
        kinematics_yaml = None

    ompl_planning_pipeline_config = { 'ompl' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }

    ompl_planning_yaml = load_yaml(robot_moveit_pkg, 'config/ompl_planning.yaml')
    if ompl_planning_yaml is None:
        logger.warning(
            "OMPL planning YAML not found or invalid for '%s'",
            robot_moveit_pkg,
        )
    else:
        ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # RViz
    rviz_config_file = get_package_share_directory(scene_pkg) + "/launch/demo.rviz"
    rviz_parameters = [
        robot_description,
        robot_description_semantic,
    ]
    if kinematics_yaml is not None:
        rviz_parameters.append(
            { 'robot_description_kinematics' : kinematics_yaml }
        )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=rviz_parameters,
    )
    # Publish base link TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', robot_base_link])

    return LaunchDescription([ static_tf, rviz_node])
