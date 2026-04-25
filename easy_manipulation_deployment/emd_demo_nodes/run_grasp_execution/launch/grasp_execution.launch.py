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
import re
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

import xacro
import yaml

DEFAULT_SCENE_PACKAGE_CANDIDATES = ("ur5_3f_test", "ur5_2f_test", "ur5_airpick4_test", "suction_test")
PACKAGE_NAME = "run_grasp_execution"
SCENE_PACKAGE_ARGUMENT = "scene_package"
MOVEIT_CONFIG_PACKAGE_ARGUMENT = "moveit_config_package"
PLANNING_FRAME_ARGUMENT = "planning_frame"


GRIPPER_CONTROLLER_JOINTS_BY_END_EFFECTOR = {
    "robotiq_3f": (
        "palm_finger_1_joint",
        "finger_1_joint_1",
        "finger_1_joint_2",
        "finger_1_joint_3",
        "palm_finger_2_joint",
        "finger_2_joint_1",
        "finger_2_joint_2",
        "finger_2_joint_3",
        "finger_middle_joint_1",
        "finger_middle_joint_2",
        "finger_middle_joint_3",
    ),
    "robotiq_2f": ("gripper_finger1_joint",),
}

KNOWN_PLANNER_END_EFFECTOR_IDS = {"robotiq_2f", "robotiq_3f", "suction_cup", "suction", "airpick"}


def scene_exposes_gripper_position_interfaces(robot_description_xml, gripper_controller_joints):
    for joint_name in gripper_controller_joints:
        joint_tag = f'<joint name="{joint_name}">'
        if joint_tag not in robot_description_xml:
            return False
    return True


def resolve_gripper_controller_joints(scene_package, scene_metadata=None):
    scene_metadata = scene_metadata if scene_metadata is not None else load_scene_environment(scene_package)
    end_effector = scene_metadata.get("end_effector", {})
    ee_name = str(end_effector.get("name", "")).lower()
    ee_brand = str(end_effector.get("brand", "")).lower()
    ee_fingers = end_effector.get("attributes", {}).get("fingers")

    if "robotiq_3f" in ee_name or "robotiq_3f" in ee_brand or ee_fingers == 3:
        return GRIPPER_CONTROLLER_JOINTS_BY_END_EFFECTOR["robotiq_3f"]
    if "robotiq_85" in ee_name or "robotiq_85" in ee_brand or ee_fingers == 2:
        return GRIPPER_CONTROLLER_JOINTS_BY_END_EFFECTOR["robotiq_2f"]
    return ()


def _normalize_text(value):
    return str(value or "").strip().lower()


def load_scene_environment(scene_package):
    scene_metadata = load_yaml(scene_package, "environment.yaml")
    if not isinstance(scene_metadata, dict):
        raise RuntimeError(
            "Invalid scene metadata schema in 'environment.yaml': expected a YAML mapping (dict) at the root, "
            f"got {type(scene_metadata).__name__}. Check '{SCENE_PACKAGE_ARGUMENT}' points to a valid generated "
            "scene package with a proper environment.yaml structure."
        )
    return scene_metadata


def derive_planner_end_effector_id(end_effector):
    ee_name = _normalize_text(end_effector.get("name"))
    ee_brand = _normalize_text(end_effector.get("brand"))
    ee_type = _normalize_text(end_effector.get("ee_type"))
    ee_fingers = end_effector.get("attributes", {}).get("fingers")

    if ee_name in KNOWN_PLANNER_END_EFFECTOR_IDS:
        return ee_name
    if ee_brand in KNOWN_PLANNER_END_EFFECTOR_IDS:
        return ee_brand

    search_blob = " ".join((ee_name, ee_brand, ee_type))
    if "robotiq_3f" in search_blob or ee_fingers == 3:
        return "robotiq_3f"
    if any(marker in search_blob for marker in ("robotiq_2f", "robotiq_85")) or ee_fingers == 2:
        return "robotiq_2f"
    if any(marker in search_blob for marker in ("suction", "single_suction", "airpick")):
        # Planner conventions are usually "suction_cup"; this preserves compatibility
        # with scenes that encode suction variants in name/brand.
        return "suction_cup"

    return "ur_tool0"


def derive_workcell_end_effector_link(end_effector):
    robot_link = _normalize_text(end_effector.get("robot_link"))
    if robot_link:
        return robot_link
    return "tool0"


def derive_workcell_grasp_frame(end_effector):
    for key in ("grasp_frame", "tcp_link", "physical_ee_link", "base_link", "link"):
        value = _normalize_text(end_effector.get(key))
        if value:
            return value
    return ""


def build_workcell_context_for_scene(scene_package, scene_metadata):
    end_effector = scene_metadata.get("end_effector", {})
    if end_effector is None:
        end_effector = {}
    if not isinstance(end_effector, dict):
        raise RuntimeError(
            f"Invalid scene metadata in package '{scene_package}': 'end_effector' must be a YAML mapping (dict), "
            f"got {type(end_effector).__name__}."
        )

    ee_id = derive_planner_end_effector_id(end_effector)
    ee_link = derive_workcell_end_effector_link(end_effector)
    ee_grasp_frame = derive_workcell_grasp_frame(end_effector) or ee_link
    return {
        "workcell": {
            "ros__parameters": {
                "groups": ["manipulator"],
                "groups.manipulator.executors": ["default"],
                "groups.manipulator.executors.default.plugin": "grasp_execution/DefaultExecutor",
                "groups.manipulator.end_effectors": [ee_id],
                f"groups.manipulator.end_effectors.{ee_id}.brand": ee_id,
                f"groups.manipulator.end_effectors.{ee_id}.link": ee_link,
                f"groups.manipulator.end_effectors.{ee_id}.grasp_frame": ee_grasp_frame,
                f"groups.manipulator.end_effectors.{ee_id}.clearance": 0.1,
                f"groups.manipulator.end_effectors.{ee_id}.driver.plugin": "grasp_execution/DummyGripperDriver",
                f"groups.manipulator.end_effectors.{ee_id}.driver.controller": "",
            }
        }
    }, ee_id, ee_link, ee_grasp_frame


def align_gripper_controller_joints(
    controllers_yaml, ros2_controllers_yaml, gripper_controller_joints, enable_gripper_controller
):
    controller_names = controllers_yaml.setdefault("controller_names", [])
    if controller_names is None:
        controller_names = []
        controllers_yaml["controller_names"] = controller_names
    elif not isinstance(controller_names, list):
        raise RuntimeError(
            "Invalid controllers schema: 'controller_names' must be a YAML sequence (list), "
            f"got {type(controller_names).__name__}."
        )

    controller_manager = ros2_controllers_yaml.setdefault("controller_manager", {})
    if controller_manager is None:
        controller_manager = {}
        ros2_controllers_yaml["controller_manager"] = controller_manager
    elif not isinstance(controller_manager, dict):
        raise RuntimeError(
            "Invalid ros2 controllers schema: 'controller_manager' must be a YAML mapping (dict), "
            f"got {type(controller_manager).__name__}."
        )

    controller_manager_ros_params = controller_manager.setdefault("ros__parameters", {})
    if controller_manager_ros_params is None:
        controller_manager_ros_params = {}
        controller_manager["ros__parameters"] = controller_manager_ros_params
    elif not isinstance(controller_manager_ros_params, dict):
        raise RuntimeError(
            "Invalid ros2 controllers schema: 'controller_manager.ros__parameters' must be a YAML "
            f"mapping (dict), got {type(controller_manager_ros_params).__name__}."
        )

    if gripper_controller_joints and enable_gripper_controller:
        gripper_joints = list(gripper_controller_joints)
        controllers_yaml.setdefault("ur5_gripper_controller", {})["joints"] = gripper_joints
        ros2_controllers_yaml.setdefault("ur5_gripper_controller", {}).setdefault("ros__parameters", {})[
            "joints"
        ] = gripper_joints
        if "ur5_gripper_controller" not in controller_names:
            controller_names.append("ur5_gripper_controller")
        controller_manager_ros_params.setdefault(
            "ur5_gripper_controller", {"type": "joint_trajectory_controller/JointTrajectoryController"}
        )
    else:
        controller_names[:] = [name for name in controller_names if name != "ur5_gripper_controller"]
        controllers_yaml.pop("ur5_gripper_controller", None)
        ros2_controllers_yaml.pop("ur5_gripper_controller", None)
        controller_manager_ros_params.pop("ur5_gripper_controller", None)

def find_default_scene_package():
    for package_name in DEFAULT_SCENE_PACKAGE_CANDIDATES:
        try:
            get_package_share_directory(package_name)
            return package_name
        except PackageNotFoundError:
            continue
    return None


DEFAULT_SCENE_PACKAGE = find_default_scene_package()


def to_urdf(xacro_path, urdf_path=None, mappings=None):
    import atexit
    xacro_path = str(xacro_path)

    if urdf_path is None:
        fd, urdf_path = tempfile.mkstemp(prefix=f"{Path(xacro_path).stem}_", suffix=".urdf")
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


def load_file(package, file_path, mappings=None):
    logger = get_logger(__name__)
    package_path = Path(get_package_share_directory(package))
    target = Path(file_path)
    absolute_file_path = (package_path / target) if not target.is_absolute() else target

    try:
        with tempfile.TemporaryDirectory() as tmpdir:
            if target.suffix == ".xacro":
                temp_urdf_path = Path(tmpdir) / target.with_suffix(".urdf").name
                temp_urdf_path = Path(to_urdf(str(absolute_file_path), str(temp_urdf_path), mappings))
            else:
                temp_urdf_path = absolute_file_path

            with Path(temp_urdf_path).open("r", encoding="utf-8") as f:
                return f.read()
    except Exception as exc:
        msg = f"Failed to load robot description file from '{absolute_file_path}'. Original error: {exc}"
        logger.error(msg)
        raise RuntimeError(msg) from exc


def load_yaml(package, file_path):
    package_path = get_package_share_directory(package)
    yaml_path = os.path.join(package_path, file_path)

    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return data if data is not None else {}
    except Exception as exc:
        raise RuntimeError(
            f"Failed to load YAML from resolved path '{yaml_path}' "
            f"(package='{package}', file='{file_path}'). Original error: {exc}"
        ) from exc


def write_temp_yaml_params(data, prefix="ros2_controllers_"):
    import atexit

    fd, path = tempfile.mkstemp(prefix=prefix, suffix=".yaml")
    os.close(fd)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
    atexit.register(lambda p=path: Path(p).unlink(missing_ok=True))
    return path


def require_yaml_mapping(data, field_name, package_name, file_name):
    if not isinstance(data, dict):
        raise RuntimeError(
            f"Invalid YAML schema for package '{package_name}', file '{file_name}': '{field_name}' must be a "
            f"YAML mapping (dict), got {type(data).__name__}. Please verify the file content and launch arguments."
        )
    return data


def _extract_scene_xacro_args(xacro_file_path):
    text = Path(xacro_file_path).read_text(encoding="utf-8")
    return set(re.findall(r"<xacro:arg\s+name=['\"]([^'\"]+)['\"]", text))


def _extract_ur_robot_macro_params():
    try:
        ur_macro_path = Path(get_package_share_directory("ur_description")) / "urdf" / "ur_macro.xacro"
    except PackageNotFoundError:
        return set()

    if not ur_macro_path.exists():
        return set()

    text = ur_macro_path.read_text(encoding="utf-8")
    call_start = text.find("<xacro:macro")
    if call_start == -1:
        return set()

    ur_robot_block_start = text.find('name="ur_robot"', call_start)
    if ur_robot_block_start == -1:
        return set()

    params_match = re.search(r'params\s*=\s*["\'](.*?)["\']', text[ur_robot_block_start:], re.DOTALL)
    if not params_match:
        return set()

    raw_params = params_match.group(1).split()
    return {token.lstrip("*") for token in raw_params if token and ":=" not in token}


def _extract_link_names_from_urdf(robot_description_xml):
    try:
        root = ET.fromstring(robot_description_xml)
    except ET.ParseError:
        return set()
    return {link.attrib.get("name") for link in root.findall("link") if link.attrib.get("name")}


def _normalize_srdf_for_ur_base_inertia(robot_description_xml, srdf_xml, logger):
    required_pairs = (("base_link", "base_link_inertia"), ("base_link_inertia", "shoulder_link"))
    link_names = _extract_link_names_from_urdf(robot_description_xml)
    required_links = {"base_link_inertia", "shoulder_link"}
    if not required_links.issubset(link_names):
        return srdf_xml

    try:
        root = ET.fromstring(srdf_xml)
    except ET.ParseError as exc:
        logger.warning(
            f"Skipping UR base_link_inertia SRDF normalization due to parse error: {exc}"
        )
        return srdf_xml

    existing_pairs = set()
    for element in root.findall("disable_collisions"):
        link1 = element.attrib.get("link1", "")
        link2 = element.attrib.get("link2", "")
        if link1 and link2:
            existing_pairs.add(tuple(sorted((link1, link2))))

    injected_pairs = []
    for link1, link2 in required_pairs:
        pair_key = tuple(sorted((link1, link2)))
        if pair_key in existing_pairs:
            continue
        ET.SubElement(root, "disable_collisions", link1=link1, link2=link2, reason="Adjacent")
        existing_pairs.add(pair_key)
        injected_pairs.append(f"{link1}<->{link2}")

    if injected_pairs:
        logger.info(
            "Injected SRDF disable_collisions pairs for UR base_link_inertia compatibility: "
            + ", ".join(injected_pairs)
        )
        return ET.tostring(root, encoding="unicode")

    return srdf_xml


def resolve_required_package_share_dir(package_name, remediation_hint):
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f"Required package '{package_name}' was not found in ament_index. {remediation_hint}"
        ) from exc


def resolve_scene_package_share_dir(scene_package):
    try:
        return get_package_share_directory(scene_package)
    except PackageNotFoundError as exc:
        available = ", ".join(DEFAULT_SCENE_PACKAGE_CANDIDATES)
        raise RuntimeError(
            f"Scene package '{scene_package}' was not found. Pass a valid '{SCENE_PACKAGE_ARGUMENT}' launch "
            f"argument, for example '{SCENE_PACKAGE_ARGUMENT}:={available.split(', ')[0]}', or build/source "
            "your generated scene package first."
        ) from exc


def launch_setup(context, *args, **kwargs):
    logger = get_logger(__name__)
    scene_package = LaunchConfiguration(SCENE_PACKAGE_ARGUMENT).perform(context)
    if not scene_package:
        available = ", ".join(DEFAULT_SCENE_PACKAGE_CANDIDATES)
        raise RuntimeError(
            f"Launch argument '{SCENE_PACKAGE_ARGUMENT}' was empty. Pass a valid '{SCENE_PACKAGE_ARGUMENT}' "
            f"launch argument, for example '{SCENE_PACKAGE_ARGUMENT}:={available.split(', ')[0]}', or build/source "
            "your generated scene package first."
        )
    moveit_config_package = LaunchConfiguration(MOVEIT_CONFIG_PACKAGE_ARGUMENT).perform(context)
    planning_frame = LaunchConfiguration(PLANNING_FRAME_ARGUMENT).perform(context).strip()
    if not planning_frame:
        raise RuntimeError(
            f"Launch argument '{PLANNING_FRAME_ARGUMENT}' resolved to an empty frame after trimming whitespace."
        )
    run_share = get_package_share_directory(PACKAGE_NAME)
    resolve_scene_package_share_dir(scene_package)
    resolve_required_package_share_dir(
        moveit_config_package,
        "Build/source your selected MoveIt config package and pass it with "
        f"'{MOVEIT_CONFIG_PACKAGE_ARGUMENT}:=<package_name>' if it is not 'ur5_moveit_config'.",
    )
    try:
        scene_metadata = load_scene_environment(scene_package)
        gripper_controller_joints = resolve_gripper_controller_joints(scene_package, scene_metadata=scene_metadata)
        scene_workcell_context, scene_ee_id, scene_ee_link, scene_ee_grasp_frame = build_workcell_context_for_scene(
            scene_package, scene_metadata
        )
    except Exception as exc:
        scene_package_path = get_package_share_directory(scene_package)
        scene_yaml_path = os.path.join(scene_package_path, "environment.yaml")
        raise RuntimeError(
            "Failed while parsing scene metadata for grasp execution launch. "
            f"package='{scene_package}', file='environment.yaml', resolved_path='{scene_yaml_path}', args: "
            f"{SCENE_PACKAGE_ARGUMENT}='{scene_package}', "
            f"{MOVEIT_CONFIG_PACKAGE_ARGUMENT}='{moveit_config_package}', "
            f"{PLANNING_FRAME_ARGUMENT}='{planning_frame}'. "
            f"Original error: {exc}"
        ) from exc
    workcell_context_params_file = write_temp_yaml_params(scene_workcell_context, prefix="workcell_context_")
    logger.info(
        f"Generated workcell context for scene '{scene_package}': ee={scene_ee_id} "
        f"brand={scene_ee_id} moveit_link={scene_ee_link} "
        f"grasp_frame={scene_ee_grasp_frame} (file='{workcell_context_params_file}')"
    )

    scene_xacro_path = Path(get_package_share_directory(scene_package)) / "urdf" / "scene.urdf.xacro"
    scene_args = _extract_scene_xacro_args(scene_xacro_path)
    ur_robot_args = _extract_ur_robot_macro_params()

    initial_position_mappings = {}
    if "initial_positions_file" in ur_robot_args or "initial_positions_file" in scene_args:
        initial_position_path = os.path.join(run_share, "config", "start_positions.yaml")
        initial_position_mappings["initial_positions_file"] = initial_position_path

    if "mock_sensor_commands" in ur_robot_args:
        initial_position_mappings["mock_sensor_commands"] = "true"
    elif "fake_sensor_commands" in ur_robot_args:
        # Backward compatibility for older ur_description versions.
        initial_position_mappings["fake_sensor_commands"] = "true"


    try:
        robot_description_config = load_file(scene_package, "urdf/scene.urdf.xacro", initial_position_mappings)
        robot_description_semantic_config = load_file(scene_package, "urdf/arm_hand.srdf.xacro")
    except Exception as exc:
        raise RuntimeError(
            "Failed while loading robot description files for grasp execution launch. "
            f"package='{scene_package}', files='urdf/scene.urdf.xacro, urdf/arm_hand.srdf.xacro', args: "
            f"{SCENE_PACKAGE_ARGUMENT}='{scene_package}', "
            f"{MOVEIT_CONFIG_PACKAGE_ARGUMENT}='{moveit_config_package}', "
            f"{PLANNING_FRAME_ARGUMENT}='{planning_frame}'. "
            "Ensure both URDF/SRDF xacro files exist and are valid."
        ) from exc

    robot_description_semantic_config = _normalize_srdf_for_ur_base_inertia(
        robot_description_config,
        robot_description_semantic_config,
        logger,
    )

    robot_description = {"robot_description": robot_description_config}
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    try:
        kinematics_data = require_yaml_mapping(
            load_yaml(moveit_config_package, "config/kinematics.yaml"),
            "robot_description_kinematics",
            moveit_config_package,
            "config/kinematics.yaml",
        )
        joint_limits_yaml = require_yaml_mapping(
            load_yaml(moveit_config_package, "config/joint_limits.yaml"),
            "robot_description_planning",
            moveit_config_package,
            "config/joint_limits.yaml",
        )
        ompl_planning_yaml = require_yaml_mapping(
            load_yaml(moveit_config_package, "config/ompl_planning.yaml"),
            "ompl",
            moveit_config_package,
            "config/ompl_planning.yaml",
        )
        controllers_yaml = require_yaml_mapping(
            load_yaml(PACKAGE_NAME, "config/controllers.yaml"),
            "moveit_simple_controller_manager",
            PACKAGE_NAME,
            "config/controllers.yaml",
        )
        ros2_controllers_yaml = require_yaml_mapping(
            load_yaml(PACKAGE_NAME, "config/ur5_ros_controllers.yaml"),
            "controller_manager",
            PACKAGE_NAME,
            "config/ur5_ros_controllers.yaml",
        )
    except Exception as exc:
        raise RuntimeError(
            "Failed while parsing YAML configuration for grasp execution launch. "
            f"args: {SCENE_PACKAGE_ARGUMENT}='{scene_package}', "
            f"{MOVEIT_CONFIG_PACKAGE_ARGUMENT}='{moveit_config_package}', "
            f"{PLANNING_FRAME_ARGUMENT}='{planning_frame}'. "
            "Check YAML schemas in MoveIt and run_grasp_execution config files."
        ) from exc

    kinematics_yaml = {"robot_description_kinematics": kinematics_data}
    joint_limits = {"robot_description_planning": joint_limits_yaml}
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    scene_supports_gripper_controller = bool(
        gripper_controller_joints
        and scene_exposes_gripper_position_interfaces(robot_description_config, gripper_controller_joints)
    )

    align_gripper_controller_joints(
        controllers_yaml,
        ros2_controllers_yaml,
        gripper_controller_joints,
        scene_supports_gripper_controller,
    )
    ros2_controllers_params_file = write_temp_yaml_params(ros2_controllers_yaml)
    logger.info(
        f"Generated temporary ROS 2 controllers params file at '{ros2_controllers_params_file}'."
    )
    moveit_controller = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    grasp_execution_yaml = os.path.join(run_share, "config", "grasp_execution.yaml")
    sensors_yaml = os.path.join(run_share, "config", "sensors_3d.yaml")
    rviz_config_file = os.path.join(run_share, "config", "grasp_execution.rviz")

    grasp_execution_demo_node = Node(
        name="grasp_execution_node",
        package=PACKAGE_NAME,
        executable="demo_node",
        output="screen",
        prefix=PythonExpression(
            [
                "'xterm -e gdb --args' if '",
                LaunchConfiguration("debug"),
                "' == 'true' else ''",
            ]
        ),
        parameters=[
            grasp_execution_yaml,
            sensors_yaml,
            {"workcell_context": workcell_context_params_file},
            {"planning_frame": planning_frame, "octomap_frame": planning_frame},
            robot_description,
            robot_description_semantic,
            joint_limits,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controller,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description, robot_description_semantic],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output={"stdout": "screen", "stderr": "screen"},
        parameters=[ros2_controllers_params_file],
        remappings=[("~/robot_description", "/robot_description")],
    )

    joint_state_spawner = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    arm_spawner = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "ur5_arm_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    gripper_spawner = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "ur5_gripper_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    spawn_joint_state = TimerAction(period=2.0, actions=[joint_state_spawner])
    spawn_arm = TimerAction(period=2.5, actions=[arm_spawner])

    launch_actions = [
        robot_state_publisher,
        rviz_node,
        ros2_control_node,
        spawn_joint_state,
        spawn_arm,
        grasp_execution_demo_node,
    ]

    if scene_supports_gripper_controller:
        spawn_gripper = TimerAction(period=3.0, actions=[gripper_spawner])
        launch_actions.insert(-1, spawn_gripper)
    else:
        logger.warning(
            "Skipping ur5_gripper_controller spawner: scene metadata and robot description do not agree on "
            "gripper controller joints/interfaces; dummy gripper driver remains responsible for gripper actions."
        )

    return launch_actions


def generate_launch_description():
    debug_arg = DeclareLaunchArgument("debug", default_value="false", description="Launch in debug mode")
    scene_description = "Scene package containing urdf/scene.urdf.xacro and urdf/arm_hand.srdf.xacro"
    scene_arg_kwargs = {
        "description": scene_description,
        "default_value": DEFAULT_SCENE_PACKAGE if DEFAULT_SCENE_PACKAGE is not None else "ur5_3f_test",
    }

    return LaunchDescription(
        [
            debug_arg,
            DeclareLaunchArgument(SCENE_PACKAGE_ARGUMENT, **scene_arg_kwargs),
            DeclareLaunchArgument(
                MOVEIT_CONFIG_PACKAGE_ARGUMENT,
                default_value="ur5_moveit_config",
                description="MoveIt config package containing config/{kinematics,joint_limits,ompl_planning}.yaml",
            ),
            DeclareLaunchArgument(
                PLANNING_FRAME_ARGUMENT,
                default_value="world",
                description="Canonical planning/reference frame shared by grasp execution and octomap.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
