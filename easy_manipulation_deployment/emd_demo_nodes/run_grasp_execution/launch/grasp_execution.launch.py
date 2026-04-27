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
from launch.conditions import IfCondition
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
    if not gripper_controller_joints:
        return False, {
            "missing_robot_description_joints": [],
            "missing_command_interfaces": {},
            "missing_state_interfaces": {},
        }

    try:
        root = ET.fromstring(robot_description_xml)
    except ET.ParseError:
        return False, {
            "missing_robot_description_joints": list(gripper_controller_joints),
            "missing_command_interfaces": {},
            "missing_state_interfaces": {},
        }

    ros2_control_joint_interfaces = {}
    for ros2_control in root.findall(".//ros2_control"):
        for joint in ros2_control.findall("joint"):
            joint_name = (joint.get("name") or "").strip()
            if not joint_name:
                continue
            command_interfaces = {
                (interface.get("name") or "").strip()
                for interface in joint.findall("command_interface")
                if (interface.get("name") or "").strip()
            }
            state_interfaces = {
                (interface.get("name") or "").strip()
                for interface in joint.findall("state_interface")
                if (interface.get("name") or "").strip()
            }
            ros2_control_joint_interfaces[joint_name] = {
                "command_interfaces": command_interfaces,
                "state_interfaces": state_interfaces,
            }

    missing_robot_description_joints = []
    missing_command_interfaces = {}
    missing_state_interfaces = {}
    for joint_name in gripper_controller_joints:
        if not root.findall(f".//joint[@name='{joint_name}']"):
            missing_robot_description_joints.append(joint_name)
            continue

        interfaces = ros2_control_joint_interfaces.get(joint_name)
        if interfaces is None:
            missing_command_interfaces[joint_name] = ["position"]
            missing_state_interfaces[joint_name] = ["position", "velocity"]
            continue

        command_interfaces = interfaces["command_interfaces"]
        state_interfaces = interfaces["state_interfaces"]
        if "position" not in command_interfaces:
            missing_command_interfaces[joint_name] = ["position"]

        missing_joint_state_interfaces = [
            interface_name for interface_name in ("position", "velocity") if interface_name not in state_interfaces
        ]
        if missing_joint_state_interfaces:
            missing_state_interfaces[joint_name] = missing_joint_state_interfaces

    is_valid = (
        not missing_robot_description_joints
        and not missing_command_interfaces
        and not missing_state_interfaces
    )
    return is_valid, {
        "missing_robot_description_joints": missing_robot_description_joints,
        "missing_command_interfaces": missing_command_interfaces,
        "missing_state_interfaces": missing_state_interfaces,
    }


def validate_gripper_controller_consistency(
    robot_description_xml, ros2_controllers_yaml, gripper_controller_joints
):
    robot_description_valid, robot_description_details = scene_exposes_gripper_position_interfaces(
        robot_description_xml, gripper_controller_joints
    )

    gripper_ros_params = (
        ros2_controllers_yaml.get("ur5_gripper_controller", {}).get("ros__parameters", {})
        if isinstance(ros2_controllers_yaml, dict)
        else {}
    )
    configured_joints = gripper_ros_params.get("joints", [])
    if not isinstance(configured_joints, list):
        configured_joints = []
    missing_yaml_joints = [
        joint_name for joint_name in gripper_controller_joints if joint_name not in configured_joints
    ]

    yaml_list_valid = not missing_yaml_joints
    return robot_description_valid and yaml_list_valid, {
        "robot_description": robot_description_details,
        "missing_ros2_controller_joints": missing_yaml_joints,
    }


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

    if ee_name in {"suction", "airpick"}:
        return "suction_cup"
    if ee_brand in {"suction", "airpick"}:
        return "suction_cup"
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


def _extract_end_effector_link_candidates(end_effector):
    links = end_effector.get("links")
    if isinstance(links, dict):
        values = links.values()
    elif isinstance(links, list):
        values = links
    elif isinstance(links, str):
        values = [links]
    else:
        values = []

    candidates = []
    for value in values:
        normalized = _normalize_text(value)
        if normalized and normalized not in candidates:
            candidates.append(normalized)
    return candidates


def _derive_normalized_ee_family(end_effector):
    ee_name = _normalize_text(end_effector.get("name"))
    ee_brand = _normalize_text(end_effector.get("brand"))
    ee_type = _normalize_text(end_effector.get("ee_type"))
    ee_fingers = end_effector.get("attributes", {}).get("fingers")
    search_blob = " ".join((ee_name, ee_brand, ee_type))

    if any(marker in search_blob for marker in ("robotiq_2f", "robotiq_85")) or ee_fingers == 2:
        return "2f"
    if "robotiq_3f" in search_blob or ee_fingers == 3:
        return "3f"
    if any(marker in search_blob for marker in ("suction", "single_suction", "airpick")):
        return "suction"
    return "generic"


def _first_from_metadata_or_links(end_effector, metadata_keys, link_candidates):
    for key in metadata_keys:
        value = _normalize_text(end_effector.get(key))
        if value:
            return value
    for value in link_candidates:
        if value:
            return value
    return ""


def derive_workcell_grasp_frame(end_effector):
    ee_family = _derive_normalized_ee_family(end_effector)
    link_candidates = _extract_end_effector_link_candidates(end_effector)

    if ee_family == "2f":
        metadata_and_links = ("grasp_frame", "ee_palm", "base_link", "link")
        return _first_from_metadata_or_links(end_effector, metadata_and_links, link_candidates) or "tool0"

    if ee_family == "3f":
        for key in ("grasp_frame", "tcp_link", "physical_ee_link", "base_link", "link"):
            value = _normalize_text(end_effector.get(key))
            if value:
                return value
        for palm_frame in ("ee_palm", "palm"):
            if palm_frame in link_candidates:
                return palm_frame
        return "tool0"

    if ee_family == "suction":
        suction_metadata_keys = ("suction_cup_link",)
        suction_link_candidates = [
            candidate
            for candidate in link_candidates
            if candidate == "suction_cup_link" or "suction_cup" in candidate
        ]
        suction_frame = _first_from_metadata_or_links(end_effector, suction_metadata_keys, suction_link_candidates)
        if suction_frame:
            return suction_frame

        tcp_keys = ("grasp_frame", "tcp_link", "physical_ee_link", "base_link", "link")
        tcp_link_candidates = [
            candidate
            for candidate in link_candidates
            if "tcp" in candidate or "tool_center_point" in candidate
        ]
        return _first_from_metadata_or_links(end_effector, tcp_keys, tcp_link_candidates) or "tool0"

    for key in ("grasp_frame", "tcp_link", "physical_ee_link", "base_link", "link"):
        value = _normalize_text(end_effector.get(key))
        if value:
            return value
    return ""


def build_workcell_context_for_scene(scene_package, scene_metadata, logger=None):
    end_effector = scene_metadata.get("end_effector", {})
    if end_effector is None:
        end_effector = {}
    if not isinstance(end_effector, dict):
        raise RuntimeError(
            f"Invalid scene metadata in package '{scene_package}': 'end_effector' must be a YAML mapping (dict), "
            f"got {type(end_effector).__name__}."
        )

    normalized_ee_name = _normalize_text(end_effector.get("name"))
    normalized_ee_brand = _normalize_text(end_effector.get("brand"))
    normalized_ee_type = _normalize_text(end_effector.get("ee_type"))
    ee_id = derive_planner_end_effector_id(end_effector)
    ee_link = derive_workcell_end_effector_link(end_effector)
    ee_grasp_frame = derive_workcell_grasp_frame(end_effector) or ee_link

    if ee_id == "ur_tool0":
        if logger is None:
            logger = get_logger(__name__)
        logger.warning(
            "Falling back to arm-only workcell end effector classification for scene "
            f"'{scene_package}' (normalized metadata: name='{normalized_ee_name}', "
            f"brand='{normalized_ee_brand}', ee_type='{normalized_ee_type}'); "
            "using fallback link/frame 'tool0' and planner ee id 'ur_tool0'."
        )
    return build_workcell_context(ee_id, ee_link, ee_grasp_frame), ee_id, ee_link, ee_grasp_frame


def build_workcell_context(ee_id, ee_link, ee_grasp_frame):
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
    }


def validate_and_normalize_workcell_end_effector_frames(scene_metadata, ee_id, ee_link, ee_grasp_frame, link_names, logger):
    end_effector = scene_metadata.get("end_effector", {}) if isinstance(scene_metadata, dict) else {}
    if not isinstance(end_effector, dict):
        end_effector = {}

    expected_3f_links = {"palm", "finger_1_link_0", "finger_2_link_0", "finger_middle_link_0"}
    metadata_declares_3f = ee_id == "robotiq_3f" or resolve_gripper_controller_joints("", scene_metadata) == GRIPPER_CONTROLLER_JOINTS_BY_END_EFFECTOR["robotiq_3f"]
    urdf_has_3f_links = bool(expected_3f_links.intersection(link_names))

    if metadata_declares_3f and not urdf_has_3f_links:
        logger.warning(
            "Scene metadata declares Robotiq 3F, but the URDF does not expose 3F links; falling back to arm-only "
            "workcell context (ee='ur_tool0', link='tool0', grasp_frame='tool0')."
        )
        return "ur_tool0", "tool0", "tool0"

    missing_targets = [frame for frame in (ee_link, ee_grasp_frame) if frame not in link_names]
    if missing_targets:
        logger.warning(
            "Derived workcell frames are not present in the scene URDF links "
            f"({', '.join(missing_targets)}); falling back to arm-only context link/frame 'tool0'."
        )
        return "ur_tool0", "tool0", "tool0"

    return ee_id, ee_link, ee_grasp_frame


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


def _extract_fixed_joint_pairs_from_urdf(robot_description_xml):
    try:
        root = ET.fromstring(robot_description_xml)
    except ET.ParseError:
        return set()

    fixed_pairs = set()
    for joint in root.findall("joint"):
        if (joint.attrib.get("type") or "").strip().lower() != "fixed":
            continue
        parent = joint.find("parent")
        child = joint.find("child")
        parent_link = (parent.attrib.get("link") if parent is not None else "") or ""
        child_link = (child.attrib.get("link") if child is not None else "") or ""
        parent_link = parent_link.strip()
        child_link = child_link.strip()
        if parent_link and child_link:
            fixed_pairs.add(tuple(sorted((parent_link, child_link))))
    return fixed_pairs


def _extract_disable_collision_pairs_from_srdf_root(srdf_root):
    existing_pairs = set()
    for element in srdf_root.findall("disable_collisions"):
        link1 = element.attrib.get("link1", "")
        link2 = element.attrib.get("link2", "")
        if link1 and link2:
            existing_pairs.add(tuple(sorted((link1, link2))))
    return existing_pairs


def _is_camera_link_name(link_name):
    normalized = _normalize_text(link_name)
    return any(token in normalized for token in ("camera", "realsense", "d435"))


def _is_environment_surface_link(link_name):
    normalized = _normalize_text(link_name)
    return any(token in normalized for token in ("table", "workbench"))


def _compute_conservative_srdf_disable_collision_injections(robot_description_xml, srdf_xml):
    urdf_link_names = _extract_link_names_from_urdf(robot_description_xml)
    if not urdf_link_names:
        return []

    try:
        srdf_root = ET.fromstring(srdf_xml)
    except ET.ParseError:
        return []
    existing_pairs = _extract_disable_collision_pairs_from_srdf_root(srdf_root)

    requested_pairs = []

    # Keep existing UR base-link inertia adjacency compatibility rules.
    if {"base_link_inertia", "shoulder_link"}.issubset(urdf_link_names):
        requested_pairs.extend(
            [
                ("base_link", "base_link_inertia", "Adjacent"),
                ("base_link_inertia", "shoulder_link", "Adjacent"),
            ]
        )

    # Known UR false-positive: forearm_link vs wrist_2_link.
    if {"forearm_link", "wrist_2_link"}.issubset(urdf_link_names):
        requested_pairs.append(("forearm_link", "wrist_2_link", "Never"))

    # Camera mount links can be fixed to the wrist in some scenes; only add when
    # the URDF explicitly models a fixed wrist<->camera connection.
    wrist_links = {"wrist_1_link", "wrist_2_link", "wrist_3_link"}
    for pair in _extract_fixed_joint_pairs_from_urdf(robot_description_xml):
        link1, link2 = pair
        if _is_environment_surface_link(link1) or _is_environment_surface_link(link2):
            continue
        if (
            ((link1 in wrist_links) and _is_camera_link_name(link2))
            or ((link2 in wrist_links) and _is_camera_link_name(link1))
        ):
            requested_pairs.append((link1, link2, "Adjacent"))

    injected_pairs = []
    requested_pairs = sorted(set(requested_pairs), key=lambda item: (item[0], item[1], item[2]))
    for link1, link2, reason in requested_pairs:
        pair_key = tuple(sorted((link1, link2)))
        if pair_key in existing_pairs:
            continue
        if _is_environment_surface_link(link1) or _is_environment_surface_link(link2):
            continue
        injected_pairs.append((link1, link2, reason))
    return injected_pairs


def _normalize_srdf_for_ur_base_inertia(robot_description_xml, srdf_xml, logger):
    try:
        root = ET.fromstring(srdf_xml)
    except ET.ParseError as exc:
        logger.warning(
            f"Skipping UR base_link_inertia SRDF normalization due to parse error: {exc}"
        )
        return srdf_xml

    pairs_to_inject = _compute_conservative_srdf_disable_collision_injections(robot_description_xml, srdf_xml)
    if not pairs_to_inject:
        return srdf_xml

    injected_pairs = []
    for link1, link2, reason in pairs_to_inject:
        ET.SubElement(root, "disable_collisions", link1=link1, link2=link2, reason=reason)
        injected_pairs.append(f"{link1}<->{link2} ({reason})")

    logger.info(
        "Injected conservative SRDF disable_collisions pairs for UR scene normalization: "
        + ", ".join(injected_pairs)
    )
    return ET.tostring(root, encoding="unicode")


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

    scene_workcell_context, scene_ee_id, scene_ee_link, scene_ee_grasp_frame = build_workcell_context_for_scene(
        scene_package, scene_metadata, logger=logger
    )
    scene_link_names = _extract_link_names_from_urdf(robot_description_config)
    scene_ee_id, scene_ee_link, scene_ee_grasp_frame = validate_and_normalize_workcell_end_effector_frames(
        scene_metadata,
        scene_ee_id,
        scene_ee_link,
        scene_ee_grasp_frame,
        scene_link_names,
        logger,
    )
    scene_workcell_context = build_workcell_context(scene_ee_id, scene_ee_link, scene_ee_grasp_frame)
    workcell_context_params_file = write_temp_yaml_params(scene_workcell_context, prefix="workcell_context_")
    logger.info(
        f"Generated workcell context for scene '{scene_package}': ee={scene_ee_id} "
        f"brand={scene_ee_id} moveit_link={scene_ee_link} "
        f"grasp_frame={scene_ee_grasp_frame} (file='{workcell_context_params_file}')"
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

    scene_supports_gripper_controller = False
    gripper_validation_details = {
        "robot_description": {
            "missing_robot_description_joints": [],
            "missing_command_interfaces": {},
            "missing_state_interfaces": {},
        },
        "missing_ros2_controller_joints": [],
    }
    if gripper_controller_joints:
        scene_supports_gripper_controller, gripper_validation_details = validate_gripper_controller_consistency(
            robot_description_config,
            ros2_controllers_yaml,
            gripper_controller_joints,
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

    sensors_3d_config = require_yaml_mapping(
        load_yaml(PACKAGE_NAME, "config/sensors_3d.yaml"),
        "grasp_execution_node",
        PACKAGE_NAME,
        "config/sensors_3d.yaml",
    )
    node_section = require_yaml_mapping(
        sensors_3d_config.get("grasp_execution_node"),
        "grasp_execution_node",
        PACKAGE_NAME,
        "config/sensors_3d.yaml",
    )
    sensor_ros_params = require_yaml_mapping(
        node_section.get("ros__parameters"),
        "ros__parameters",
        PACKAGE_NAME,
        "config/sensors_3d.yaml",
    )
    sensor_ros_params["octomap_frame"] = planning_frame
    sensors_yaml = write_temp_yaml_params(sensors_3d_config, prefix="sensors_3d_")
    logger.info(
        f"Generated sanitized sensors params file at '{sensors_yaml}' with octomap_frame='{planning_frame}'."
    )

    grasp_execution_yaml = os.path.join(run_share, "config", "grasp_execution.yaml")
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
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
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
        details = gripper_validation_details["robot_description"]
        reason_parts = []
        if details["missing_robot_description_joints"]:
            reason_parts.append(
                "missing joints in robot_description="
                f"{details['missing_robot_description_joints']}"
            )
        if details["missing_command_interfaces"]:
            reason_parts.append(
                "missing ros2_control command interfaces="
                f"{details['missing_command_interfaces']}"
            )
        if details["missing_state_interfaces"]:
            reason_parts.append(
                "missing ros2_control state interfaces="
                f"{details['missing_state_interfaces']}"
            )
        if gripper_validation_details["missing_ros2_controller_joints"]:
            reason_parts.append(
                "missing joints in ur5_ros_controllers.yaml ur5_gripper_controller.ros__parameters.joints="
                f"{gripper_validation_details['missing_ros2_controller_joints']}"
            )
        reasons = "; ".join(reason_parts) if reason_parts else "no matching gripper controller joints"
        logger.warning(
            "Skipping ur5_gripper_controller spawner: "
            f"{reasons}; dummy gripper driver remains responsible for gripper actions."
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
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz for grasp execution visualization.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
