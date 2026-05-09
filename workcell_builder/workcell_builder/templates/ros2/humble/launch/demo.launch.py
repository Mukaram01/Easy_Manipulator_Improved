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
import re
import subprocess
import xml.etree.ElementTree as ET
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

scene_pkg = 'scene_name'
robot_base_link = 'base_link_name'
robot_moveit_pkg = 'moveit_config_name'


def load_xacro(package_name, rel_path, mappings=None):
    pkg_share = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_share, rel_path)
    if not os.path.exists(abs_path):
        raise FileNotFoundError(f"Missing file: {abs_path}")

    effective_mappings = dict(mappings or {})
    while True:
        cmd = ["xacro", abs_path]
        cmd.extend(f"{key}:={value}" for key, value in effective_mappings.items())
        try:
            completed = subprocess.run(
                cmd, check=True, text=True, capture_output=True, timeout=30
            )
            return completed.stdout
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(
                f"Timed out while expanding xacro file '{abs_path}'. "
                "This usually means a dependency package is missing from the sourced "
                "workspace or the xacro include graph is blocking."
            ) from exc
        except subprocess.CalledProcessError as exc:
            stderr = exc.stderr or ""
            match = re.search(r'Invalid parameter "([^"]+)"', stderr)
            if not match:
                raise RuntimeError(
                    f"Failed to expand xacro file '{abs_path}':\n{stderr}"
                ) from exc
            invalid_param = match.group(1)
            if invalid_param not in effective_mappings:
                raise RuntimeError(
                    f"Failed to expand xacro file '{abs_path}':\n{stderr}"
                ) from exc
            effective_mappings.pop(invalid_param)


def load_yaml(package_name, rel_path):
    pkg_share = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_share, rel_path)
    if not os.path.exists(abs_path):
        return {}
    with open(abs_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def extract_end_effector_metadata(environment_config):
    end_effector = environment_config.get("end_effector", {}) if isinstance(environment_config, dict) else {}
    if not isinstance(end_effector, dict):
        end_effector = {}

    gripper_type = end_effector.get("gripper_type") or end_effector.get("ee_type") or ""
    planner_id = end_effector.get("planner_id") or end_effector.get("brand") or ""
    grasp_frame = end_effector.get("grasp_frame") or end_effector.get("tcp_link") or end_effector.get("base_link") or ""
    tcp_link = end_effector.get("tcp_link") or end_effector.get("grasp_frame") or end_effector.get("base_link") or ""

    spawn_gripper_controller = end_effector.get("spawn_gripper_controller")
    if spawn_gripper_controller is None:
        spawn_gripper_controller = gripper_type == "finger"

    finger_count = end_effector.get("finger_count")
    if finger_count is None and gripper_type == "finger":
        attributes = end_effector.get("attributes", {})
        if isinstance(attributes, dict):
            finger_count = attributes.get("fingers")

    return {
        "planner_id": planner_id,
        "grasp_frame": grasp_frame,
        "tcp_link": tcp_link,
        "gripper_type": gripper_type,
        "spawn_gripper_controller": bool(spawn_gripper_controller),
        "finger_count": finger_count,
    }


def _normalize_ros_param_types(value):
    if isinstance(value, dict):
        return {str(key): _normalize_ros_param_types(item) for key, item in value.items()}

    if isinstance(value, tuple):
        return [_normalize_ros_param_types(item) for item in value]

    if isinstance(value, list):
        return [_normalize_ros_param_types(item) for item in value]

    return value


_DROP_PARAM = object()


def _sanitize_ros_param_types(value):
    if isinstance(value, dict):
        sanitized = {}
        for key, item in value.items():
            sanitized_item = _sanitize_ros_param_types(item)
            if sanitized_item is _DROP_PARAM:
                continue
            sanitized[str(key)] = sanitized_item
        return sanitized if sanitized else _DROP_PARAM

    if isinstance(value, tuple):
        value = list(value)

    if isinstance(value, list):
        sanitized_items = []
        for item in value:
            sanitized_item = _sanitize_ros_param_types(item)
            if sanitized_item is not _DROP_PARAM:
                sanitized_items.append(sanitized_item)
        return sanitized_items if sanitized_items else _DROP_PARAM

    if value is None:
        return _DROP_PARAM

    if isinstance(value, (bool, int, float, str, bytes)):
        return value

    return _DROP_PARAM


def _param_dict(value):
    sanitized = _sanitize_ros_param_types(value)
    return {} if sanitized is _DROP_PARAM else sanitized


def _param_list(*values):
    result = []
    for value in values:
        sanitized = _sanitize_ros_param_types(value)
        if sanitized is _DROP_PARAM:
            continue
        if isinstance(sanitized, dict) and not sanitized:
            continue
        result.append(sanitized)
    return result


def _validate_ros_param_types(value, path="root"):
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise TypeError(f"Invalid ROS param type at {path}: {type(key).__name__} -> {key!r}")
            _validate_ros_param_types(item, f"{path}.{key}")
        return

    if isinstance(value, list):
        for index, item in enumerate(value):
            _validate_ros_param_types(item, f"{path}[{index}]")
        return

    if isinstance(value, (bool, int, float, str, bytes)):
        return

    raise TypeError(f"Invalid ROS param type at {path}: {type(value).__name__} -> {value!r}")


UR_ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def _collect_movable_urdf_joints(urdf_root):
    movable = []
    mimic_joints = set()
    for joint in urdf_root.findall(".//joint"):
        name = joint.get("name")
        joint_type = joint.get("type")
        if not name or joint_type in (None, "fixed"):
            continue
        if joint.find("mimic") is not None:
            mimic_joints.add(name)
            continue
        movable.append(name)
    return movable, mimic_joints


def _derive_chain_joints_from_urdf(robot_description_config, base_link, tip_link):
    urdf_root = ET.fromstring(robot_description_config)
    edges = {}
    for joint in urdf_root.findall(".//joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        name = joint.get("name")
        joint_type = joint.get("type")
        if parent is None or child is None or not name:
            continue
        edges[parent.get("link")] = (child.get("link"), name, joint_type, joint.find("mimic") is not None)

    chain_joints = []
    current = base_link
    seen = set()
    while current != tip_link and current not in seen and current in edges:
        seen.add(current)
        child, joint_name, joint_type, is_mimic = edges[current]
        if joint_type not in (None, "fixed") and not is_mimic:
            chain_joints.append(joint_name)
        current = child

    if current != tip_link or not chain_joints:
        return []
    return chain_joints


def _extract_controller_joints(robot_description_semantic_config, robot_description_config):
    try:
        srdf_root = ET.fromstring(robot_description_semantic_config)
        urdf_root = ET.fromstring(robot_description_config)
    except ET.ParseError as exc:
        raise RuntimeError("Failed to parse SRDF/URDF for controller joint extraction") from exc

    group_names = []
    groups_with_explicit = []
    groups_with_chain = []
    preferred_group_names = ["manipulator", "arm", "ur_manipulator"]

    for group in srdf_root.findall(".//group"):
        group_name = group.get("name") or "arm"
        group_names.append(group_name)
        joints = [joint.get("name") for joint in group.findall("joint") if joint.get("name")]
        if joints:
            groups_with_explicit.append((group_name, joints))
            if group_name in preferred_group_names:
                return group_name, joints
        for chain in group.findall("chain"):
            base_link = chain.get("base_link")
            tip_link = chain.get("tip_link")
            if base_link and tip_link:
                groups_with_chain.append((group_name, base_link, tip_link))

    if groups_with_explicit:
        return groups_with_explicit[0]

    movable_joints, _ = _collect_movable_urdf_joints(urdf_root)
    if all(joint in set(movable_joints) for joint in UR_ARM_JOINTS):
        return "manipulator", UR_ARM_JOINTS

    for group_name, base_link, tip_link in groups_with_chain:
        chain_joints = _derive_chain_joints_from_urdf(robot_description_config, base_link, tip_link)
        if chain_joints:
            return group_name, chain_joints

    non_gripper = [joint for joint in movable_joints if "gripper" not in joint and "finger" not in joint]
    fallback_joints = non_gripper[:6]
    if len(fallback_joints) == 6:
        print("[workcell_builder] WARNING: using URDF-only fallback arm joints for preview/fake hardware")
        return "manipulator", fallback_joints

    raise RuntimeError(
        "No controller joints were found in robot_description_semantic. "
        f"groups={group_names}, explicit={bool(groups_with_explicit)}, chain={bool(groups_with_chain)}, "
        f"first_movable_urdf_joints={movable_joints[:20]}. "
        "Regenerate SRDF with explicit manipulator joint entries or configure robot capability arm_joints."
    )


def _validate_joint_state_configuration(robot_description_config, controller_joints):
    if not controller_joints:
        raise RuntimeError("Fake controller joints list is empty; unable to configure joint state publishing")

    try:
        urdf_root = ET.fromstring(robot_description_config)
    except ET.ParseError as exc:
        raise RuntimeError("Failed to parse robot_description while validating joint states") from exc

    movable_joint_names = []
    for joint in urdf_root.findall(".//joint"):
        joint_name = joint.get("name")
        joint_type = joint.get("type")
        if not joint_name or joint_type in (None, "fixed"):
            continue
        movable_joint_names.append(joint_name)

    if len(movable_joint_names) != len(set(movable_joint_names)):
        raise RuntimeError("Duplicate movable joint names were found in robot_description")

    missing = [joint for joint in controller_joints if joint not in set(movable_joint_names)]
    if missing:
        raise RuntimeError(
            "Controller joints are not present in robot_description: "
            + ", ".join(missing)
        )






def _derive_controller_configs(scene_name, controller_group_name, controller_joints, robot_description_config, end_effector_metadata):
    urdf_root = ET.fromstring(robot_description_config)
    movable_joints = {
        j.get("name") for j in urdf_root.findall(".//joint")
        if j.get("name") and j.get("type") not in (None, "fixed")
    }

    robot_name = (scene_name or controller_group_name or "robot").lower()
    detected_ur_arm = [joint for joint in UR_ARM_JOINTS if joint in movable_joints]
    arm_joints = detected_ur_arm if len(detected_ur_arm) == len(UR_ARM_JOINTS) else [
        joint for joint in controller_joints if joint in movable_joints
    ]

    if not arm_joints:
        raise RuntimeError("Unable to derive movable arm joints for MoveIt controller configuration")

    if robot_name.startswith("ur") and len(detected_ur_arm) == len(UR_ARM_JOINTS):
        arm_controller_name = f"{robot_name}_arm_controller"
    else:
        arm_controller_name = f"{robot_name}_arm_controller"

    gripper_joints = [
        joint for joint in movable_joints
        if joint.startswith("gripper_")
        and joint not in set(arm_joints)
        and joint != "gripper_base_joint"
    ]
    gripper_joints = [
        joint for joint in gripper_joints
        if "inner_knuckle" not in joint and "finger_tip" not in joint
    ]
    preferred = ["gripper_finger1_joint", "gripper_finger2_joint"]
    preferred_gripper = [joint for joint in preferred if joint in gripper_joints]
    if preferred_gripper:
        gripper_joints = preferred_gripper
    else:
        gripper_joints.sort()

    controllers = {
        "controller_names": [arm_controller_name],
        arm_controller_name: {
            "action_ns": "follow_joint_trajectory",
            "type": "FollowJointTrajectory",
            "default": True,
            "joints": arm_joints,
        },
    }

    should_emit_gripper_controller = (
        bool(end_effector_metadata.get("spawn_gripper_controller"))
        or end_effector_metadata.get("gripper_type") == "finger"
        or bool(gripper_joints)
    )
    if should_emit_gripper_controller and gripper_joints:
        gripper_controller_name = (f"{robot_name}_gripper_controller" if robot_name.startswith("ur") else f"{robot_name}_gripper_controller")
        controllers["controller_names"].append(gripper_controller_name)
        controllers[gripper_controller_name] = {
            "action_ns": "follow_joint_trajectory",
            "type": "FollowJointTrajectory",
            "default": False,
            "joints": gripper_joints,
        }

    return controllers

def _write_robot_description_file(scene_name, robot_description_config):
    path = os.path.join(
        tempfile.gettempdir(),
        f"{scene_name}_robot_description.urdf",
    )
    with open(path, "w", encoding="utf-8") as file:
        file.write(robot_description_config)
    return path


def _launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_octomap = LaunchConfiguration("enable_octomap")
    octomap_resolution = LaunchConfiguration("octomap_resolution")
    octomap_pointcloud_topic = LaunchConfiguration("octomap_pointcloud_topic")
    octomap_max_range = LaunchConfiguration("octomap_max_range")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    # Keep joint states isolated per-scene to avoid global topic collisions.
    joint_states_topic = f"/{scene_pkg}/joint_states"

    robot_description_config = load_xacro(
        scene_pkg,
        "urdf/scene.urdf.xacro",
        mappings={"use_fake_hardware": use_fake_hardware.perform(context)},
    )
    robot_description = {"robot_description": robot_description_config}

    robot_description_file = _write_robot_description_file(
        scene_pkg,
        robot_description_config,
    )

    robot_description_semantic_config = load_xacro(scene_pkg, "urdf/arm_hand.srdf.xacro")
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(robot_moveit_pkg, "config/kinematics.yaml")
    robot_description_kinematics = (
        {"robot_description_kinematics": kinematics_yaml} if kinematics_yaml else {}
    )

    ompl_planning_yaml = load_yaml(robot_moveit_pkg, "config/ompl_planning.yaml")

    planning_pipelines_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
    }

    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": " ".join([
                "default_planner_request_adapters/AddTimeOptimalParameterization",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
                "default_planner_request_adapters/FixStartStatePathConstraints",
            ]),
            "start_state_max_bounds_error": 0.1,
        }
    }
    if isinstance(ompl_planning_yaml, dict):
        ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    controller_group_name, controller_joints = _extract_controller_joints(
        robot_description_semantic_config, robot_description_config
    )
    if not controller_joints:
        raise RuntimeError(
            "No joints were found in the selected SRDF group. "
            "The generated fake controller cannot be created with an empty joints list."
        )
    _validate_joint_state_configuration(robot_description_config, controller_joints)
    environment_config = load_yaml(scene_pkg, "environment.yaml")
    end_effector_metadata = extract_end_effector_metadata(environment_config)
    controller_configs = _derive_controller_configs(
        scene_pkg,
        controller_group_name,
        controller_joints,
        robot_description_config,
        end_effector_metadata,
    )

    moveit_controller_manager = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    }

    moveit_simple_controller_manager = {
        "moveit_simple_controller_manager": controller_configs
    }

    trajectory_execution = {
        "allow_trajectory_execution": False,
        "moveit_manage_controllers": False,
    }

    planning_scene_monitor_params = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    octomap_enabled = enable_octomap.perform(context).lower() == "true"
    octomap_config = {}
    octomap_launch_message = None
    if octomap_enabled:
        octomap_config = {
            "octomap_resolution": float(octomap_resolution.perform(context)),
            "sensors": ["realsense_pointcloud"],
            "realsense_pointcloud": {
                "sensor_plugin": "occupancy_map_monitor/PointCloudOctomapUpdater",
                "point_cloud_topic": octomap_pointcloud_topic.perform(context),
                "max_range": float(octomap_max_range.perform(context)),
                "point_subsample": 1,
                "padding_offset": 0.1,
                "padding_scale": 1.0,
                "max_update_rate": 1.0,
                "filtered_cloud_topic": "filtered_cloud",
            },
        }
        octomap_launch_message = LogInfo(
            msg=(
                "Octomap occupancy monitor enabled with PointCloudOctomapUpdater "
                f"on topic '{octomap_pointcloud_topic.perform(context)}'."
            )
        )
    else:
        octomap_launch_message = LogInfo(
            msg="Octomap occupancy monitor disabled (enable_octomap:=false)."
        )

    try:
        validated_use_sim_time = _param_dict(_normalize_ros_param_types({"use_sim_time": use_sim_time.perform(context).lower() == "true"}))
        validated_robot_description = _param_dict(_normalize_ros_param_types(robot_description))
        validated_robot_description_semantic = _param_dict(_normalize_ros_param_types(robot_description_semantic))
        validated_robot_description_kinematics = _param_dict(_normalize_ros_param_types(robot_description_kinematics))
        validated_planning_pipelines_config = _param_dict(_normalize_ros_param_types(planning_pipelines_config))
        validated_ompl_planning_pipeline_config = _param_dict(_normalize_ros_param_types(ompl_planning_pipeline_config))
        validated_planning_scene_monitor_params = _param_dict(_normalize_ros_param_types(planning_scene_monitor_params))
        validated_octomap_config = _param_dict(_normalize_ros_param_types(octomap_config))
        validated_trajectory_execution = _param_dict(_normalize_ros_param_types(trajectory_execution))
        validated_moveit_controller_manager = _param_dict(_normalize_ros_param_types(moveit_controller_manager))
        validated_moveit_simple_controller_manager = _param_dict(_normalize_ros_param_types(moveit_simple_controller_manager))
        validated_end_effector_metadata = _param_dict(_normalize_ros_param_types({
            "workcell_end_effector_metadata": end_effector_metadata
        }))

        _validate_ros_param_types(validated_use_sim_time, "use_sim_time")
        _validate_ros_param_types(validated_robot_description, "robot_description")
        _validate_ros_param_types(validated_robot_description_semantic, "robot_description_semantic")
        _validate_ros_param_types(validated_robot_description_kinematics, "robot_description_kinematics")
        _validate_ros_param_types(validated_planning_pipelines_config, "planning_pipelines_config")
        _validate_ros_param_types(validated_ompl_planning_pipeline_config, "ompl_planning_pipeline_config")
        _validate_ros_param_types(validated_planning_scene_monitor_params, "planning_scene_monitor_params")
        _validate_ros_param_types(validated_octomap_config, "octomap_config")
        _validate_ros_param_types(validated_trajectory_execution, "trajectory_execution")
        _validate_ros_param_types(validated_moveit_controller_manager, "moveit_controller_manager")
        _validate_ros_param_types(validated_moveit_simple_controller_manager, "moveit_simple_controller_manager")
        _validate_ros_param_types(validated_end_effector_metadata, "end_effector_metadata")
    except TypeError as exc:
        raise TypeError(f"{scene_pkg} demo.launch parameter validation failed: {exc}") from exc

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "world",
            "--child-frame-id", robot_base_link,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=f"{scene_pkg}_robot_state_publisher",
        output="screen",
        parameters=_param_list(validated_use_sim_time, validated_robot_description),
        remappings=[
            ("joint_states", joint_states_topic),
            ("/joint_states", joint_states_topic),
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name=f"{scene_pkg}_joint_state_publisher",
        output="screen",
        arguments=[robot_description_file],
        parameters=_param_list(
            validated_use_sim_time,
        ),
        remappings=[
            ("joint_states", joint_states_topic),
            ("/joint_states", joint_states_topic),
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=_param_list(
            validated_use_sim_time,
            validated_robot_description,
            validated_robot_description_semantic,
            validated_robot_description_kinematics,
            validated_planning_pipelines_config,
            validated_ompl_planning_pipeline_config,
            validated_planning_scene_monitor_params,
            validated_octomap_config,
            validated_trajectory_execution,
            validated_moveit_controller_manager,
            validated_moveit_simple_controller_manager,
            validated_end_effector_metadata,
        ),
        remappings=[
            ("joint_states", joint_states_topic),
            ("/joint_states", joint_states_topic),
        ],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(scene_pkg), "launch", "demo.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=_param_list(
            validated_use_sim_time,
            validated_robot_description,
            validated_robot_description_semantic,
            validated_robot_description_kinematics,
        ),
        remappings=[
            ("joint_states", joint_states_topic),
            ("/joint_states", joint_states_topic),
        ],
    )

    return [
        octomap_launch_message,
        static_tf,
        robot_state_publisher,
        joint_state_publisher,
        move_group,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "enable_octomap",
            default_value="true",
            description=(
                "Enable MoveIt octomap integration. Defaults to true with a "
                "PointCloudOctomapUpdater config suitable for RealSense D435i."
            ),
        ),
        DeclareLaunchArgument(
            "octomap_resolution",
            default_value="0.1",
            description="Octomap voxel resolution in meters.",
        ),
        DeclareLaunchArgument(
            "octomap_pointcloud_topic",
            default_value="/camera/camera/depth/color/points",
            description="Point cloud topic consumed by MoveIt octomap updater.",
        ),
        DeclareLaunchArgument(
            "octomap_max_range",
            default_value="2.5",
            description="Maximum point cloud range (m) used for octomap updates.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description=(
                "Use simulated (fake) hardware instead of a real robot. "
                "Default: true (safe for development/testing without a physical robot). "
                "Set to false only when connected to a real robot via ur_robot_driver "
                "and after setting the robot_ip in the driver launch."
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
