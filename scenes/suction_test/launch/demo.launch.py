#!/usr/bin/env python3
import math
import os
import tempfile
import yaml
import re
import subprocess

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

scene_pkg = "suction_test"
robot_base_link = "base_link"
robot_moveit_pkg = "ur5_moveit_config"

CANONICAL_LAYOUT_REL_PATH = "layout/workcell_studio_layout.yaml"
CANONICAL_LAYOUT_SCHEMA = "workcell_studio_layout/v1"
REQUIRED_CANONICAL_XACRO_MAPPINGS = {
    "table_world_xyz",
    "table_world_rpy",
    "camera_world_xyz",
    "camera_world_rpy",
}


def _format_xacro_vector(values):
    return " ".join(format(value, ".17g") for value in values)


def _canonical_physical_role(item):
    role = str(item.get("role") or "").strip().lower()
    category = str(item.get("category") or "").strip().lower()
    item_type = str(item.get("type") or "").strip().lower()
    if role == "support_surface" or item_type in {"table", "workbench"}:
        return "support_surface"
    if role == "camera" or category == "camera" or item_type in {"camera", "realsense", "rgbd"}:
        return "camera"
    return ""


def load_canonical_layout_poses(package_name=scene_pkg, layout_path=None):
    """Resolve unique physical owners by semantics, independent of scene-specific ids."""
    if layout_path is None:
        layout_path = os.path.join(
            get_package_share_directory(package_name),
            CANONICAL_LAYOUT_REL_PATH,
        )
    try:
        with open(layout_path, "r", encoding="utf-8") as file:
            layout = yaml.safe_load(file)
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"Cannot load canonical Workcell Studio layout '{layout_path}': {exc}") from exc
    if not isinstance(layout, dict) or layout.get("schema_version") != CANONICAL_LAYOUT_SCHEMA:
        raise RuntimeError(
            f"Canonical Workcell Studio layout '{layout_path}' must use "
            f"schema_version '{CANONICAL_LAYOUT_SCHEMA}'."
        )
    items = layout.get("items")
    if not isinstance(items, list):
        raise RuntimeError(f"Canonical Workcell Studio layout '{layout_path}' must contain an items list.")

    resolved = {}
    for required_role in ("support_surface", "camera"):
        matches = [item for item in items if isinstance(item, dict) and _canonical_physical_role(item) == required_role]
        if len(matches) != 1:
            ids = [str(item.get("id") or "<missing-id>") for item in matches]
            raise RuntimeError(
                f"Canonical Workcell Studio layout '{layout_path}' must contain exactly one "
                f"physical {required_role} owner; found {len(matches)}: {ids}."
            )
        item = matches[0]
        pose = item.get("pose")
        if not isinstance(pose, dict):
            raise RuntimeError(f"Canonical {required_role} owner '{item.get('id')}' must contain pose.xyz and pose.rpy.")
        validated = {"id": str(item.get("id") or "")}
        for vector_name in ("xyz", "rpy"):
            vector = pose.get(vector_name)
            if (
                not isinstance(vector, (list, tuple))
                or len(vector) != 3
                or any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) for value in vector)
            ):
                raise RuntimeError(
                    f"Canonical {required_role} owner '{item.get('id')}' pose.{vector_name} "
                    "must contain exactly 3 finite numbers."
                )
            validated[vector_name] = tuple(float(value) for value in vector)
        resolved[required_role] = validated
    return resolved


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
            completed = subprocess.run(cmd, check=True, text=True, capture_output=True, timeout=30)
            return completed.stdout
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(
                f"Timed out while expanding xacro file '{abs_path}'. "
                "This usually means a dependency package is missing from the sourced workspace "
                "or the xacro include graph is blocking."
            ) from exc
        except subprocess.CalledProcessError as exc:
            stderr = exc.stderr or ""
            match = re.search(r'Invalid parameter "([^"]+)"', stderr)
            if not match:
                raise RuntimeError(f"Failed to expand xacro file '{abs_path}':\n{stderr}") from exc

            invalid_param = match.group(1)
            if invalid_param not in effective_mappings:
                raise RuntimeError(f"Failed to expand xacro file '{abs_path}':\n{stderr}") from exc
            if invalid_param in REQUIRED_CANONICAL_XACRO_MAPPINGS:
                raise RuntimeError(
                    f"Scene xacro '{abs_path}' rejected required canonical pose mapping "
                    f"'{invalid_param}'; no pose fallback was used."
                ) from exc
            effective_mappings.pop(invalid_param)


def load_yaml(package_name, rel_path):
    pkg_share = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_share, rel_path)
    if not os.path.exists(abs_path):
        return {}
    with open(abs_path, "r") as f:
        return yaml.safe_load(f) or {}






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

    if isinstance(value, (bool, int, float, str, bytes)):
        return value

    return value


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
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    joint_states_topic = f"/{scene_pkg}/joint_states"
    canonical_poses = load_canonical_layout_poses()
    table_pose = canonical_poses["support_surface"]
    camera_pose = canonical_poses["camera"]

    # --- Robot descriptions ---
    robot_description_config = load_xacro(
        scene_pkg,
        "urdf/scene.urdf.xacro",
        mappings={
            "ur_type": "ur5",
            "name": "ur5",
            "tf_prefix": "",
            "world_frame": "world",
            "use_fake_hardware": use_fake_hardware.perform(context),
            "table_world_xyz": _format_xacro_vector(table_pose["xyz"]),
            "table_world_rpy": _format_xacro_vector(table_pose["rpy"]),
            "camera_world_xyz": _format_xacro_vector(camera_pose["xyz"]),
            "camera_world_rpy": _format_xacro_vector(camera_pose["rpy"]),
        },
    )
    robot_description = {"robot_description": robot_description_config}

    robot_description_file = _write_robot_description_file(
        scene_pkg,
        robot_description_config,
    )

    robot_description_semantic_config = load_xacro(scene_pkg, "urdf/arm_hand.srdf.xacro")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # --- MoveIt configs ---
    kinematics_yaml = load_yaml(robot_moveit_pkg, "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml} if kinematics_yaml else {}

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

    # --- Controller manager (prevents move_group crash) ---
    # Minimal “fake” controller definition just to satisfy initialization.
    # You can later replace this with ros2_control controllers if you want execution.
    moveit_controller_manager = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    }

    moveit_simple_controller_manager = {
        "moveit_simple_controller_manager": {
            "controller_names": ["fake_ur5_controller"],
            "fake_ur5_controller": {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
            },
        }
    }

    # We’re not executing trajectories in this demo:
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

    # No depth sensor plugin is configured in this fake-hardware demo.
    # Disable occupancy map sensor plugins to suppress octomap warnings.
    occupancy_map_monitor_params = {
        "octomap_resolution": 0.025,
    }

    # --- Nodes ---

    try:
        validated_use_sim_time = _param_dict(_normalize_ros_param_types({"use_sim_time": use_sim_time.perform(context).lower() == "true"}))
        validated_robot_description = _param_dict(_normalize_ros_param_types(robot_description))
        validated_robot_description_semantic = _param_dict(_normalize_ros_param_types(robot_description_semantic))
        validated_robot_description_kinematics = _param_dict(_normalize_ros_param_types(robot_description_kinematics))
        validated_planning_pipelines_config = _param_dict(_normalize_ros_param_types(planning_pipelines_config))
        validated_ompl_planning_pipeline_config = _param_dict(_normalize_ros_param_types(ompl_planning_pipeline_config))
        validated_planning_scene_monitor_params = _param_dict(_normalize_ros_param_types(planning_scene_monitor_params))
        validated_trajectory_execution = _param_dict(_normalize_ros_param_types(trajectory_execution))
        validated_occupancy_map_monitor_params = _param_dict(_normalize_ros_param_types(occupancy_map_monitor_params))
        validated_moveit_controller_manager = _param_dict(_normalize_ros_param_types(moveit_controller_manager))
        validated_moveit_simple_controller_manager = _param_dict(_normalize_ros_param_types(moveit_simple_controller_manager))

        _validate_ros_param_types(validated_use_sim_time, "use_sim_time")
        _validate_ros_param_types(validated_robot_description, "robot_description")
        _validate_ros_param_types(validated_robot_description_semantic, "robot_description_semantic")
        _validate_ros_param_types(validated_robot_description_kinematics, "robot_description_kinematics")
        _validate_ros_param_types(validated_planning_pipelines_config, "planning_pipelines_config")
        _validate_ros_param_types(validated_ompl_planning_pipeline_config, "ompl_planning_pipeline_config")
        _validate_ros_param_types(validated_planning_scene_monitor_params, "planning_scene_monitor_params")
        _validate_ros_param_types(validated_trajectory_execution, "trajectory_execution")
        _validate_ros_param_types(validated_occupancy_map_monitor_params, "occupancy_map_monitor_params")
        _validate_ros_param_types(validated_moveit_controller_manager, "moveit_controller_manager")
        _validate_ros_param_types(validated_moveit_simple_controller_manager, "moveit_simple_controller_manager")
    except TypeError as exc:
        raise TypeError(f"{scene_pkg} demo.launch parameter validation failed: {exc}") from exc

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
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
            validated_occupancy_map_monitor_params,
            validated_trajectory_execution,
            validated_moveit_controller_manager,
            validated_moveit_simple_controller_manager,
        ),
        remappings=[
            ("joint_states", joint_states_topic),
            ("/joint_states", joint_states_topic),
        ],
    )

    rviz_config_file = os.path.join(get_package_share_directory(scene_pkg), "launch", "demo.rviz")
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
