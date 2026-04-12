#!/usr/bin/env python3
import os
import yaml
import re
import subprocess

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

scene_pkg = "ur10_2f_test"
robot_base_link = "base_link"
robot_moveit_pkg = "ur10_moveit_config"


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
            effective_mappings.pop(invalid_param)


def load_yaml(package_name, rel_path):
    pkg_share = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_share, rel_path)
    if not os.path.exists(abs_path):
        return {}
    with open(abs_path, "r") as file:
        return yaml.safe_load(file) or {}






def _normalize_ros_param_types(value):
    if isinstance(value, dict):
        return {str(key): _normalize_ros_param_types(item) for key, item in value.items()}

    if isinstance(value, tuple):
        return [_normalize_ros_param_types(item) for item in value]

    if isinstance(value, list):
        return [_normalize_ros_param_types(item) for item in value]

    return value


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

    if isinstance(value, (bool, int, float, str)):
        return

    raise TypeError(f"Invalid ROS param type at {path}: {type(value).__name__} -> {value!r}")


def _launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    robot_description_config = load_xacro(
        scene_pkg,
        "urdf/scene.urdf.xacro",
        mappings={
            "ur_type": "ur10",
            "name": "ur10",
            "tf_prefix": "",
            "use_fake_hardware": use_fake_hardware.perform(context),
        },
    )
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_xacro(scene_pkg, "urdf/arm_hand.srdf.xacro")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

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

    moveit_controller_manager = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    }

    moveit_simple_controller_manager = {
        "moveit_simple_controller_manager": {
            "controller_names": ["fake_ur10_controller"],
            "fake_ur10_controller": {
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

    try:
        validated_use_sim_time = _normalize_ros_param_types({"use_sim_time": use_sim_time.perform(context).lower() == "true"})
        validated_robot_description = _normalize_ros_param_types(robot_description)
        validated_robot_description_semantic = _normalize_ros_param_types(robot_description_semantic)
        validated_robot_description_kinematics = _normalize_ros_param_types(robot_description_kinematics)
        validated_planning_pipelines_config = _normalize_ros_param_types(planning_pipelines_config)
        validated_ompl_planning_pipeline_config = _normalize_ros_param_types(ompl_planning_pipeline_config)
        validated_planning_scene_monitor_params = _normalize_ros_param_types(planning_scene_monitor_params)
        validated_trajectory_execution = _normalize_ros_param_types(trajectory_execution)
        validated_moveit_controller_manager = _normalize_ros_param_types(moveit_controller_manager)
        validated_moveit_simple_controller_manager = _normalize_ros_param_types(moveit_simple_controller_manager)

        _validate_ros_param_types(validated_use_sim_time, "use_sim_time")
        _validate_ros_param_types(validated_robot_description, "robot_description")
        _validate_ros_param_types(validated_robot_description_semantic, "robot_description_semantic")
        _validate_ros_param_types(validated_robot_description_kinematics, "robot_description_kinematics")
        _validate_ros_param_types(validated_planning_pipelines_config, "planning_pipelines_config")
        _validate_ros_param_types(validated_ompl_planning_pipeline_config, "ompl_planning_pipeline_config")
        _validate_ros_param_types(validated_planning_scene_monitor_params, "planning_scene_monitor_params")
        _validate_ros_param_types(validated_trajectory_execution, "trajectory_execution")
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
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "world",
            "--child-frame-id",
            robot_base_link,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[validated_use_sim_time, validated_robot_description],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[validated_use_sim_time, validated_robot_description],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            validated_use_sim_time,
            validated_robot_description,
            validated_robot_description_semantic,
            validated_robot_description_kinematics,
            validated_planning_pipelines_config,
            validated_ompl_planning_pipeline_config,
            validated_planning_scene_monitor_params,
            validated_trajectory_execution,
            validated_moveit_controller_manager,
            validated_moveit_simple_controller_manager,
        ],
    )

    rviz_config_file = os.path.join(get_package_share_directory(scene_pkg), "launch", "demo.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[
            validated_use_sim_time,
            validated_robot_description,
            validated_robot_description_semantic,
            validated_robot_description_kinematics,
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
