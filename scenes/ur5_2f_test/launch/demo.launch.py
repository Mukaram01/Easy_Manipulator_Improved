#!/usr/bin/env python3

import math
import os
import tempfile
import yaml
import re
import subprocess
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

scene_pkg = "ur5_2f_test"
# scene_package_dir:=<authored cell> runs this qualified launch on a Builder-saved cell's own files
# (environment, layout, model, controllers, collision manifest, cell definition) instead of the
# installed reference package. Other packages (robot, tool, MoveIt configs) still resolve normally.
_SCENE_PACKAGE_DIR = None


def _share(package_name):
    if package_name == scene_pkg and _SCENE_PACKAGE_DIR:
        return _SCENE_PACKAGE_DIR
    return get_package_share_directory(package_name)


def _bind_scene_package_dir(value):
    global _SCENE_PACKAGE_DIR
    if not value:
        _SCENE_PACKAGE_DIR = None
        return None
    path = os.path.realpath(value)
    required = ("environment.yaml", "cell_definition.yaml", "urdf/scene.urdf.xacro", "urdf/arm_hand.srdf.xacro",
                "config/ros2_controllers.yaml", "config/moveit_collision_objects.yaml")
    missing = [rel for rel in required if not os.path.isfile(os.path.join(path, rel))]
    if missing:
        raise RuntimeError(f"scene_package_dir {path} is not a generated cell package; missing {missing}. "
                           "Generate the cell in Workcell Builder first.")
    _SCENE_PACKAGE_DIR = path
    return path
robot_model = "ur5"
planning_group = "manipulator"
world_frame = "world"
robot_base_link = "base_link"
tool_mount_link = "tool0"
gripper_profile = "robotiq_85_gripper"
grasp_frame = "ee_palm"
robot_moveit_pkg = "ur5_moveit_config"

# Reference-scene runtime identity. A bound cell package supplies its own (_cell_runtime).
REFERENCE_RUNTIME = dict(
    robot_model=robot_model, planning_group=planning_group, tool_mount_link=tool_mount_link,
    gripper_profile=gripper_profile, grasp_frame=grasp_frame, robot_moveit_pkg=robot_moveit_pkg,
    arm_controller="ur5_arm_controller", tool_controller="ur5_gripper_controller",
    arm_joints=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
    tool_joints=["gripper_finger1_joint"],
    support_pose_id="support_surface_table", camera_pose_id="realsense_overhead",
)


def _cell_runtime():
    """Robot, tool, MoveIt package, controllers, joints and mount ids of the launched scene package.

    Reference package: REFERENCE_RUNTIME. Bound cell: its saved robot/end effector (environment.yaml),
    generated runtime bindings (cell_definition.yaml runtime) and controllers (config/ros2_controllers.yaml);
    a missing or inconsistent value fails closed instead of falling back to the reference cell.
    """
    if not _SCENE_PACKAGE_DIR:
        return dict(REFERENCE_RUNTIME)
    def load(rel):
        with open(os.path.join(_SCENE_PACKAGE_DIR, rel), "r", encoding="utf-8") as file:
            return yaml.safe_load(file) or {}
    environment, cell, controllers = load("environment.yaml"), load("cell_definition.yaml"), load(CONTROLLERS_REL_PATH)
    robot, tool, runtime = environment.get("robot") or {}, environment.get("end_effector") or environment.get("tool") or {}, cell.get("runtime") or {}
    supports = (environment.get("environment") or {}).get("support_surfaces") or []
    def controller_joints(name):
        joints = ((controllers.get(name) or {}).get("ros__parameters") or {}).get("joints")
        if not name or not isinstance(joints, list) or not joints:
            raise RuntimeError(f"cell controller {name!r} has no joints in {CONTROLLERS_REL_PATH}; Generate the cell again")
        return [str(joint) for joint in joints]
    values = dict(
        robot_model=str(robot.get("model") or "").lower(), planning_group=robot.get("planning_group"),
        tool_mount_link=robot.get("tool_mount_link"), gripper_profile=tool.get("profile"),
        grasp_frame=tool.get("grasp_frame"), robot_moveit_pkg=runtime.get("robot_moveit_package"),
        arm_controller=runtime.get("arm_controller"), tool_controller=runtime.get("tool_controller"),
        support_pose_id=supports[0].get("id") if len(supports) == 1 else None,
        camera_pose_id=(environment.get("camera") or {}).get("camera_id"))
    missing = sorted(key for key, value in values.items() if not value)
    if missing:
        raise RuntimeError(f"cell {_SCENE_PACKAGE_DIR} lacks runtime identity {missing}; Generate the cell again")
    values.update(arm_joints=controller_joints(values["arm_controller"]), tool_joints=controller_joints(values["tool_controller"]))
    command_joints = runtime.get("tool_command_joints")
    if command_joints is not None and list(command_joints) != values["tool_joints"]:
        raise RuntimeError(f"cell tool command joints {command_joints} differ from its tool controller {values['tool_joints']}")
    return values


# Joint-space corner blend for TOTG (see the planning pipeline below; the offline replay is in
# log/qualification/g3c/trajectory_continuity).
TOTG_PATH_TOLERANCE = 0.01
CANONICAL_LAYOUT_REL_PATH = "layout/workcell_studio_layout.yaml"
COLLISION_MANIFEST_REL_PATH = "config/moveit_collision_objects.yaml"
CONTROLLERS_REL_PATH = "config/ros2_controllers.yaml"
CANONICAL_LAYOUT_SCHEMA = "workcell_studio_layout/v1"
REQUIRED_AUTHORED_POSE_IDS = (
    "support_surface_table",
    "realsense_overhead",
)
REQUIRED_ENVIRONMENT_ROBOT_POSE = "robot"
REQUIRED_CANONICAL_XACRO_MAPPINGS = {
    "table_world_xyz",
    "table_world_rpy",
    "camera_world_xyz",
    "camera_world_rpy",
    "robot_world_xyz",
    "robot_world_rpy",
}


def _format_xacro_vector(values):
    return " ".join(format(value, ".17g") for value in values)


def load_canonical_layout_poses(package_name=scene_pkg, layout_path=None, required_ids=REQUIRED_AUTHORED_POSE_IDS):
    """Load runtime mount poses from authoritative environment.yaml."""
    if layout_path is None:
        layout_path = os.path.join(
            _share(package_name),
            "environment.yaml",
        )

    try:
        with open(layout_path, "r", encoding="utf-8") as file:
            layout = yaml.safe_load(file)
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(
            f"Cannot load canonical Workcell Studio layout '{layout_path}': {exc}. "
            "Save the scene layout in Workcell Studio and relaunch."
        ) from exc

    if not isinstance(layout, dict) or not isinstance(layout.get("environment"), dict):
        raise RuntimeError(f"Authoritative environment mapping missing: {layout_path}")
    physical = layout["environment"]
    items = [dict(item, pose=item.get("pose") or {
        "xyz": item.get("pose_xyz"), "rpy": item.get("pose_rpy")})
        for section in ("support_surfaces", "assets", "sensors")
        for item in physical.get(section, [])]

    poses = {}
    for required_id in required_ids:
        matches = [
            item
            for item in items
            if isinstance(item, dict) and item.get("id") == required_id
        ]
        if len(matches) != 1:
            raise RuntimeError(
                f"Canonical Workcell Studio layout '{layout_path}' must contain exactly one "
                f"physical authored item with id '{required_id}'; found {len(matches)}."
            )

        pose = matches[0].get("pose")
        if not isinstance(pose, dict):
            raise RuntimeError(
                f"Canonical Workcell Studio layout item '{required_id}' in '{layout_path}' "
                "must contain a pose with xyz and rpy vectors."
            )

        validated_pose = {}
        for vector_name in ("xyz", "rpy"):
            vector = pose.get(vector_name)
            if (
                not isinstance(vector, (list, tuple))
                or len(vector) != 3
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                    for value in vector
                )
            ):
                raise RuntimeError(
                    f"Canonical Workcell Studio layout item '{required_id}' pose.{vector_name} "
                    f"in '{layout_path}' must contain exactly 3 finite numbers."
                )
            validated_pose[vector_name] = tuple(float(value) for value in vector)
        poses[required_id] = validated_pose

    return poses


def load_authored_robot_pose(package_name=scene_pkg, environment_path=None):
    """Load the robot mounting pose from the authored environment contract."""
    if environment_path is None:
        environment_path = os.path.join(_share(package_name), "environment.yaml")
    try:
        with open(environment_path, "r", encoding="utf-8") as file:
            environment = yaml.safe_load(file)
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"Cannot load authored robot pose from '{environment_path}': {exc}") from exc
    robot = environment.get("robot") if isinstance(environment, dict) else None
    if not isinstance(robot, dict):
        raise RuntimeError(f"Authored environment '{environment_path}' must contain a robot map")
    pose = {"xyz": robot.get("pose_xyz"), "rpy": robot.get("pose_rpy")}
    for vector_name, vector in pose.items():
        if (not isinstance(vector, (list, tuple)) or len(vector) != 3 or
                any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                    for value in vector)):
            raise RuntimeError(f"Authored environment robot pose_{vector_name} must contain three finite numbers")
    return {name: tuple(float(value) for value in vector) for name, vector in pose.items()}


def load_xacro(package_name, rel_path, mappings=None):
    pkg_share = _share(package_name)
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
                    f"Scene xacro '{abs_path}' rejected required canonical Workcell Studio "
                    f"pose mapping '{invalid_param}'. Rebuild/install {package_name} so the "
                    "launch file and scene xacro use the same version; no pose fallback was used."
                ) from exc
            effective_mappings.pop(invalid_param)


def load_yaml(package_name, rel_path):
    pkg_share = _share(package_name)
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
    # One file per launch: concurrent sessions (or cells) never read each other's model.
    descriptor, path = tempfile.mkstemp(prefix=f"{scene_name}_", suffix="_robot_description.urdf")
    with os.fdopen(descriptor, "w", encoding="utf-8") as file:
        file.write(robot_description_config)
    return path


def _launch_setup(context):
    _bind_scene_package_dir(LaunchConfiguration("scene_package_dir").perform(context))
    backend = LaunchConfiguration("execution_backend").perform(context)
    if backend not in ("fake", "simulator"):
        raise RuntimeError("Unknown execution backend")
    simulator_actions = []
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    joint_states_topic = "/joint_states"
    canonical_layout_path = os.path.join(
        _share(scene_pkg),
        CANONICAL_LAYOUT_REL_PATH,
    )
    collision_manifest_path = os.path.join(
        _share(scene_pkg),
        COLLISION_MANIFEST_REL_PATH,
    )
    controllers_config_path = os.path.join(
        _share(scene_pkg),
        CONTROLLERS_REL_PATH,
    )
    if not os.path.isfile(controllers_config_path):
        raise RuntimeError(
            f"Missing fake-hardware controller configuration: {controllers_config_path}. "
            f"Rebuild/install {scene_pkg} before launching."
        )

    cell_runtime = _cell_runtime()
    canonical_poses = load_canonical_layout_poses(
        required_ids=(cell_runtime["support_pose_id"], cell_runtime["camera_pose_id"]))
    table_pose = canonical_poses[cell_runtime["support_pose_id"]]
    camera_pose = canonical_poses[cell_runtime["camera_pose_id"]]
    robot_pose = load_authored_robot_pose()

    robot_description_config = load_xacro(
        scene_pkg,
        "urdf/scene.urdf.xacro",
        mappings={
            "ur_type": cell_runtime["robot_model"],
            "name": cell_runtime["robot_model"],
            "tf_prefix": "",
            "world_frame": world_frame,
            "tool_mount_link": cell_runtime["tool_mount_link"],
            "gripper_profile": cell_runtime["gripper_profile"],
            "grasp_frame": cell_runtime["grasp_frame"],
            "use_fake_hardware": use_fake_hardware.perform(context),
            "table_world_xyz": _format_xacro_vector(table_pose["xyz"]),
            "table_world_rpy": _format_xacro_vector(table_pose["rpy"]),
            "camera_world_xyz": _format_xacro_vector(camera_pose["xyz"]),
            "camera_world_rpy": _format_xacro_vector(camera_pose["rpy"]),
            "robot_world_xyz": _format_xacro_vector(robot_pose["xyz"]),
            "robot_world_rpy": _format_xacro_vector(robot_pose["rpy"]),
        },
    )
    if backend == "simulator":
        if use_sim_time.perform(context).lower() != "true" or use_fake_hardware.perform(context).lower() != "true":
            raise RuntimeError("Simulator requires use_sim_time:=true and safe mock-source model derivation")
        # Source checkout and installed entry points use the same reusable backend.
        source_scripts = Path(__file__).resolve().parents[3] / "scripts"
        if source_scripts.is_dir():
            sys.path.insert(0, str(source_scripts))
        else:
            from ament_index_python.packages import get_package_prefix
            sys.path.insert(0, str(Path(get_package_prefix('workcell_builder')) / 'lib/workcell_builder'))
        import simulator_backend
        import end_effector_hold
        output = LaunchConfiguration("simulator_output").perform(context)
        # The cell's end-effector capability profile states how this tool holds
        # (method, reserve, actuator force). Its actuator cap is bound into the
        # source description here, outside the evidence-bound simulator_backend
        # module, so the derived model and the receipt carry it.
        hold_profile = end_effector_hold.load_cell_hold_profile(
            os.path.join(_share(scene_pkg), "cell_definition.yaml"))
        if hold_profile is None and LaunchConfiguration("simulator_commissioning").perform(context).lower() == "true":
            raise RuntimeError("simulator commissioning requires the cell to bind an end-effector capability with a hold profile")
        hold_binding = None
        if hold_profile is not None:
            robot_description_config, hold_binding = end_effector_hold.bind_source_description(
                robot_description_config, hold_profile)
        robot_description_config, controllers_config_path, simulator_spec = simulator_backend.prepare(
            robot_description_config, controllers_config_path,
            LaunchConfiguration("simulator_world").perform(context),
            f"{scene_pkg}_robot_state_publisher", output,
            collision_manifest_path=collision_manifest_path)
        if hold_binding is not None:
            simulator_spec['end_effector_hold'] = end_effector_hold.augment_spec(output, hold_binding)
        # Physics contact geometry declared by the same capability record: primitive pads
        # replace the concave fingertip meshes in the spawned physics model (robot.sdf) only;
        # MoveIt keeps the exact mesh. The new model hash goes into the spec the receipt verifies.
        if hold_profile is not None:
            import simulation_collision
            from capability_registry import load_capability_registry
            record = load_capability_registry().get(hold_profile.capability_id)
            collision_binding = simulation_collision.bind_native_model(output, record.payload)
            if collision_binding is not None:
                simulator_spec['simulator_collision_primitives'] = collision_binding
                simulator_spec['native_description_sha256'] = collision_binding['native_description_sha256']
        # Destination (bin floor) binding for the contact guard; kept outside the
        # evidence-bound simulator_backend module. Fails closed on a floor mismatch.
        import destination_binding
        simulator_spec['destination_geometry_binding'] = destination_binding.augment_spec(output, collision_manifest_path,
            measured_floor=_SCENE_PACKAGE_DIR is not None)  # a derived cell world measures its own floor
        simulator_actions = [
            ExecuteProcess(cmd=[sys.executable, simulator_backend.__file__, output], output='screen',
                           additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}),
            Node(package='ros_gz_bridge', executable=simulator_backend.bridge_executable(
                     LaunchConfiguration('simulator_commissioning').perform(context).lower() == 'true'),
                 parameters=[{'config_file': str(Path(output)/'bridges.yaml')}], output='screen'),
            TimerAction(period=5.0, actions=[Node(package='controller_manager', executable='spawner',
                arguments=['physical_joint_states', '--param-file', controllers_config_path], output='screen')]),
        ]
    robot_description = {"robot_description": robot_description_config}

    robot_description_file = _write_robot_description_file(
        scene_pkg,
        robot_description_config,
    )

    robot_description_semantic_config = load_xacro(scene_pkg, "urdf/arm_hand.srdf.xacro")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    kinematics_yaml = load_yaml(cell_runtime["robot_moveit_pkg"], "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml} if kinematics_yaml else {}

    ompl_planning_yaml = load_yaml(cell_runtime["robot_moveit_pkg"], "config/ompl_planning.yaml")

    planning_pipelines_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
    }

    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": " ".join([
                "workcell/InitialSupportContact",
            ] + (["workcell/PlanningEvidence"] if os.environ.get('WORKCELL_PLANNING_TRACE_DIR') else []) + [
                "workcell/StraightCartesianPath",
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
    # TOTG blends each OMPL vertex within this joint-space tolerance. 0.0 (the robot package's value) left every corner
    # unblended: the trajectory turned instantly between two 0.1 s samples, so the controller's quintic implied up to
    # 18.5 rad/s^2 (UR5 limit 3.15) and the arm cut the corner by up to 14 mm (G3C). The blend cannot cut an obstacle
    # unseen: workcell/StraightCartesianPath certifies the time-parameterised controller spline afterwards.
    ompl_planning_pipeline_config["ompl"]["path_tolerance"] = TOTG_PATH_TOLERANCE

    moveit_controller_manager = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    }

    moveit_simple_controller_manager = {
        "moveit_simple_controller_manager": {
            "controller_names": [cell_runtime["arm_controller"], cell_runtime["tool_controller"]],
            cell_runtime["arm_controller"]: {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": cell_runtime["arm_joints"],
            },
            cell_runtime["tool_controller"]: {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": False,
                "joints": cell_runtime["tool_joints"],
            },
        }
    }

    fake_hardware_enabled = use_fake_hardware.perform(context).lower() == "true"
    execution_requested = LaunchConfiguration("allow_trajectory_execution").perform(context).lower() == "true"
    commissioning = LaunchConfiguration("simulator_commissioning").perform(context).lower() == "true"
    if commissioning and (backend != 'simulator' or not execution_requested):
        raise RuntimeError('simulator_commissioning requires simulator backend and explicit trajectory execution')
    if execution_requested and not fake_hardware_enabled:
        raise RuntimeError("This scene permits trajectory execution only with use_fake_hardware:=true")
    trajectory_execution = {
        # Planning and controller discovery are available in fake hardware,
        # while trajectory execution requires explicit fake-hardware opt-in.
        "allow_trajectory_execution": False,
        "moveit_manage_controllers": False,
        "use_fake_hardware": fake_hardware_enabled and backend == "fake",
        "execution_backend": backend,
        "simulator_commissioning": commissioning,
    }
    if execution_requested:
        trajectory_execution["allow_trajectory_execution"] = True
    if commissioning:
        trajectory_execution.update(capabilities='workcell/CommissionExecuteTrajectory',
            disable_capabilities='move_group/MoveGroupExecuteTrajectoryAction')

    planning_scene_monitor_params = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        # EPD publishes explicit, lifecycle-managed collision objects into
        # MoveIt's PlanningScene.  This canonical scene intentionally does not
        # create a second depth-cloud Octomap.  MoveIt Humble's occupancy-map
        # middleware otherwise emits ERROR/WARN messages when no 3D plugin is
        # configured.  A '~' plugin is the middleware's explicit skip contract.
        "octomap_resolution": 0.1,
        "octomap_frame": world_frame,
        "sensors": ["workcell_epd_collision_objects"],
        "workcell_epd_collision_objects": {
            "sensor_plugin": "~workcell_epd_collision_objects_are_planning_truth",
        },
    }

    try:
        validated_use_sim_time = _param_dict(_normalize_ros_param_types({"use_sim_time": use_sim_time.perform(context).lower() == "true"}))
        validated_robot_description = _param_dict(_normalize_ros_param_types(robot_description))
        validated_robot_description_semantic = _param_dict(_normalize_ros_param_types(robot_description_semantic))
        validated_robot_description_kinematics = _param_dict(_normalize_ros_param_types(robot_description_kinematics))
        validated_planning_pipelines_config = _param_dict(_normalize_ros_param_types(planning_pipelines_config))
        validated_ompl_planning_pipeline_config = _param_dict(_normalize_ros_param_types(ompl_planning_pipeline_config))
        validated_planning_scene_monitor_params = _param_dict(_normalize_ros_param_types(planning_scene_monitor_params))
        validated_trajectory_execution = _param_dict(_normalize_ros_param_types(trajectory_execution))
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
        _validate_ros_param_types(validated_moveit_controller_manager, "moveit_controller_manager")
        _validate_ros_param_types(validated_moveit_simple_controller_manager, "moveit_simple_controller_manager")
    except TypeError as exc:
        raise TypeError(f"{scene_pkg} demo.launch parameter validation failed: {exc}") from exc

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

    # A named cell reference is useful to RViz and marker tools.  The robot's
    # world->base_link transform comes exclusively from the URDF base joint;
    # this identity frame intentionally avoids a duplicate robot-base TF.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{scene_pkg}_cell_reference_tf",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", world_frame,
            "--child-frame-id", "workcell_reference",
        ],
    )

    # Preserve the previous visualization-only state publisher for the explicit
    # non-fake path, but fake-hardware acceptance must use ros2_control state.
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name=f"{scene_pkg}_joint_state_publisher",
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
        arguments=[robot_description_file],
        parameters=_param_list(
            validated_use_sim_time,
        ),
        remappings=[
            ("joint_states", joint_states_topic),
            ("/joint_states", joint_states_topic),
        ],
    )

    # Canonical M1 fake-hardware runtime.  The controller manager exists only
    # when use_fake_hardware is true, so this launch never starts ros2_control
    # against a physical UR driver.  robot_description comes from the local
    # robot_state_publisher topic and contains mock_components/GenericSystem.
    # Do not set Node(name=...) here: on ROS 2 Humble that becomes a process-wide
    # __node remap and also renames controller nodes created inside ros2_control_node.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        condition=IfCondition(use_fake_hardware),
        parameters=_param_list(validated_use_sim_time, controllers_config_path),
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        condition=IfCondition(use_fake_hardware),
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_config_path,
        ],
    )
    # Compatibility contract: ["joint_state_broadcaster", "--controller-manager", "/controller_manager"]

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        condition=IfCondition(use_fake_hardware),
        arguments=[
            cell_runtime["arm_controller"],
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_config_path,
        ],
    )
    # Compatibility contract: ["ur5_arm_controller", "--controller-manager", "/controller_manager"]

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        condition=IfCondition(use_fake_hardware),
        arguments=[
            cell_runtime["tool_controller"],
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_config_path,
        ],
    )
    # Compatibility contract: ["ur5_gripper_controller", "--controller-manager", "/controller_manager"]

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
            validated_trajectory_execution,
            validated_moveit_controller_manager,
            validated_moveit_simple_controller_manager,
        ),
        remappings=[
            ("joint_states", joint_states_topic),
            ("/joint_states", joint_states_topic),
        ],
    )

    rviz_config_file = os.path.join(_share(scene_pkg), "launch", "demo.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(launch_rviz),
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

    canonical_mesh_preview = Node(
        package="workcell_builder",
        executable="workcell_studio_layout_mesh_preview_node.py",
        name=f"{scene_pkg}_canonical_mesh_preview",
        output="screen",
        arguments=[
            canonical_layout_path,
            "--frame-id",
            world_frame,
            "--topic",
            f"/{scene_pkg}/canonical_mesh_markers",
        ],
    )

    # Web3D remains the authoring visualization.  This guarded loader applies
    # the reviewed, deterministic collision manifest to MoveIt, which is the
    # collision and planning source of truth.
    planning_scene_loader = Node(
        package="workcell_builder",
        executable="workcell_studio_planning_scene_node.py",
        name=f"{scene_pkg}_planning_scene_loader",
        output="screen",
        arguments=[
            collision_manifest_path,
            "--service",
            "/apply_planning_scene",
            "--verify-service",
            "/get_planning_scene",
            "--timeout-seconds",
            "45",
        ],
    )

    return [
        robot_state_publisher,
        static_tf,
        joint_state_publisher,
        *([control_node] if backend == "fake" else simulator_actions),
        # ros2_control_node must advertise its controller-manager services
        # before the spawners query them; staggering also prevents three
        # simultaneous configure requests from racing on Humble.
        TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=3.0, actions=[arm_controller_spawner]),
        TimerAction(period=4.0, actions=[gripper_controller_spawner]),
        move_group,
        *([canonical_mesh_preview] if backend == "fake" else []),
        planning_scene_loader,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("execution_backend", default_value="fake"),
        DeclareLaunchArgument("scene_package_dir", default_value=""),
        DeclareLaunchArgument("simulator_world", default_value=""),
        DeclareLaunchArgument("simulator_output", default_value=""),
        DeclareLaunchArgument("simulator_commissioning", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "allow_trajectory_execution", default_value="false",
            description="Opt in to fake-hardware trajectory execution; rejected with real hardware.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description=(
                "Launch RViz for interactive fake-hardware visualization. "
                "Set false for bounded headless acceptance checks."
            ),
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
