import os
import tempfile
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

import xacro

DEFAULT_SCENE_PACKAGE_CANDIDATES = ("ur5_3f_test", "ur5_2f_test", "ur5_airpick4_test", "suction_test")
PACKAGE_NAME = "run_grasp_execution"
SCENE_PACKAGE_ARGUMENT = "scene_package"


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
    return xacro.load_yaml(os.path.join(package_path, file_path))


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
    scene_package = LaunchConfiguration(SCENE_PACKAGE_ARGUMENT).perform(context)
    run_share = get_package_share_directory(PACKAGE_NAME)
    resolve_scene_package_share_dir(scene_package)
    resolve_required_package_share_dir(
        "ur5_moveit_config",
        "Run ./src/easy_manipulation_deployment/scripts/fix_workspace_layout.sh, rebuild the workspace, "
        "and source install/setup.bash before launching UR5 demo scenes.",
    )

    initial_position_path = os.path.join(run_share, "config", "start_positions.yaml")
    initial_position_mappings = {"initial_positions_file": initial_position_path}

    robot_description_config = load_file(scene_package, "urdf/scene.urdf.xacro", initial_position_mappings)
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_file(scene_package, "urdf/arm_hand.srdf.xacro")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    kinematics_yaml = {"robot_description_kinematics": load_yaml("ur5_moveit_config", "config/kinematics.yaml")}
    joint_limits_yaml = load_yaml("ur5_moveit_config", "config/joint_limits.yaml")
    joint_limits = {"robot_description_planning": joint_limits_yaml}

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
    ompl_planning_yaml = load_yaml("ur5_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    controllers_yaml = load_yaml(PACKAGE_NAME, "config/controllers.yaml")
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

    ros2_controllers_path = os.path.join(run_share, "config", "ur5_ros_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output={"stdout": "screen", "stderr": "screen"},
        parameters=[robot_description, ros2_controllers_path],
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

    spawn_joint_state = TimerAction(period=2.0, actions=[joint_state_spawner])
    spawn_arm = TimerAction(period=2.5, actions=[arm_spawner])

    return [
        robot_state_publisher,
        rviz_node,
        ros2_control_node,
        spawn_joint_state,
        spawn_arm,
        grasp_execution_demo_node,
    ]


def generate_launch_description():
    debug_arg = DeclareLaunchArgument("debug", default_value="false", description="Launch in debug mode")
    scene_description = "Scene package containing urdf/scene.urdf.xacro and urdf/arm_hand.srdf.xacro"
    scene_arg_kwargs = {"description": scene_description}
    if DEFAULT_SCENE_PACKAGE is not None:
        scene_arg_kwargs["default_value"] = DEFAULT_SCENE_PACKAGE

    return LaunchDescription(
        [
            debug_arg,
            DeclareLaunchArgument(SCENE_PACKAGE_ARGUMENT, **scene_arg_kwargs),
            OpaqueFunction(function=launch_setup),
        ]
    )
