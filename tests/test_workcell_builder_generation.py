from pathlib import Path
import re

REPO_ROOT = Path(__file__).resolve().parents[1]


def test_scene_template_filters_fixed_gripper_joints_from_controller_list() -> None:
    content = (REPO_ROOT / "workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py").read_text(encoding="utf-8")
    assert "fixed" in content
    assert "_extract_controller_joints(robot_description_semantic_config, robot_description_config)" in content


def test_object_package_parser_no_spurious_nonpackage_warning() -> None:
    content = (REPO_ROOT / "workcell_builder/workcell_builder/include/object_package_parser.h").read_text(encoding="utf-8")
    assert "No CMakeLists or package.xml available" not in content


def test_default_assets_not_hardcoded_to_workspace_src_assets() -> None:
    content = (REPO_ROOT / "workcell_builder/workcell_builder/gui/addrobot.cpp").read_text(encoding="utf-8")
    assert "/home/ubuntu/workcell_ws/src/assets" not in content
    assert "get_package_share_directory(\"ur_description\")" in content


def test_object_package_parser_resolves_package_uris_before_copy() -> None:
    content = (REPO_ROOT / "workcell_builder/workcell_builder/include/object_package_parser.h").read_text(encoding="utf-8")
    assert "resolvePackageUriToPath" in content
    assert "fs::copy_file(link.visual_vector[0].geometry.filepath" not in content
    assert "fs::copy_file(\n              link.collision_vector[0].geometry.filepath" not in content


def test_launch_template_drops_none_typed_ros_params() -> None:
    content = (REPO_ROOT / "workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py").read_text(encoding="utf-8")
    assert "if value is None:" in content
    assert "return _DROP_PARAM" in content
    assert "\"finger_count\": finger_count" in content


def test_readme_validate_generated_scene_docs_no_manual_assets_move() -> None:
    content = (REPO_ROOT / "README.md").read_text(encoding="utf-8")
    assert "Validate generated scene" in content
    assert "Do not manually move assets/scenes into `~/workcell_ws/src/assets`" in content
    assert "rm -rf src/scenes/new_scene* src/new_scene* build/new_scene* install/new_scene* log/latest_build/new_scene*" in content


def test_onrobot_airpick_standalone_wrapper_exists_and_instantiates_required_params() -> None:
    content = (
        REPO_ROOT
        / "workcell_builder/workcell_builder/assets/end_effectors/onrobot_airpick4/onrobot_airpick4_description/urdf/onrobot_airpick4_gripper_standalone.urdf.xacro"
    ).read_text(encoding="utf-8")
    assert "<xacro:onrobot_airpick4_gripper" in content
    assert "prefix=\"\"" in content
    assert "parent=\"tool0\"" in content


def test_scene_select_startup_rediscovery_supports_scaffolded_packages() -> None:
    content = (REPO_ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp").read_text(encoding="utf-8")
    assert "discover_scene_packages_on_startup" in content
    assert "missing one of [package.xml, CMakeLists.txt, urdf/]" in content
    assert "without environment.yaml; scene can launch but cannot be fully edited until YAML exists." in content
    assert "scene already loaded" in content


def test_scene_xacro_sanitizes_end_effector_mount_origin_defaults_to_identity() -> None:
    content = (REPO_ROOT / "workcell_builder/workcell_builder/include/scene_xacro_parser.h").read_text(encoding="utf-8")
    assert "resolve_ee_mount_origin" in content
    assert "unset sentinel values (-1)" in content
    assert "invalid NaN/Inf values" in content
    assert "defaulting to identity origin xyz/rpy (0 0 0)" in content


def test_humble_scene_template_emits_ur5_arm_and_gripper_controllers() -> None:
    content = (REPO_ROOT / "workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py").read_text(encoding="utf-8")
    assert "UR_ARM_JOINTS" in content
    for joint in [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]:
        assert joint in content
    assert "_arm_controller" in content
    assert "_gripper_controller" in content
    assert '"controller_names": [arm_controller_name]' in content
    assert 'controllers["controller_names"].append(gripper_controller_name)' in content


def test_humble_scene_template_excludes_fixed_and_legacy_gripper_only_controller() -> None:
    content = (REPO_ROOT / "workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py").read_text(encoding="utf-8")
    assert 'joint != "gripper_base_joint"' in content
    assert "fake_gripper_controller" not in content
    assert '"type") not in (None, "fixed")' in content
    assert '"inner_knuckle" not in joint and "finger_tip" not in joint' in content
    assert 'preferred = ["gripper_finger1_joint", "gripper_finger2_joint"]' in content
    assert '"default": False' in content


def test_humble_scene_template_preserves_fake_hardware_launch_behavior() -> None:
    content = (REPO_ROOT / "workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py").read_text(encoding="utf-8")
    assert '"allow_trajectory_execution": False' in content
    assert '"moveit_manage_controllers": False' in content
    assert "use_fake_hardware" in content
