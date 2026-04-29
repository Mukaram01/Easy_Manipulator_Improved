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


def test_scene_xacro_uses_fallback_robotiq_85_mount_orientation_for_ur() -> None:
    content = (REPO_ROOT / "workcell_builder/workcell_builder/include/scene_xacro_parser.h").read_text(encoding="utf-8")
    assert "resolve_default_ee_mount_origin" in content
    assert "Fallback defaults only apply when user did not explicitly set an EE origin." in content
    assert "robot.brand == \"universal_robot\" && ee.brand == \"robotiq_85_gripper\"" in content
    assert "fallback.yaw = 1.57079632679;" in content
