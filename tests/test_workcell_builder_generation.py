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
