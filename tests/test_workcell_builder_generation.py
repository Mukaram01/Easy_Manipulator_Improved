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
