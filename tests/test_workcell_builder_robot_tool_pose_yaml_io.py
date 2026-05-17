from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
YAML_IO = ROOT / "workcell_builder/workcell_builder/include/object_placement_yaml_io.hpp"
GEN = ROOT / "workcell_builder/workcell_builder/include/yaml_parser/generate_yaml.h"
PARSER = ROOT / "workcell_builder/workcell_builder/include/scene_xacro_parser.h"
LAYOUT_EDITOR = ROOT / "workcell_builder/workcell_builder/src_workcell_studio_layout_editor.cpp"
ROUNDTRIP = ROOT / "workcell_builder/workcell_builder/src_workcell_scene_roundtrip.cpp"


def test_robot_mount_and_tool_attachment_yaml_io_contracts_present():
    text = YAML_IO.read_text(encoding="utf-8") + GEN.read_text(encoding="utf-8")
    assert "RobotMountConfig" in text
    assert "ToolAttachmentConfig" in text
    assert '"robot_mount"' in text
    assert '"tool_attachment"' in text


def test_legacy_scene_fallback_marker_present():
    text = ROUNDTRIP.read_text(encoding="utf-8")
    assert '"legacy"' in text
    assert "schema_version" in text


def test_malformed_yaml_warning_path_present():
    text = LAYOUT_EDITOR.read_text(encoding="utf-8")
    assert "malformed YAML returns warning, not crash" in text


def test_nan_inf_rejection_for_mount_pose_present():
    text = PARSER.read_text(encoding="utf-8")
    assert "invalid NaN/Inf" in text


def test_recommended_rpy_presence_for_mount_fallback():
    text = PARSER.read_text(encoding="utf-8")
    assert "-1.5708 -1.5708 0" in text
