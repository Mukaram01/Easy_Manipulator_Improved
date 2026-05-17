from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCENE_XACRO_TEST = ROOT / "workcell_builder/workcell_builder/test/scene_xacro_tool_attachment_test.cpp"
YAML_TEST = ROOT / "workcell_builder/workcell_builder/test/generate_yaml_end_effector_metadata_test.cpp"
HEALTHCHECK = ROOT / "scripts/validate_workcell_builder_healthcheck.py"


def test_robot_base_origin_and_generated_urdf_xacro_markers_present():
    text = SCENE_XACRO_TEST.read_text(encoding="utf-8")
    assert "scene.urdf.xacro" in text
    assert "ee.origin.x" in text
    assert "ee.origin.roll" in text


def test_tool_fixed_joint_parent_child_origin_markers_present():
    text = YAML_TEST.read_text(encoding="utf-8")
    assert 'tool_attachment["parent_link"]' in text
    assert 'tool_attachment["child_link"]' in text
    assert 'tool_attachment["joint_name"]' in text
    assert 'tool_attachment["origin"]["rpy"]' in text


def test_explicit_recommended_rpy_value_present():
    text = YAML_TEST.read_text(encoding="utf-8")
    assert "-1.5708F" in text
    assert "0.0F" in text


def test_safe_fallback_when_tool_attachment_missing_marker_present():
    text = (ROOT / "workcell_builder/workcell_builder/templates/ros2/humble/test_load_yaml.py").read_text(encoding="utf-8")
    assert "extract_end_effector_metadata" in text
    assert "legacy" in text.lower()


def test_preview_artifacts_do_not_enable_controllers_trajectory_or_real_hardware():
    text = HEALTHCHECK.read_text(encoding="utf-8").lower()
    assert "execute_trajectory" in text
    assert "followjointtrajectory" in text
    assert "real_hardware_enabled: false" in text
