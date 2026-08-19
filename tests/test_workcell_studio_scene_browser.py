from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_scene_browser_files_and_status_strings_exist():
    h = ROOT / "workcell_builder/workcell_builder/include/workcell_studio_scene_browser.hpp"
    c = ROOT / "workcell_builder/workcell_builder/src_workcell_studio_scene_browser.cpp"
    cmake = ROOT / "workcell_builder/workcell_builder/CMakeLists.txt"
    assert h.exists()
    assert c.exists()
    text = c.read_text()
    for token in ["READY", "WARNINGS", "BLOCKED", "SCAFFOLD_ONLY", "MISSING_ENVIRONMENT_YAML", "PREVIEW_ONLY"]:
        assert token in text
    assert "src_workcell_studio_scene_browser.cpp" in cmake.read_text()


def test_scene_fixture_has_environment_yaml():
    scenes = ROOT / "scenes"
    assert scenes.exists()
    envs = list(scenes.glob("*/environment.yaml"))
    assert envs


def test_scene_browser_reads_current_robot_and_tool_identity_fields():
    text = (
        ROOT / "workcell_builder/workcell_builder/src_workcell_studio_scene_browser.cpp"
    ).read_text(encoding="utf-8")
    assert "scene_component_summary" in text
    assert 'for (const char * key : {"name", "model", "id", "profile"})' in text
    assert 'scene_component_summary(n["robot"])' in text
    assert 'scene_component_summary(n["end_effector"])' in text
    assert 'scene_component_summary(n["tool"])' in text

    canonical = (ROOT / "scenes/ur5_2f_test/environment.yaml").read_text(encoding="utf-8")
    assert "robot:\n  id: ur5\n  model: ur5" in canonical
    assert "tool:\n  id: robotiq_85_gripper\n  model: robotiq_85_gripper" in canonical
