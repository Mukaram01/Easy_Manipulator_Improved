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
