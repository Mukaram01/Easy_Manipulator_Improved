import importlib.util
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/run_workcell_web3d_visual_acceptance.py"
spec = importlib.util.spec_from_file_location("web3d_acceptance", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_derives_scene_id_from_arbitrary_manifest_scene(tmp_path):
    scene = tmp_path / "not_ur5"
    scene.mkdir()
    (scene / "scene_manifest.yaml").write_text(yaml.safe_dump({"scene": {"id": "custom_cell_42"}}), encoding="utf-8")
    assert module.derive_scene_id(scene) == "custom_cell_42"
    default_output = module.BUILD_ROOT / f"{module.derive_scene_id(scene)}.web_scene.json"
    assert default_output == ROOT / "build/workcell_studio_web_scene/custom_cell_42.web_scene.json"


def test_derives_scene_id_from_environment_then_folder(tmp_path):
    scene = tmp_path / "folder_scene"
    scene.mkdir()
    (scene / "environment.yaml").write_text(yaml.safe_dump({"environment": {"scene_id": "env_scene"}}), encoding="utf-8")
    assert module.derive_scene_id(scene) == "env_scene"
    (scene / "environment.yaml").unlink()
    assert module.derive_scene_id(scene) == "folder_scene"


def test_acceptance_script_has_no_ur5_scene_logic_and_outputs_under_build():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "scenes/ur5_2f_test" not in text
    assert "ensure_workcell_studio_web_scene_fresh.py" in text
    assert "check_workcell_web_scene_mesh_contract.py" in text
    assert "check_workcell_web_scene_visual_bounds.py" in text
    assert "build" in text and "workcell_studio_web_scene" in text
    assert "visual_acceptance.json" in text
    assert "visual_acceptance.png" in text

WORKFLOW = ROOT / ".github/workflows/web3d-visual-acceptance.yml"


def test_web3d_visual_acceptance_workflow_exists():
    assert WORKFLOW.is_file()


def test_web3d_visual_acceptance_workflow_requires_browser_runtime():
    text = WORKFLOW.read_text(encoding="utf-8")
    assert "scripts/run_workcell_web3d_visual_acceptance.py" in text
    assert "--require-browser" in text
    assert "python3 -m playwright install --with-deps chromium" in text


def test_web3d_visual_acceptance_workflow_uploads_screenshot_report_and_scene_artifacts():
    text = WORKFLOW.read_text(encoding="utf-8")
    assert "actions/upload-artifact@v4" in text
    assert "build/workcell_studio_web_scene/ur5_2f_test.visual_acceptance.json" in text
    assert "build/workcell_studio_web_scene/ur5_2f_test.visual_acceptance.png" in text
    assert "build/workcell_studio_web_scene/ur5_2f_test.web_scene.json" in text
    assert "GITHUB_STEP_SUMMARY" in text
    for token in [
        "scene id",
        "browser_runtime_status",
        "mesh_loaded_count",
        "required_mesh_failed_count",
        "fallback_count",
        "screenshot artifact name",
        "report artifact name",
    ]:
        assert token in text


def test_web3d_visual_acceptance_workflow_does_not_commit_generated_outputs():
    result = __import__("subprocess").run(
        [
            "git",
            "ls-files",
            "scenes/*/generated/*.json",
            "build/workcell_studio_web_scene/*.json",
            "build/workcell_studio_web_scene/*.png",
        ],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=__import__("subprocess").PIPE,
        stderr=__import__("subprocess").PIPE,
    )
    assert result.stdout.splitlines() == []


def test_acceptance_script_supports_playwright_browser_status_path():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "from playwright.sync_api import sync_playwright" in text
    assert "p.chromium.launch" in text
    assert "window.__WORKCELL_VIEWER_STATUS__" in text
    assert "page.wait_for_function" in text
    assert "mesh_loaded_count == 0" in text
    assert "required_mesh_failed_count > 0" in text
    assert "viewer_url:" in text
    assert "screenshot_path:" in text
    assert "report_path:" in text
