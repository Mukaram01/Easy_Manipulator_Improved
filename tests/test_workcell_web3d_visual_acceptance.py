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
    assert "validate_browser_status(status)" in text
    assert "EXPECTED_MESH_LOADED_COUNT = 18" in text
    assert "EXPECTED_REQUIRED_MESH_FAILED_COUNT = 0" in text
    assert "viewer_url:" in text
    assert "screenshot_path:" in text
    assert "report_path:" in text


def test_acceptance_script_preserves_canonical_mesh_count_expectations():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "EXPECTED_MESH_LOADED_COUNT = 18" in text
    assert "EXPECTED_REQUIRED_MESH_FAILED_COUNT = 0" in text
    assert "meshLoadedCount expected {EXPECTED_MESH_LOADED_COUNT}" in text
    assert "requiredMeshFailedCount expected {EXPECTED_REQUIRED_MESH_FAILED_COUNT}" in text


def test_acceptance_script_requires_viewer_side_resolved_tool_chain_distances():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "REQUIRED_VIEWER_RESOLVED_DISTANCE_PAIRS" in text
    for pair in [
        "wrist_3_link -> tool0",
        "tool0 -> gripper_base_link",
        "wrist_3_link -> gripper_base_link",
    ]:
        assert pair in text
    assert "viewer_resolved_distances_m" in text
    assert "resolved_distances_m" in text
    assert "browser viewer resolved distance {pair} expected <=" in text


def test_browser_status_validator_accepts_expected_meshes_and_tool_chain_distances():
    distance_pairs = {
        "wrist_3_link -> tool0": 0.001,
        "tool0 -> gripper_base_link": 0.10,
        "wrist_3_link -> gripper_base_link": 0.10,
    }
    assert set(distance_pairs) == set(module.REQUIRED_VIEWER_RESOLVED_DISTANCE_PAIRS)
    for pair, distance in distance_pairs.items():
        assert distance <= module.REQUIRED_VIEWER_RESOLVED_DISTANCE_PAIRS[pair]

    camel_status = {
        "meshLoadedCount": 18,
        "requiredMeshFailedCount": 0,
        "viewer_resolved_distances_m": distance_pairs,
        "renderedMeshDiagnostics": _valid_rendered_mesh_diagnostics(),
    }
    snake_status = {
        "mesh_loaded_count": 18,
        "required_mesh_failed_count": 0,
        "viewer_resolved_distances_m": distance_pairs,
        "rendered_mesh_diagnostics": _valid_rendered_mesh_diagnostics(),
    }

    assert camel_status["meshLoadedCount"] == module.EXPECTED_MESH_LOADED_COUNT == 18
    assert camel_status["requiredMeshFailedCount"] == module.EXPECTED_REQUIRED_MESH_FAILED_COUNT == 0
    assert snake_status["mesh_loaded_count"] == module.EXPECTED_MESH_LOADED_COUNT == 18
    assert snake_status["required_mesh_failed_count"] == module.EXPECTED_REQUIRED_MESH_FAILED_COUNT == 0
    assert module.validate_browser_status(camel_status) == []
    assert module.validate_browser_status(snake_status) == []


def test_browser_status_validator_rejects_missing_viewer_side_tool_chain_distance():
    missing_pair = "wrist_3_link -> gripper_base_link"
    reported_distances = {
        "wrist_3_link -> tool0": 0.001,
        "tool0 -> gripper_base_link": 0.10,
    }
    assert missing_pair in module.REQUIRED_VIEWER_RESOLVED_DISTANCE_PAIRS
    assert missing_pair not in reported_distances

    status = {
        "mesh_loaded_count": 18,
        "required_mesh_failed_count": 0,
        "viewer_resolved_distances_m": reported_distances,
        "rendered_mesh_diagnostics": _valid_rendered_mesh_diagnostics(),
    }

    errors = module.validate_browser_status(status)
    assert any(missing_pair in error for error in errors)


def test_browser_status_validator_rejects_missing_viewer_side_distance_map():
    status = {
        "mesh_loaded_count": 18,
        "required_mesh_failed_count": 0,
        "rendered_mesh_diagnostics": _valid_rendered_mesh_diagnostics(),
    }

    errors = module.validate_browser_status(status)
    for pair in module.REQUIRED_VIEWER_RESOLVED_DISTANCE_PAIRS:
        assert any(pair in error for error in errors)


def _valid_rendered_mesh_diagnostics(spacing=0.05):
    diagnostics = []
    for index, link in enumerate([
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
        "tool0",
        "gripper_base_link",
    ]):
        diagnostics.append(
            {
                "link_name": link,
                "loaded_mesh_bounding_box_center": {"x": index * spacing, "y": 0.0, "z": 0.0},
                "visual_wrapper_world_position": {"x": index * spacing, "y": 0.0, "z": 0.0},
            }
        )
    diagnostics.append(_valid_table_diagnostic())
    return diagnostics


def _valid_table_diagnostic():
    return {
        "id": "workbench",
        "object_id": "workbench",
        "object_name": "M1 workbench",
        "category": "table",
        "link_name": "workbench",
        "bounding_box_size": {"x": 1.20, "y": 0.80, "z": 0.05},
        "bounding_box_center": {"x": 0.40, "y": 0.0, "z": 0.75},
        "inferred_up_axis": {"x": 0.0, "y": 0.0, "z": 1.0},
    }


def _rotated_table_diagnostic():
    diagnostic = _valid_table_diagnostic()
    diagnostic["bounding_box_size"] = {"x": 1.20, "y": 0.05, "z": 0.80}
    diagnostic["inferred_up_axis"] = {"x": 0.0, "y": 1.0, "z": 0.0}
    return diagnostic


def _valid_viewer_distances():
    return {
        "wrist_3_link -> tool0": 0.001,
        "tool0 -> gripper_base_link": 0.10,
        "wrist_3_link -> gripper_base_link": 0.10,
    }


def _valid_browser_status_with_rendered_diagnostics():
    return {
        "meshLoadedCount": 18,
        "requiredMeshFailedCount": 0,
        "viewer_resolved_distances_m": _valid_viewer_distances(),
        "renderedMeshDiagnostics": _valid_rendered_mesh_diagnostics(),
    }


def test_browser_status_validator_rejects_missing_rendered_mesh_diagnostics():
    status = {
        "mesh_loaded_count": 18,
        "required_mesh_failed_count": 0,
        "viewer_resolved_distances_m": _valid_viewer_distances(),
    }

    errors = module.validate_browser_status(status)

    assert any("renderedMeshDiagnostics/rendered_mesh_diagnostics is required" in error for error in errors)


def test_browser_status_validator_rejects_missing_required_rendered_links():
    status = _valid_browser_status_with_rendered_diagnostics()
    status["renderedMeshDiagnostics"] = [
        diagnostic for diagnostic in status["renderedMeshDiagnostics"] if diagnostic["link_name"] != "forearm_link"
    ]

    errors = module.validate_browser_status(status)

    assert any("upper_arm_link -> forearm_link" in error and "forearm_link" in error for error in errors)
    assert any("forearm_link -> wrist_1_link" in error and "forearm_link" in error for error in errors)


def test_browser_status_validator_rejects_excessive_adjacent_rendered_mesh_separation():
    status = _valid_browser_status_with_rendered_diagnostics()
    for diagnostic in status["renderedMeshDiagnostics"]:
        if diagnostic["link_name"] == "upper_arm_link":
            diagnostic["loaded_mesh_bounding_box_center"] = {"x": 10.0, "y": 0.0, "z": 0.0}
            diagnostic["visual_wrapper_world_position"] = {"x": 10.0, "y": 0.0, "z": 0.0}

    errors = module.validate_browser_status(status)

    assert any("shoulder_link -> upper_arm_link" in error and "expected <=" in error for error in errors)
    assert any("upper_arm_link -> forearm_link" in error and "expected <=" in error for error in errors)


def test_browser_status_validator_accepts_valid_rendered_mesh_diagnostics_snake_case():
    status = {
        "mesh_loaded_count": 18,
        "required_mesh_failed_count": 0,
        "viewer_resolved_distances_m": _valid_viewer_distances(),
        "rendered_mesh_diagnostics": _valid_rendered_mesh_diagnostics(),
    }

    assert module.validate_browser_status(status) == []


def test_browser_status_validator_accepts_horizontal_table_diagnostic():
    status = _valid_browser_status_with_rendered_diagnostics()

    assert module.validate_browser_status(status) == []


def test_browser_status_validator_rejects_90_degree_rotated_table_diagnostic():
    status = _valid_browser_status_with_rendered_diagnostics()
    status["renderedMeshDiagnostics"] = [
        _rotated_table_diagnostic() if diagnostic.get("category") == "table" else diagnostic
        for diagnostic in status["renderedMeshDiagnostics"]
    ]

    errors = module.validate_browser_status(status)

    assert any("thickness/smallest dimension along Z" in error for error in errors)
    assert any("normal/up close to world +Z" in error for error in errors)
