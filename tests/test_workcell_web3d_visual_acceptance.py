import importlib.util
import json
import sys
import types
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/run_workcell_web3d_visual_acceptance.py"
spec = importlib.util.spec_from_file_location("web3d_acceptance", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_request_failure_text_handles_none_string_mapping_and_object():
    failure_object = type("Failure", (), {"error_text": "object network reset"})()
    assert module._request_failure_error_text(None) == ""
    assert module._request_failure_error_text("string network reset") == "string network reset"
    assert module._request_failure_error_text({"error_text": "dict network reset"}) == "dict network reset"
    assert module._request_failure_error_text({"message": "dict message reset"}) == "dict message reset"
    assert module._request_failure_error_text(failure_object) == "object network reset"


def test_record_failed_request_records_string_object_and_dict_failures_without_throwing():
    failures = []
    for failure in (
        "string failed",
        {"error_text": "dict failed"},
        type("Failure", (), {"error_text": "object failed"})(),
    ):
        request = type("Request", (), {"method": "GET", "url": "http://viewer/asset.dae", "failure": failure})()
        module._record_failed_request(request, failures)

    class BrokenRequest:
        @property
        def method(self):
            raise RuntimeError("method broke")

        @property
        def url(self):
            raise RuntimeError("url broke")

        @property
        def failure(self):
            raise RuntimeError("failure broke")

    module._record_failed_request(BrokenRequest(), failures)
    assert any("string failed" in item for item in failures)
    assert any("dict failed" in item for item in failures)
    assert any("object failed" in item for item in failures)
    assert any("unreadable method" in item and "unreadable url" in item for item in failures)


def test_playwright_requestfailed_callback_does_not_abort_collection_and_reports_playwright(monkeypatch, tmp_path):
    class FakePage:
        def __init__(self):
            self.handlers = {}

        def on(self, event, callback):
            self.handlers[event] = callback
            if event == "requestfailed":
                callback(type("Request", (), {"method": "GET", "url": "http://viewer/missing.dae", "failure": "net::ERR_FAILED"})())

        def goto(self, *args, **kwargs):
            return None

        def wait_for_function(self, *args, **kwargs):
            return True

        def evaluate(self, script):
            if "robot_preview_lifecycle_state" in script:
                return "ready"
            if "__WORKCELL_ROBOT_PREVIEW_READY__" in script:
                return True
            return {"robot_preview_lifecycle_state": "ready", "web3d_readiness_state": "scene_ready"}

        def screenshot(self, path, full_page):
            Path(path).write_bytes(module.PNG_1X1)

    class FakeBrowser:
        def __init__(self):
            self.page = FakePage()

        def new_page(self, **kwargs):
            return self.page

        def close(self):
            return None

    class FakePlaywright:
        def __enter__(self):
            self.chromium = types.SimpleNamespace(launch=lambda **kwargs: FakeBrowser())
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

    fake_sync_api = types.SimpleNamespace(sync_playwright=lambda: FakePlaywright())
    monkeypatch.setitem(sys.modules, "playwright", types.SimpleNamespace(sync_api=fake_sync_api))
    monkeypatch.setitem(sys.modules, "playwright.sync_api", fake_sync_api)

    result = module.run_browser("http://viewer", tmp_path / "status.json", tmp_path / "shot.png", require=True)

    assert result["available"] is True
    assert result["method"] == "playwright"
    assert result["status"]["web3d_readiness_state"] == "scene_ready"
    assert result["failed_network_requests"] == ["GET http://viewer/missing.dae net::ERR_FAILED"]


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
    assert "rviz_parity.png" in text

WORKFLOW = ROOT / ".github/workflows/web3d-visual-acceptance.yml"


def test_web3d_visual_acceptance_workflow_exists():
    assert WORKFLOW.is_file()


def test_web3d_visual_acceptance_workflow_requires_browser_runtime():
    text = WORKFLOW.read_text(encoding="utf-8")
    assert "scripts/run_workcell_web3d_visual_acceptance.py" in text
    assert "--require-browser" in text
    assert "python3 -m playwright install --with-deps chromium" in text


def test_web3d_visual_acceptance_workflow_installs_viewer_node_dependencies_before_static_tests():
    text = WORKFLOW.read_text(encoding="utf-8")
    assert "actions/setup-node@v4" in text
    assert "node-version: '20'" in text
    assert "cache-dependency-path: workcell_studio_web/viewer/package-lock.json" in text
    assert "working-directory: workcell_studio_web/viewer" in text
    assert "npm ci" in text
    assert "npm run check:stale-bundle" in text
    assert text.index("npm ci") < text.index("Run static acceptance tests")


def test_web3d_visual_acceptance_workflow_installs_pinned_real_xacro_and_preflights():
    text = WORKFLOW.read_text(encoding="utf-8")
    assert "xacro==2.1.1" in text
    assert "importlib.metadata.version('xacro')" in text or 'importlib.metadata.version("xacro")' in text
    assert "discover_xacro_command()" in text
    assert "xacro-lite cannot satisfy supported-scenes acceptance" in text


def test_web3d_visual_acceptance_workflow_runs_supported_scene_matrix_before_browser():
    text = WORKFLOW.read_text(encoding="utf-8")
    assert "--all-supported-scenes" in text
    assert "--sequential" in text
    assert "--output build/web3d_supported_scenes_matrix.json" in text
    assert "python3 scripts/run_workcell_studio_scene_readiness_matrix.py" in text


def test_web3d_visual_acceptance_workflow_uploads_matrix_report_and_scene_artifacts():
    text = WORKFLOW.read_text(encoding="utf-8")
    assert "actions/upload-artifact@v4" in text
    assert "build/web3d_supported_scenes_matrix.json" in text
    assert "build/workcell_studio_web_scene/*.visual_acceptance.json" in text
    assert "build/workcell_studio_web_scene/*.png" in text
    assert "GITHUB_STEP_SUMMARY" in text
    assert "if-no-files-found: warn" in text
    for token in [
        "browser runtime method",
        "screenshot artifact name",
        "report artifact name",
        "web3d_supported_scenes_matrix.json",
    ]:
        assert token in text



def test_web3d_visual_acceptance_workflow_writes_blocked_matrix_before_preflight_and_static_failures():
    text = WORKFLOW.read_text(encoding="utf-8")
    assert "Initialize blocked matrix artifact" in text
    assert "'status': 'BLOCKED'" in text
    assert "Workflow preflight has not completed yet." in text
    assert 'MATRIX_FAILURE_REASON="Preflight real xacro discovery failed: $output"' in text
    assert 'MATRIX_FAILURE_REASON="Run static acceptance tests failed: $output"' in text
    assert "report.update(status='BLOCKED', failure_reason=os.environ['MATRIX_FAILURE_REASON'])" in text

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



def test_acceptance_script_defines_supported_scene_browser_matrix_order():
    assert module.SUPPORTED_SCENE_ACCEPTANCE_ORDER == (
        "ur5_2f_test",
        "ur5_3f_test",
        "suction_test",
        "ur10_2f_test",
        "ur3_suction_test",
        "ur5_airpick4_test",
        "ur5_2f_sorting_test",
        "ur5_2f_builder_pick_place_demo",
    )
    assert module.SEQUENTIAL_SCENE_SWITCH_ORDER == (
        "ur5_2f_test",
        "suction_test",
        "ur5_3f_test",
        "ur5_airpick4_test",
        "ur5_2f_test",
    )

def test_acceptance_script_supports_playwright_browser_status_path():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "from playwright.sync_api import sync_playwright" in text
    assert "p.chromium.launch" in text
    assert "window.__WORKCELL_VIEWER_STATUS__" in text
    assert "page.wait_for_function" in text
    assert "window.__WORKCELL_ROBOT_PREVIEW_READY__" in text
    assert "robot_preview_lifecycle_state" in text
    assert "validate_browser_status(status)" in text
    assert "EXPECTED_MESH_LOADED_COUNT = 18" in text
    assert "EXPECTED_REQUIRED_MESH_FAILED_COUNT = 0" in text
    assert "viewer_url:" in text
    assert "screenshot_path:" in text
    assert "report_path:" in text



def test_acceptance_script_waits_for_robot_preview_lifecycle_and_reports_diagnostics():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "window.__WORKCELL_ROBOT_PREVIEW_READY__" in text
    for state in ["idle", "loading_urdf", "loading_meshes", "ready", "failed"]:
        assert state in text
    for key in [
        "expected_visual_count",
        "completed_visual_count",
        "failed_visual_count",
        "loaded_link_count",
        "root_link_count",
        "missing_links",
        "disconnected_links",
        "duplicate_links",
        "missing_meshes",
        "final_state",
    ]:
        assert key in text
    assert "screenshot_before_ready" in text
    assert "rviz_parity.png" in text

def test_acceptance_script_preserves_canonical_mesh_count_expectations():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "EXPECTED_MESH_LOADED_COUNT = 18" in text
    assert "EXPECTED_REQUIRED_MESH_FAILED_COUNT = 0" in text
    assert "meshLoadedCount plus robot_loaded_visual_count expected {EXPECTED_MESH_LOADED_COUNT}" in text
    assert "requiredMeshFailedCount expected {EXPECTED_REQUIRED_MESH_FAILED_COUNT}" in text


def test_acceptance_script_requires_viewer_side_resolved_tool_chain_distances():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "REQUIRED_VIEWER_RESOLVED_DISTANCE_PAIRS" in text
    assert "REQUIRED_RENDERED_MESH_ADJACENT_PAIRS" in text
    for pair in [
        "wrist_3_link -> tool0",
        "tool0 -> gripper_base_link",
        "wrist_3_link -> gripper_base_link",
    ]:
        assert pair in text
        assert pair not in module.REQUIRED_RENDERED_MESH_ADJACENT_PAIRS
    assert "viewer_resolved_distances_m" in text
    assert "resolved_distances_m" in text
    assert "resolved_frame_positions" in text
    assert "resolvedFramePositions" in text
    assert "frame_diagnostics" in text
    assert "frameDiagnostics" in text
    assert "browser viewer resolved distance {pair} expected <=" in text


def test_viewer_status_reports_loaded_mesh_vs_fallback_geometry():
    text = (ROOT / "workcell_studio_web/viewer/viewer.js").read_text(encoding="utf-8")
    assert "renderedObjectStatuses" in text
    assert "rendered_object_statuses" in text
    assert "mesh_loaded: renderStatus === 'mesh_loaded'" in text
    assert "fallback_visible" in text
    assert "required_mesh_failed_debug_fallback" in text


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
        "resolvedFramePositions": _valid_resolved_frame_positions(),
        "frameDiagnostics": _valid_frame_diagnostics(),
        "renderedMeshDiagnostics": _valid_rendered_mesh_diagnostics(),
        **_valid_robot_hierarchy_fields(),
    }
    snake_status = {
        "mesh_loaded_count": 18,
        "required_mesh_failed_count": 0,
        "viewer_resolved_distances_m": distance_pairs,
        "resolved_frame_positions": _valid_resolved_frame_positions(),
        "frame_diagnostics": _valid_frame_diagnostics(),
        "rendered_mesh_diagnostics": _valid_rendered_mesh_diagnostics(),
        **_valid_robot_hierarchy_fields(),
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
        "resolved_frame_positions": _valid_resolved_frame_positions(),
        "frame_diagnostics": _valid_frame_diagnostics(),
        "rendered_mesh_diagnostics": _valid_rendered_mesh_diagnostics(),
    }

    errors = module.validate_browser_status(status)
    assert any(missing_pair in error for error in errors)


def test_browser_status_validator_rejects_missing_viewer_side_distance_map():
    status = {
        "mesh_loaded_count": 18,
        "required_mesh_failed_count": 0,
        "resolved_frame_positions": _valid_resolved_frame_positions(),
        "frame_diagnostics": _valid_frame_diagnostics(),
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
                "category": "tool" if link in {"tool0", "gripper_base_link"} else "robot",
                "render_status": "mesh_loaded",
                "mesh_loaded": True,
                "fallback_visible": False,
                "loaded_mesh_bounding_box_center": {"x": index * spacing, "y": 0.0, "z": 0.0},
                "visual_wrapper_world_position": {"x": index * spacing, "y": 0.0, "z": 0.0},
                "workcell_web_render_pose_mode": "assembled_urdf_hierarchy",
                "baked_world_visual_pose": {"xyz": [index * spacing, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
            }
        )
    diagnostics.append(_valid_table_diagnostic())
    diagnostics.append(_valid_camera_diagnostic())
    return diagnostics


def _valid_table_diagnostic():
    return {
        "id": "workbench",
        "object_id": "workbench",
        "object_name": "M1 workbench",
        "category": "table",
        "link_name": "workbench",
        "render_status": "mesh_loaded",
        "mesh_loaded": True,
        "fallback_visible": False,
        "mesh_uri": "assets/workbench.stl",
        "support_surface_kind": "table_surface",
        "expected_dimensions_m": {"x": 1.20, "y": 0.80, "z": 0.05},
        "mesh_local_scale": {"x": 1.0, "y": 1.0, "z": 1.0},
        "bounding_box_size": {"x": 1.20, "y": 0.80, "z": 0.05},
        "bounding_box_center": {"x": 0.40, "y": 0.0, "z": 0.75},
        "inferred_up_axis": {"x": 0.0, "y": 0.0, "z": 1.0},
    }




def _workbench_body_diagnostic(with_height=True):
    diagnostic = _valid_table_diagnostic()
    diagnostic.update(
        {
            "id": "workbench_body",
            "object_id": "workbench_body",
            "object_name": "M1 workbench body",
            "link_name": "workbench_body",
            "support_surface_kind": "workbench_body",
            "expected_dimensions_m": {"x": 1.20, "y": 0.80, "z": 0.90},
            "bounding_box_size": {"x": 1.20, "y": 0.80, "z": 0.90},
            "loaded_mesh_bounding_box_size": {"x": 1.20, "y": 0.80, "z": 0.90},
            "bounding_box_center": {"x": 0.40, "y": 0.0, "z": 0.45},
            "loaded_mesh_bounding_box_center": {"x": 0.40, "y": 0.0, "z": 0.45},
        }
    )
    diagnostic.pop("inferred_up_axis", None)
    if with_height:
        diagnostic["top_surface_z_m"] = 0.90
    return diagnostic


def _valid_camera_diagnostic():
    return {
        "id": "camera_realsense",
        "object_id": "camera_realsense",
        "display_name": "Intel RealSense camera",
        "category": "camera",
        "link_name": "camera_link",
        "render_status": "mesh_loaded",
        "mesh_loaded": True,
        "fallback_visible": False,
        "mesh_uri": "assets/realsense.dae",
        "expected_dimensions_m": {"x": 0.08, "y": 0.08, "z": 0.06},
        "loaded_mesh_bounding_box_size": {"x": 0.08, "y": 0.08, "z": 0.06},
        "bounding_box_size": {"x": 0.08, "y": 0.08, "z": 0.06},
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


def _valid_resolved_frame_positions():
    return {
        "wrist_3_link": {"x": 0.30, "y": 0.0, "z": 0.0},
        "tool0": {"x": 0.301, "y": 0.0, "z": 0.0},
        "gripper_base_link": {"x": 0.40, "y": 0.0, "z": 0.0},
    }


def _valid_frame_diagnostics():
    return [
        {"frame_name": "wrist_3_link", "resolved": True},
        {"frame_name": "tool0", "resolved": True},
        {"frame_name": "gripper_base_link", "resolved": True},
    ]



def _valid_robot_hierarchy_fields():
    links = list(module.REQUIRED_BROWSER_MATRIX_LINKS)
    identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    link_matrices = {link: {"matrix_world": identity, "parent_link_name": links[index - 1] if index else ""} for index, link in enumerate(links)}
    visual_matrices = [
        {"link_name": link, "visual_index": 0, "matrix_world": identity}
        for link in links
        if link != "tool0"
    ]
    return {
        "robot_render_mode": "expanded_urdf_loader",
        "robot_preview_loaded": True,
        "robot_missing_meshes": [],
        "robot_loaded_visual_count": 18,
        "robot_expected_visual_count": 18,
        "robot_completed_visual_count": 18,
        "robot_failed_visual_count": 0,
        "robot_loaded_link_count": len(links),
        "robot_root_link_count": 1,
        "robot_root_links": ["base_link_inertia"],
        "robot_disconnected_links": [],
        "robot_duplicate_links": [],
        "robot_preview_lifecycle_state": "ready",
        "robot_preview_canonical_fallback_used": False,
        "physical_assembly_root_count": 1,
        "physical_fit_included_robot_preview": True,
        "physical_assembly_bounds": {"min": {"x": -0.5, "y": -0.4, "z": 0.0}, "max": {"x": 0.8, "y": 0.4, "z": 1.2}},
        "final_physical_fit_bounds": {"min": {"x": -0.6, "y": -0.5, "z": -0.1}, "max": {"x": 0.9, "y": 0.5, "z": 1.3}},
        "robot_hierarchy_links": links,
        "robot_hierarchy_missing_links": [],
        "robot_hierarchy_missing_parents": [],
        "robot_hierarchy_mesh_count": 18,
        "robot_link_world_matrices": link_matrices,
        "robot_visual_wrapper_world_matrices": visual_matrices,
    }

def _valid_browser_status_with_rendered_diagnostics():
    return {
        "meshLoadedCount": 18,
        "requiredMeshFailedCount": 0,
        "viewer_resolved_distances_m": _valid_viewer_distances(),
        "resolvedFramePositions": _valid_resolved_frame_positions(),
        "frameDiagnostics": _valid_frame_diagnostics(),
        "renderedMeshDiagnostics": _valid_rendered_mesh_diagnostics(),
        **_valid_robot_hierarchy_fields(),
    }


def test_browser_status_validator_rejects_missing_rendered_mesh_diagnostics():
    status = {
        "mesh_loaded_count": 18,
        "required_mesh_failed_count": 0,
        "viewer_resolved_distances_m": _valid_viewer_distances(),
        "resolved_frame_positions": _valid_resolved_frame_positions(),
        "frame_diagnostics": _valid_frame_diagnostics(),
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
        "resolved_frame_positions": _valid_resolved_frame_positions(),
        "frame_diagnostics": _valid_frame_diagnostics(),
        "rendered_mesh_diagnostics": _valid_rendered_mesh_diagnostics(),
        **_valid_robot_hierarchy_fields(),
    }

    assert module.validate_browser_status(status) == []


def test_browser_status_validator_accepts_tool0_in_frame_diagnostics_but_not_rendered_mesh_diagnostics():
    status = _valid_browser_status_with_rendered_diagnostics()
    status["renderedMeshDiagnostics"] = [
        diagnostic for diagnostic in status["renderedMeshDiagnostics"] if diagnostic.get("link_name") != "tool0"
    ]

    assert module.validate_browser_status(status) == []


def test_browser_status_validator_rejects_missing_tool0_frame_diagnostics():
    status = _valid_browser_status_with_rendered_diagnostics()
    status["frameDiagnostics"] = [
        diagnostic for diagnostic in status["frameDiagnostics"] if diagnostic.get("frame_name") != "tool0"
    ]

    errors = module.validate_browser_status(status)

    assert any("frameDiagnostics/frame_diagnostics must include tool0" in error for error in errors)


def test_browser_status_validator_rejects_missing_tool0_resolved_frame_position():
    status = _valid_browser_status_with_rendered_diagnostics()
    status["resolvedFramePositions"] = {
        key: value for key, value in status["resolvedFramePositions"].items() if key != "tool0"
    }

    errors = module.validate_browser_status(status)

    assert any("resolvedFramePositions/resolved_frame_positions must include tool0" in error for error in errors)


def test_browser_status_validator_rejects_implausible_wrist_to_tool0_distance():
    status = _valid_browser_status_with_rendered_diagnostics()
    status["viewer_resolved_distances_m"]["wrist_3_link -> tool0"] = 99.0

    errors = module.validate_browser_status(status)

    assert any("wrist_3_link -> tool0" in error and "expected <=" in error for error in errors)


def test_browser_status_validator_rejects_implausible_tool0_to_gripper_distance():
    status = _valid_browser_status_with_rendered_diagnostics()
    status["viewer_resolved_distances_m"]["tool0 -> gripper_base_link"] = 99.0

    errors = module.validate_browser_status(status)

    assert any("tool0 -> gripper_base_link" in error and "expected <=" in error for error in errors)


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


def test_browser_status_validator_rejects_required_product_visible_fallbacks():
    status = _valid_browser_status_with_rendered_diagnostics()
    for diagnostic in status["renderedMeshDiagnostics"]:
        if diagnostic["link_name"] == "gripper_base_link":
            diagnostic["render_status"] = "required_mesh_failed_debug_fallback"
            diagnostic["fallback_visible"] = True

    errors = module.validate_browser_status(status)

    assert any("required product gripper_base_link" in error and "visible fallback/debug geometry" in error for error in errors)


def test_browser_status_validator_rejects_table_expected_mesh_rendered_as_primitive_fallback():
    status = _valid_browser_status_with_rendered_diagnostics()
    for diagnostic in status["renderedMeshDiagnostics"]:
        if diagnostic.get("category") == "table":
            diagnostic["render_status"] = "primitive_fallback"
            diagnostic["fallback_visible"] = True

    errors = module.validate_browser_status(status)

    assert any("table/workbench workbench expected a loaded mesh" in error for error in errors)


def test_browser_status_validator_rejects_table_non_uniform_scale_from_expected_dimensions():
    status = _valid_browser_status_with_rendered_diagnostics()
    for diagnostic in status["renderedMeshDiagnostics"]:
        if diagnostic.get("category") == "table":
            diagnostic["mesh_local_scale"] = {"x": 1.0, "y": 0.5, "z": 2.0}

    errors = module.validate_browser_status(status)

    assert any("non-uniform mesh scale derived from expected_dimensions_m" in error for error in errors)




def test_browser_status_validator_rejects_assembled_hierarchy_visual_wrapper_explosion():
    status = _valid_browser_status_with_rendered_diagnostics()
    for diagnostic in status["renderedMeshDiagnostics"]:
        if diagnostic.get("link_name") == "wrist_3_link":
            diagnostic["visual_wrapper_world_position"] = {"x": 2.0, "y": 0.0, "z": 0.0}

    errors = module.validate_browser_status(status)

    assert any("wrist_2_link -> wrist_3_link" in error and "visual wrapper" in error for error in errors)


def test_browser_status_validator_rejects_exploded_loaded_mesh_bounds_even_when_wrappers_are_connected():
    status = _valid_browser_status_with_rendered_diagnostics()
    for diagnostic in status["renderedMeshDiagnostics"]:
        if diagnostic.get("link_name") == "forearm_link":
            diagnostic["loaded_mesh_bounding_box_center"] = {"x": 3.0, "y": 0.0, "z": 0.0}

    errors = module.validate_browser_status(status)

    assert any("upper_arm_link -> forearm_link loaded mesh bounds center expected <=" in error for error in errors)


def test_browser_status_validator_rejects_camera_expected_mesh_rendered_as_primitive_fallback():
    status = _valid_browser_status_with_rendered_diagnostics()
    for diagnostic in status["renderedMeshDiagnostics"]:
        if diagnostic.get("category") == "camera":
            diagnostic["render_status"] = "primitive_fallback"
            diagnostic["mesh_loaded"] = False
            diagnostic["fallback_visible"] = True

    errors = module.validate_browser_status(status)

    assert any("camera/Realsense camera_link must remain mesh-backed" in error for error in errors)

def test_browser_status_validator_rejects_oversized_camera_mesh_in_fit_bounds():
    status = _valid_browser_status_with_rendered_diagnostics()
    status["renderedMeshDiagnostics"].append(
        {
            "id": "camera_realsense",
            "object_id": "camera_realsense",
            "display_name": "Intel RealSense camera",
            "category": "camera",
            "link_name": "camera_link",
            "render_status": "mesh_loaded",
            "mesh_loaded": True,
            "exclude_from_fit_bounds": False,
            "expected_dimensions_m": {"x": 0.08, "y": 0.08, "z": 0.06},
            "loaded_mesh_bounding_box_size": {"x": 4.0, "y": 4.0, "z": 3.0},
        }
    )

    errors = module.validate_browser_status(status)

    assert any("camera/Realsense camera_link loaded mesh bounds are oversized" in error for error in errors)


def test_table_horizontal_accepts_tabletop_with_thin_z():
    assert module._table_horizontal_errors({"renderedMeshDiagnostics": [_valid_table_diagnostic()]}) == []


def test_table_horizontal_rejects_tabletop_with_large_z():
    diagnostic = _valid_table_diagnostic()
    diagnostic["bounding_box_size"] = {"x": 1.20, "y": 0.80, "z": 0.60}

    errors = module._table_horizontal_errors({"renderedMeshDiagnostics": [diagnostic]})

    assert any("thin tabletop Z" in error for error in errors)


def test_table_horizontal_accepts_workbench_body_only_with_support_height_metadata():
    assert module._table_horizontal_errors({"renderedMeshDiagnostics": [_workbench_body_diagnostic(with_height=True)]}) == []


def test_table_horizontal_rejects_workbench_body_without_support_height_metadata():
    errors = module._table_horizontal_errors({"renderedMeshDiagnostics": [_workbench_body_diagnostic(with_height=False)]})

    assert any("missing finite top/support height metadata" in error for error in errors)


def test_table_horizontal_rejects_missing_support_surface_kind_metadata():
    diagnostic = _valid_table_diagnostic()
    diagnostic.pop("support_surface_kind", None)

    errors = module._table_horizontal_errors({"renderedMeshDiagnostics": [diagnostic]})

    assert any("missing explicit support-surface kind metadata" in error for error in errors)


def test_browser_status_validator_rejects_xacro_lite_render_mode_for_canonical_scene():
    status = _valid_browser_status_with_rendered_diagnostics()
    status["robot_render_mode"] = "xacro_lite_expanded"
    errors = module.validate_browser_status(status)
    assert any("robot_render_mode=expanded_urdf_loader" in error and "xacro_lite_expanded" not in error for error in errors)


def test_browser_status_validator_requires_single_physical_assembly_root():
    status = _valid_browser_status_with_rendered_diagnostics()
    status["physical_assembly_root_count"] = 2
    errors = module.validate_browser_status(status)
    assert any("physical_assembly_root_count" in error and "got 2" in error for error in errors)


def test_browser_status_validator_requires_physical_fit_to_include_robot_preview():
    status = _valid_browser_status_with_rendered_diagnostics()
    status["physical_fit_included_robot_preview"] = False
    errors = module.validate_browser_status(status)
    assert any("physical_fit_included_robot_preview" in error and "got False" in error for error in errors)


def test_browser_status_validator_rejects_malformed_nan_infinite_and_zero_volume_bounds():
    bad_values = [None, {}, {"min": [0, 0, 0], "max": [0, 1, 1]}, {"min": [0, 0, 0], "max": [float("nan"), 1, 1]}, {"min": [0, 0, 0], "max": [float("inf"), 1, 1]}]
    for bad in bad_values:
        status = _valid_browser_status_with_rendered_diagnostics()
        status["physical_assembly_bounds"] = bad
        errors = module.validate_browser_status(status)
        assert any("physical_assembly_bounds" in error and "got" in error for error in errors)


def test_browser_status_validator_does_not_rewrite_or_union_fit_bounds():
    status = _valid_browser_status_with_rendered_diagnostics()
    status["final_physical_fit_bounds"] = {"min": {"x": -0.1, "y": -0.1, "z": 0.1}, "max": {"x": 0.1, "y": 0.1, "z": 0.2}}
    before = dict(status["final_physical_fit_bounds"])
    module.validate_browser_status(status)
    assert status["final_physical_fit_bounds"] == before


def test_require_browser_rejects_1x1_placeholder_screenshot(tmp_path):
    shot = tmp_path / "placeholder.png"
    shot.write_bytes(module.PNG_1X1)
    status_json = tmp_path / "status.json"
    status_json.write_text("{}", encoding="utf-8")
    errors = module.require_browser_artifact_errors({"available": True, "method": "playwright"}, "started", shot, status_json)
    assert any("screenshot dimensions" in error and "(1, 1)" in error for error in errors)


def test_physical_fit_accepts_camel_case_diagnostics():
    status = _valid_browser_status_with_rendered_diagnostics()
    status.pop("physical_assembly_root_count")
    status.pop("physical_fit_included_robot_preview")
    status.pop("physical_assembly_bounds")
    status.pop("final_physical_fit_bounds")
    status.update({
        "physicalAssemblyRootCount": 1,
        "physicalFitIncludedRobotPreview": True,
        "physicalAssemblyBounds": {"min": [-0.5, -0.4, 0.0], "max": [0.8, 0.4, 1.2]},
        "finalPhysicalFitBounds": {"min": [-0.6, -0.5, -0.1], "max": [0.9, 0.5, 1.3]},
    })
    assert module.validate_browser_status(status) == []


def _expanded_urdf_status(robot_count=16, mesh_count=2, bounds_marker=-1):
    return {
        **_valid_robot_hierarchy_fields(),
        "robot_render_mode": "expanded_urdf_loader",
        "mesh_loaded_count": mesh_count,
        "required_mesh_failed_count": 0,
        "robot_expected_visual_count": robot_count,
        "robot_completed_visual_count": robot_count,
        "robot_loaded_visual_count": robot_count,
        "robot_failed_visual_count": 0,
        "robot_mesh_callbacks_complete": True,
        "robot_collada_mesh_diagnostics": [{"uri": f"package://robotiq_85_description/meshes/visual/{name}"} for name in module.REQUIRED_ROBOTIQ_MESH_BASENAMES],
        "viewer_resolved_distances_m": {"wrist_3_link -> tool0": 0.01, "tool0 -> gripper_base_link": 0.1, "wrist_3_link -> gripper_base_link": 0.11},
        "resolved_frame_positions": _valid_resolved_frame_positions(),
        "frame_diagnostics": _valid_frame_diagnostics(),
        "rendered_mesh_diagnostics": _valid_rendered_mesh_diagnostics(),
        "physical_assembly_bounds": {"min": {"x": 0, "y": 0, "z": 0}, "max": {"x": 1, "y": 1, "z": 1}},
        "final_physical_fit_bounds": {"min": {"x": bounds_marker, "y": bounds_marker, "z": bounds_marker}, "max": {"x": bounds_marker+2, "y": bounds_marker+2, "z": bounds_marker+2}},
    }


def test_acceptance_rejects_seven_robot_visuals_and_accepts_sixteen_plus_two():
    bad = _expanded_urdf_status(robot_count=7, mesh_count=2)
    good = _expanded_urdf_status(robot_count=16, mesh_count=2)
    assert any('robot_loaded_visual_count expected 16' in e for e in module.validate_browser_status(bad))
    assert not any('robot_loaded_visual_count expected 16' in e for e in module.validate_browser_status(good))


def test_acceptance_requires_all_robotiq_collada_diagnostics():
    status = _expanded_urdf_status(robot_count=16, mesh_count=2)
    status['robot_collada_mesh_diagnostics'] = [{"uri": "package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae"}]
    errors = module.validate_browser_status(status)
    assert any('robot_collada_mesh_diagnostics missing Robotiq mesh basenames' in e for e in errors)


def test_acceptance_no_longer_rewrites_physical_fit_bounds():
    assert not hasattr(module, 'normalize_physical_fit_bounds')


def test_supported_scene_registry_includes_robot_tool_and_required_capabilities():
    catalog = yaml.safe_load((ROOT / "scenes/supported_scenes.yaml").read_text(encoding="utf-8"))
    registered = {row["scene_name"]: row for row in catalog["scenes"]}
    for scene_id in ["ur5_2f_test", "ur5_3f_test", "suction_test", "ur10_2f_test", "ur3_suction_test"]:
        row = registered[scene_id]
        assert row["robot"]
        assert row["tool"]
        assert row["required_capabilities"]
        assert "fake_hardware_launch" in row["required_capabilities"]


def test_supported_scene_reproducibility_gate_reports_blocked_with_reason(monkeypatch, tmp_path):
    entry = type("Entry", (), {
        "scene_name": "blocked_scene", "scene_path": "scenes/blocked_scene", "package_name": "blocked_scene",
        "robot": "ur5", "tool": "robotiq_2f", "required_capabilities": ("fake_hardware_launch",),
        "status": "blocked", "enabled": True, "known_blocker": "explicit ROS workspace blocker",
        "authoring_files": ("environment.yaml",), "fake_hardware_launch_command": "ros2 launch blocked_scene demo.launch.py use_fake_hardware:=true",
    })()
    monkeypatch.setattr(module, "_load_supported_entries", lambda catalog=None: ([entry], []))
    report = module.run_supported_scene_reproducibility_gate(output=tmp_path / "report.json")
    assert report["counts"] == {"PASS": 0, "FAIL": 0, "BLOCKED": 1}
    assert report["status"] == "PASS"
    assert report["scenes"][0]["blocker_reason"] == "explicit ROS workspace blocker"


def test_supported_scene_reproducibility_gate_reports_fail_and_nonzero_reason(monkeypatch, tmp_path):
    scene = ROOT / "scenes/fail_fixture"
    entry = type("Entry", (), {
        "scene_name": "fail_fixture", "scene_path": "scenes/fail_fixture", "package_name": "fail_fixture",
        "robot": "ur5", "tool": "robotiq_2f", "required_capabilities": ("fake_hardware_launch",),
        "status": "supported", "enabled": True, "known_blocker": "",
        "authoring_files": ("environment.yaml", "cell_definition.yaml"),
        "fake_hardware_launch_command": "ros2 launch fail_fixture demo.launch.py use_fake_hardware:=true",
    })()
    monkeypatch.setattr(module, "_load_supported_entries", lambda catalog=None: ([entry], []))
    report = module.run_supported_scene_reproducibility_gate(output=tmp_path / "report.json")
    row = report["scenes"][0]
    assert row["status"] == "FAIL"
    assert report["status"] == "FAIL"
    assert "required_source_files" in row["failure_reason"]


def test_supported_scene_reproducibility_gate_reports_pass(monkeypatch, tmp_path):
    scene = tmp_path / "repo" / "scenes" / "pass_scene"
    (scene / "generated").mkdir(parents=True)
    for rel in ["environment.yaml", "cell_definition.yaml", "layout/workcell_studio_layout.yaml", "launch/demo.launch.py", "urdf/scene.urdf.xacro"]:
        (scene / rel).parent.mkdir(parents=True, exist_ok=True)
        (scene / rel).write_text("ok\n", encoding="utf-8")
    (scene / "generated/generated_workcell_summary.json").write_text(json.dumps({"robot": {"capability": "ur5"}, "end_effector": {"capability": "robotiq_2f"}, "grasp_strategy": {"id": "finger"}}), encoding="utf-8")
    (scene / "generated/scene_package_readiness.json").write_text("{}", encoding="utf-8")
    generated = tmp_path / "repo" / "build/workcell_studio_supported_scene_reproducibility/pass_scene"
    (generated / "generated").mkdir(parents=True)
    for rel in ["launch/demo.launch.py", "urdf/scene.urdf.xacro"]:
        (generated / rel).parent.mkdir(parents=True, exist_ok=True)
        (generated / rel).write_text("ok\n", encoding="utf-8")
    (generated / "generated/generated_workcell_summary.json").write_text(json.dumps({"robot": {"capability": "ur5"}, "end_effector": {"capability": "robotiq_2f"}, "grasp_strategy": {"id": "finger"}}), encoding="utf-8")
    (generated / "generated/scene_package_readiness.json").write_text("{}", encoding="utf-8")
    web = tmp_path / "web.json"
    web.write_text(json.dumps({"render_ownership_summary": {"unknown_physical_owners": 0, "duplicate_primary_identities": 0}}), encoding="utf-8")
    entry = type("Entry", (), {
        "scene_name": "pass_scene", "scene_path": "scenes/pass_scene", "package_name": "pass_scene",
        "robot": "ur5", "tool": "robotiq_2f", "required_capabilities": ("fake_hardware_launch",),
        "status": "supported", "enabled": True, "known_blocker": "",
        "authoring_files": ("environment.yaml", "cell_definition.yaml"),
        "fake_hardware_launch_command": "ros2 launch pass_scene demo.launch.py use_fake_hardware:=true",
    })()
    monkeypatch.setattr(module, "REPO_ROOT", tmp_path / "repo")
    monkeypatch.setattr(module, "BUILD_ROOT", tmp_path)
    monkeypatch.setattr(module, "run_step", lambda cmd: {"command": cmd, "returncode": 0, "stdout": "", "stderr": ""})
    monkeypatch.setattr(module, "_load_supported_entries", lambda catalog=None: ([entry], []))
    monkeypatch.setattr(module, "_check_render_ownership", lambda path: {"status": "PASS", "summary": {}})
    report = module.run_supported_scene_reproducibility_gate(output=tmp_path / "report.json")
    assert report["counts"]["PASS"] == 1
    assert report["status"] == "PASS"
