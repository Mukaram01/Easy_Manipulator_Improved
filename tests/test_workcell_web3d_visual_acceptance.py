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
    links = [
        "base_link_inertia",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
        "tool0",
        "gripper_base_link",
    ]
    return {
        "robot_render_mode": "assembled_urdf_hierarchy",
        "robot_hierarchy_links": links,
        "robot_hierarchy_missing_links": [],
        "robot_hierarchy_missing_parents": [],
        "robot_hierarchy_mesh_count": 18,
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
