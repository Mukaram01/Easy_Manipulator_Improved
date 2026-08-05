from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EXPORTER_PATH = ROOT / "scripts/export_workcell_studio_web_scene.py"
PORTABILITY_PATH = ROOT / "scripts/workcell_studio_visual_artifact_portability.py"
FRESHENER_PATH = ROOT / "scripts/ensure_workcell_studio_web_scene_fresh.py"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _repo_layout(tmp_path: Path) -> tuple[Path, Path, Path]:
    repo = tmp_path / "repo"
    for folder in ("assets", "scripts", "scenes"):
        (repo / folder).mkdir(parents=True, exist_ok=True)
    scene = repo / "scenes" / "portable_scene"
    scene.mkdir()
    output = repo / "build" / "workcell_studio_web_scene" / "portable_scene.web_scene.json"
    return repo, scene, output


def _payload() -> dict:
    staged_robot = "build/workcell_studio_web_scene/assets/portable_scene/ur_description/meshes/ur5/visual/base.dae"
    staged_camera = "build/workcell_studio_web_scene/assets/portable_scene/realsense2_description/meshes/d435.dae"
    return {
        "schema_version": "workcell_studio_web_scene/v1",
        "scene_id": "portable_scene",
        "metadata": {},
        "viewer_summary": {},
        "robots": [
            {
                "id": "generated_urdf::base_link::visual_0",
                "package_uri": "package://ur_description/meshes/ur5/visual/base.dae",
                "mesh_uri": staged_robot,
                "mesh_url": staged_robot,
                "mesh_staged_path": staged_robot,
                "resolved_source_path": "/home/alice/workcell_ws/src/easy_manipulation_deployment/assets/robots/ur_description/meshes/ur5/visual/base.dae",
                "original_source_path": "/home/alice/workcell_ws/src/easy_manipulation_deployment/assets/robots/ur_description/meshes/ur5/visual/base.dae",
                "render_policy": "diagnostic_only",
                "render_policy_reason": "expanded_urdf_loader_owns_robot_tool_visuals",
            }
        ],
        "tools": [],
        "assets": [
            {
                "id": "capability_metadata_row",
                "source_kind": "generated_preview",
                "render_policy": "diagnostic_only",
            }
        ],
        "sensors": [
            {
                "id": "generated_urdf::camera_link::visual_17",
                "package_uri": "package://realsense2_description/meshes/d435.dae",
                "mesh_uri": staged_camera,
                "mesh_staged_path": staged_camera,
                "resolved_source_path": "C:\\Users\\builder\\workcell\\d435.dae",
            }
        ],
        "zones": [],
        "frames": [
            {
                "id": "urdf_frame_anchor_tool0",
                "link": "tool0",
                "type": "transform_anchor",
                "role": "transform_anchor",
                "category": "frame",
                "render_expected": False,
                "mesh_available": False,
                "mesh_load_required": False,
            }
        ],
    }


def test_visual_artifact_normalizer_removes_machine_paths_and_classifies_every_row(tmp_path):
    portability = _load("workcell_studio_visual_artifact_portability_test", PORTABILITY_PATH)
    _repo, scene, output = _repo_layout(tmp_path)
    payload = _payload()

    portability.normalize_web_scene_payload(
        payload,
        scene_dir=scene,
        output_path=output,
        stage_assets=True,
    )

    robot = payload["robots"][0]
    assert robot["package_uri"] == "package://ur_description/meshes/ur5/visual/base.dae"
    assert robot["authoritative_mesh_reference"] == robot["package_uri"]
    assert robot["mesh_reference_authority"] == "package_uri"
    assert robot["repo_relative_staged_path"].startswith("build/workcell_studio_web_scene/assets/portable_scene/")
    assert robot["mesh_uri"] == robot["mesh_url"] == robot["mesh_staged_path"] == robot["repo_relative_staged_path"]
    assert "resolved_source_path" not in robot
    assert robot["resolved_source_path_stale"] is False
    assert not str(robot.get("original_source_path", "")).startswith("/home/")
    assert robot["visual_artifact_classification"] == "portable_mesh_visual_diagnostic"

    camera = payload["sensors"][0]
    assert "resolved_source_path" not in camera
    assert camera["role"] == "camera"
    assert camera["category"] == "camera"
    assert camera["semantic_role"] == "configured_camera"
    assert camera["visual_artifact_classification"] == "portable_physical_visual"

    metadata_row = payload["assets"][0]
    assert metadata_row["visual_artifact_classification"] == "intentional_nonvisual_semantic_record"
    assert metadata_row["intentional_exclusion_reason"] == "semantic_capability_record_has_no_independent_visual_geometry"
    assert metadata_row["render_expected"] is False
    assert metadata_row["mesh_load_required"] is False
    assert metadata_row["selectable"] is False

    tool0 = payload["frames"][0]
    assert tool0["visual_artifact_classification"] == "intentional_non_mesh_transform_anchor"
    assert tool0["intentional_exclusion_reason"] == "meshless_transform_anchor_preserved_for_tf_and_tool_mount"
    assert tool0["render_policy"] == "diagnostic_only"
    assert tool0["render_owner"] == "diagnostic_helper"

    summary = payload["metadata"]["visual_artifact_portability"]
    assert summary["schema_version"] == "workcell_studio_visual_artifact_portability/v1"
    assert summary["status"] == "PASS"
    assert summary["stale_resolved_source_path"] == 0
    assert summary["unknown_role_no_fallback"] == 0
    assert summary["intentional_non_mesh_exclusion_count"] == 2
    assert {row["id"] for row in summary["intentional_non_mesh_exclusions"]} == {
        "capability_metadata_row",
        "urdf_frame_anchor_tool0",
    }
    assert payload["viewer_summary"]["stale_resolved_source_path"] == 0
    assert payload["viewer_summary"]["unknown_role_no_fallback"] == 0
    assert "/home/alice/" not in json.dumps(payload)
    assert "C:\\\\Users\\\\builder" not in json.dumps(payload)


def test_exporter_wrapper_applies_portability_to_direct_python_call(tmp_path, monkeypatch):
    exporter = _load("export_workcell_studio_web_scene_portability_test", EXPORTER_PATH)
    _repo, scene, output = _repo_layout(tmp_path)
    monkeypatch.setattr(exporter, "_ORIGINAL_BUILD_WEB_SCENE", lambda *args, **kwargs: _payload())

    payload = exporter.build_web_scene(scene, stage_assets=True, output_path=output)

    summary = payload["metadata"]["visual_artifact_portability"]
    assert summary["status"] == "PASS"
    assert summary["package_uri_authoritative_count"] == 2
    assert summary["repository_relative_staged_path_count"] == 2
    assert summary["stale_resolved_source_path"] == 0
    assert summary["unknown_role_no_fallback"] == 0


def test_exporter_wrapper_preserves_public_api_and_freshener_revision_inputs():
    exporter_source = EXPORTER_PATH.read_text(encoding="utf-8")
    freshener_source = FRESHENER_PATH.read_text(encoding="utf-8")

    assert "import export_workcell_studio_web_scene_impl as _impl" in exporter_source
    assert "normalize_web_scene_payload" in exporter_source
    assert "_ORIGINAL_BUILD_WEB_SCENE" in exporter_source
    assert "export_workcell_studio_web_scene_impl.py" in freshener_source
    assert "workcell_studio_visual_artifact_portability.py" in freshener_source


def test_canonical_scene_direct_export_has_zero_portability_unknowns():
    exporter = _load("export_workcell_studio_web_scene_canonical_portability_test", EXPORTER_PATH)
    payload = exporter.build_web_scene(ROOT / "scenes/ur5_2f_test", stage_assets=False)
    summary = payload["metadata"]["visual_artifact_portability"]

    assert summary["status"] == "PASS"
    assert summary["stale_resolved_source_path"] == 0
    assert summary["unknown_role_no_fallback"] == 0
    assert all(
        item.get("visual_artifact_classification")
        for section in ("robots", "tools", "assets", "sensors", "zones", "frames")
        for item in payload.get(section, [])
        if isinstance(item, dict)
    )
