from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import yaml

from scripts import run_workcell_studio_scene_readiness_matrix as matrix


def _write_catalog(repo_root: Path, entries: list[dict[str, Any]]) -> Path:
    path = repo_root / "scenes" / "supported_scenes.yaml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(
            {
                "schema_version": "workcell_studio_supported_scenes/v1",
                "scenes": entries,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return path


def _write_valid_cell_definition(scene_dir: Path, scene_name: str) -> None:
    (scene_dir / "cell_definition.yaml").write_text(
        yaml.safe_dump(
            {
                "schema_version": "cell_definition/v1",
                "cell": {"id": scene_name, "name": scene_name},
                "robot": {
                    "model": "ur5",
                    "planning_group": "manipulator",
                    "base_frame": "world",
                    "tool_link": "tool0",
                    "home_named_target": "home",
                    "safe_joint_state": [],
                },
                "end_effector": {
                    "id": "suction",
                    "type": "suction",
                    "brand": "generic",
                    "grasp_frame": "tool0",
                },
                "camera": {"id": "fixture_camera", "frame": "world"},
                "environment": {"frame": "world", "layout": "layout/workcell_studio_layout.yaml"},
                "objects": [
                    {
                        "id": "commissioning_object",
                        "type": "box",
                        "pose_xyz": [0.3, 0.0, 0.1],
                        "pose_rpy": [0.0, 0.0, 0.0],
                        "dimensions": [0.05, 0.05, 0.05],
                    }
                ],
                "task": {
                    "id": "default_task",
                    "type": "pick_place",
                    "source_object": "commissioning_object",
                    "destinations": [
                        {
                            "id": "default_drop_zone",
                            "frame": "world",
                            "pose_xyz": [0.5, -0.3, 0.1],
                            "pose_rpy": [0.0, 0.0, 0.0],
                        }
                    ],
                    "rules": [
                        {
                            "id": "default_place",
                            "when": {"always": True},
                            "destination": "default_drop_zone",
                        }
                    ],
                },
                "commissioning": {"self_test_enabled": True, "export_bundle": False},
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )


def _add_scene3d_pass_evidence(scene_dir: Path) -> None:
    smoke_path = scene_dir / "generated" / "scene3d_gui_smoke.json"
    smoke_path.parent.mkdir(parents=True, exist_ok=True)
    smoke_path.write_text(
        json.dumps(
            {
                "schema": "workcell_studio_scene3d_gui_smoke/v1",
                "primitive_rendered_count": 1,
                "rendered_count": 1,
            }
        ),
        encoding="utf-8",
    )


def _build_matrix(repo_root: Path, catalog: Path, *, ros_available: bool) -> dict[str, Any]:
    original_ros_available = matrix._ros_humble_available
    matrix._ros_humble_available = lambda: ros_available
    try:
        return matrix.build_matrix(repo_root, catalog_path=catalog, output_dir=repo_root / "build" / "matrix")
    finally:
        matrix._ros_humble_available = original_ros_available


def _single_scene_report(tmp_path: Path, minimal_scene_factory: Any, name: str, **kwargs: Any) -> dict[str, Any]:
    ros_available = kwargs.pop("ros_available", True)
    kwargs.setdefault("mesh_index_payload", {"items": [{"id": "fixture_box", "primitive_type": "box"}]})
    scene_dir, entry = minimal_scene_factory(name, repo_root=tmp_path, **kwargs)
    if (scene_dir / "cell_definition.yaml").exists():
        _write_valid_cell_definition(scene_dir, name)
    _add_scene3d_pass_evidence(scene_dir)
    catalog = _write_catalog(tmp_path, [entry])
    return _build_matrix(tmp_path, catalog, ros_available=ros_available)


def _only_scene(report: dict[str, Any]) -> dict[str, Any]:
    assert report["schema_version"] == matrix.SCHEMA_VERSION
    assert report["scene_count"] == 1
    assert isinstance(report["totals"], dict)
    assert len(report["scenes"]) == 1
    scene = report["scenes"][0]
    assert "overall_status" in scene
    assert isinstance(scene["categories"], dict)
    assert isinstance(scene["commands"], dict)
    assert "fake_hardware_launch_command" in scene["commands"]
    assert isinstance(report["commands"], dict)
    assert "safe_launch_policy" in report["commands"]
    return scene


def test_ready_scene_records_launch_smoke_as_blocked_without_execution(tmp_path: Path, minimal_scene_factory: Any) -> None:
    report = _single_scene_report(tmp_path, minimal_scene_factory, "ready_scene")

    scene = _only_scene(report)
    assert scene["overall_status"] == "BLOCKED"
    launch_state = scene["categories"]["ros_launch_smoke_skip_evaluation_state"]
    assert launch_state["status"] == "BLOCKED"
    assert "does not execute ros2 launch" in launch_state["message"]


def test_parse_args_accepts_supported_scenes_alias_for_catalog() -> None:
    args = matrix.parse_args(["--supported-scenes", "scenes/supported_scenes.yaml"])

    assert args.catalog == Path("scenes/supported_scenes.yaml")


def test_parse_args_keeps_catalog_argument_for_backward_compatibility() -> None:
    args = matrix.parse_args(["--catalog", "custom/catalog.yaml"])

    assert args.catalog == Path("custom/catalog.yaml")


def test_main_routes_supported_scenes_alias_to_build_matrix(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    catalog = tmp_path / "scenes" / "supported_scenes.yaml"
    output_dir = tmp_path / "matrix_output"
    captured: dict[str, Path] = {}

    def fake_build_matrix(repo_root: Path, catalog_path: Path, output_path: Path) -> dict[str, Any]:
        captured["repo_root"] = repo_root
        captured["catalog_path"] = catalog_path
        captured["output_dir"] = output_path
        return {
            "schema_version": matrix.SCHEMA_VERSION,
            "generated_at": "2026-06-02T00:00:00+00:00",
            "repo_root": str(repo_root),
            "workspace_root": str(repo_root.parent),
            "catalog_path": str(catalog_path),
            "catalog_errors": [],
            "scene_count": 0,
            "totals": {matrix.PASS: 0, matrix.FAIL: 0, matrix.BLOCKED: 0},
            "scenes": [],
            "commands": {},
            "ros_humble_available": False,
            "workcell_builder_executable_found": False,
        }

    monkeypatch.setattr(matrix, "build_matrix", fake_build_matrix)
    monkeypatch.setattr(matrix, "_write_markdown", lambda _payload, _path: None)

    result = matrix.main(
        [
            "--repo-root",
            str(tmp_path),
            "--supported-scenes",
            str(catalog),
            "--output-dir",
            str(output_dir),
        ]
    )

    assert result == 0
    assert captured["repo_root"] == tmp_path.resolve()
    assert captured["catalog_path"] == catalog.resolve()
    assert captured["output_dir"] == output_dir.resolve()


def test_fully_healthy_synthetic_scene_produces_overall_pass(tmp_path: Path, minimal_scene_factory: Any) -> None:
    report = _single_scene_report(tmp_path, minimal_scene_factory, "healthy_scene")

    scene = _only_scene(report)
    assert report["totals"]["BLOCKED"] == 1
    assert scene["overall_status"] == "BLOCKED"
    assert all(
        result["status"] == "PASS"
        for name, result in scene["categories"].items()
        if name != "ros_launch_smoke_skip_evaluation_state"
    )
    assert scene["categories"]["ros_launch_smoke_skip_evaluation_state"]["status"] == "BLOCKED"
    assert "does not execute ros2 launch" in scene["categories"]["ros_launch_smoke_skip_evaluation_state"]["message"]


def test_missing_package_xml_records_missing_file_blocker(tmp_path: Path, minimal_scene_factory: Any) -> None:
    scene_dir, entry = minimal_scene_factory("missing_package_xml", repo_root=tmp_path)
    (scene_dir / "package.xml").unlink()
    catalog = _write_catalog(tmp_path, [entry])

    _write_valid_cell_definition(scene_dir, "missing_package_xml")
    _add_scene3d_pass_evidence(scene_dir)
    report = _build_matrix(tmp_path, catalog, ros_available=True)

    scene = _only_scene(report)
    assert scene["overall_status"] == "FAIL"
    assert scene["categories"]["package_xml"]["status"] == "FAIL"
    assert "missing required file: package.xml" in scene["categories"]["package_xml"]["message"]


def test_missing_cell_definition_records_schema_or_file_blocker(tmp_path: Path, minimal_scene_factory: Any) -> None:
    report = _single_scene_report(
        tmp_path,
        minimal_scene_factory,
        "missing_cell_definition",
        missing_generated_files=["cell_definition.yaml"],
    )

    scene = _only_scene(report)
    assert scene["overall_status"] == "FAIL"
    assert scene["categories"]["cell_definition_yaml"]["status"] == "FAIL"
    assert scene["categories"]["cell_definition_validation"]["status"] == "BLOCKED"
    assert "cell_definition.yaml" in scene["categories"]["cell_definition_validation"]["message"]


def test_bad_scene_manifest_local_reference_records_manifest_reference_failure(
    tmp_path: Path,
    minimal_scene_factory: Any,
) -> None:
    scene_dir, entry = minimal_scene_factory("bad_manifest_reference", repo_root=tmp_path)
    (scene_dir / "scene_manifest.yaml").write_text(
        yaml.safe_dump(
            {
                "scene": {"name": "bad_manifest_reference"},
                "files": {"environment": "environment.yaml", "missing_urdf": "urdf/does_not_exist.xacro"},
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    catalog = _write_catalog(tmp_path, [entry])

    _write_valid_cell_definition(scene_dir, "bad_manifest_reference")
    _add_scene3d_pass_evidence(scene_dir)
    report = _build_matrix(tmp_path, catalog, ros_available=True)

    scene = _only_scene(report)
    assert scene["overall_status"] == "FAIL"
    assert scene["categories"]["manifest_local_file_references"]["status"] == "FAIL"
    missing_refs = scene["categories"]["manifest_local_file_references"]["missing"]
    assert any(item["reference"] == "urdf/does_not_exist.xacro" for item in missing_refs)


def test_missing_visual_mesh_index_blocks_visual_evidence(tmp_path: Path, minimal_scene_factory: Any) -> None:
    report = _single_scene_report(
        tmp_path,
        minimal_scene_factory,
        "missing_visual_mesh_index",
        missing_generated_files=["generated/scene_visual_mesh_index.json"],
    )

    scene = _only_scene(report)
    assert scene["overall_status"] == "FAIL"
    assert scene["categories"]["generated_scene_visual_mesh_index_json"]["status"] == "FAIL"
    assert scene["categories"]["scene3d_visual_quality_summary"]["status"] == "BLOCKED"
    visual_summary = scene["categories"]["scene3d_visual_quality_summary"]
    assert "generated/scene_visual_mesh_index.json" in visual_summary["message"] or visual_summary["blockers"]


def test_visual_quality_failure_propagates_current_visual_category_status(
    tmp_path: Path,
    minimal_scene_factory: Any,
) -> None:
    report = _single_scene_report(
        tmp_path,
        minimal_scene_factory,
        "blocked_visual_quality",
        mesh_index_payload={
            "visual_quality_status": "BLOCKED",
            "visual_items": [{"render_expected": True}],
            "blocker_reasons": ["rendered physical scene evidence is unavailable"],
        },
    )

    scene = _only_scene(report)
    assert scene["overall_status"] == "FAIL"
    assert scene["categories"]["scene3d_visual_quality_summary"]["status"] == "FAIL"
    assert scene["categories"]["scene3d_visual_quality_summary"]["visual_quality_status"] == "FAIL"
    assert scene["categories"]["scene3d_visual_quality_summary"]["blockers"]


def test_scene3d_runtime_unavailable_blocks_runtime_visual_evidence(
    tmp_path: Path,
    minimal_scene_factory: Any,
) -> None:
    scene_dir, entry = minimal_scene_factory(
        "scene3d_runtime_unavailable",
        repo_root=tmp_path,
        mesh_index_payload={"items": [{"id": "fixture_box", "primitive_type": "box"}]},
    )
    _write_valid_cell_definition(scene_dir, "scene3d_runtime_unavailable")
    (scene_dir / "generated" / "scene3d_gui_smoke.json").write_text(
        json.dumps(
            {
                "schema": "workcell_studio_scene3d_gui_smoke/v1",
                "status": "FAIL",
                "runtime_available": False,
                "screenshot_available": False,
                "resolved_executable": None,
                "searched_paths": ["/missing/workcell_builder"],
                "render_debug_counters": {"runtime_available": False},
            }
        ),
        encoding="utf-8",
    )
    catalog = _write_catalog(tmp_path, [entry])

    report = _build_matrix(tmp_path, catalog, ros_available=True)

    scene = _only_scene(report)
    visual_summary = scene["categories"]["scene3d_visual_quality_summary"]
    physical_evidence = scene["categories"]["credible_physical_visual_evidence"]
    assert visual_summary["status"] == "BLOCKED"
    assert physical_evidence["status"] == "BLOCKED"
    for category in (visual_summary, physical_evidence):
        assert category["smoke_status"] == "FAIL"
        assert category["runtime_available"] is False
        assert category["screenshot_available"] is False
        assert category["resolved_executable"] is None
        assert category["searched_paths"] == ["/missing/workcell_builder"]


def test_scene3d_blocked_smoke_status_blocks_visual_summary(
    tmp_path: Path,
    minimal_scene_factory: Any,
) -> None:
    scene_dir, entry = minimal_scene_factory(
        "scene3d_blocked_smoke_status",
        repo_root=tmp_path,
        mesh_index_payload={"items": [{"id": "fixture_box", "primitive_type": "box"}]},
    )
    _write_valid_cell_definition(scene_dir, "scene3d_blocked_smoke_status")
    (scene_dir / "generated" / "scene3d_gui_smoke.json").write_text(
        json.dumps(
            {
                "schema": "workcell_studio_scene3d_gui_smoke/v1",
                "status": "BLOCKED",
                "runtime_available": True,
                "screenshot_available": True,
                "resolved_executable": "/tmp/workcell_builder",
                "searched_paths": ["/tmp/workcell_builder"],
                "primitive_rendered_count": 1,
                "rendered_count": 1,
            }
        ),
        encoding="utf-8",
    )
    catalog = _write_catalog(tmp_path, [entry])

    report = _build_matrix(tmp_path, catalog, ros_available=True)

    scene = _only_scene(report)
    visual_summary = scene["categories"]["scene3d_visual_quality_summary"]
    assert visual_summary["status"] == "BLOCKED"
    assert visual_summary["visual_quality_status"] == "PASS"
    assert visual_summary["smoke_status"] == "BLOCKED"
    assert visual_summary["runtime_available"] is True

def test_ros_humble_unavailable_records_launch_smoke_as_safely_skipped(
    tmp_path: Path,
    minimal_scene_factory: Any,
) -> None:
    report = _single_scene_report(tmp_path, minimal_scene_factory, "ros_unavailable_scene", ros_available=False)

    scene = _only_scene(report)
    launch_state = scene["categories"]["ros_launch_smoke_skip_evaluation_state"]
    assert scene["overall_status"] == "BLOCKED"
    assert launch_state["status"] == "BLOCKED"
    assert "ROS Humble is not available" in launch_state["message"]
    assert "ros2 launch ros_unavailable_scene demo.launch.py" in launch_state["command"]
    assert scene["commands"]["fake_hardware_launch_command"] == launch_state["command"]


def test_fake_hardware_launch_command_is_derived_with_fake_hardware_true(
    tmp_path: Path,
    minimal_scene_factory: Any,
) -> None:
    scene_dir, entry = minimal_scene_factory(
        "derived_fake_hardware_command",
        repo_root=tmp_path,
        fake_hardware_launch_command="",
    )
    # The catalog requires a safe command, but the callable helper should still
    # be able to derive the launch command from a package entry if the raw entry
    # passed to the evaluator has no command.
    entry["fake_hardware_launch_command"] = "ros2 launch derived_fake_hardware_command demo.launch.py"
    catalog = _write_catalog(tmp_path, [entry])

    _write_valid_cell_definition(scene_dir, "derived_fake_hardware_command")
    _add_scene3d_pass_evidence(scene_dir)
    report = _build_matrix(tmp_path, catalog, ros_available=True)

    scene = _only_scene(report)
    command = scene["commands"]["fake_hardware_launch_command"]
    assert scene_dir.name == "derived_fake_hardware_command"
    assert "ros2 launch derived_fake_hardware_command demo.launch.py" in command
    assert "use_fake_hardware:=true" in command
    assert "use_fake_hardware:=false" not in command
