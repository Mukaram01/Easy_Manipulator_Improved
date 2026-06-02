from __future__ import annotations

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


def _single_scene_report(tmp_path: Path, minimal_scene_factory: Any, name: str, **kwargs: Any) -> dict[str, Any]:
    ros_available = kwargs.pop("ros_available", True)
    _scene_dir, entry = minimal_scene_factory(name, repo_root=tmp_path, **kwargs)
    catalog = _write_catalog(tmp_path, [entry])
    return matrix.build_readiness_matrix(tmp_path, catalog_path=catalog, ros_available=ros_available)


def _only_scene(report: dict[str, Any]) -> dict[str, Any]:
    assert report["schema_version"] == matrix.SCHEMA_VERSION
    assert report["scene_count"] == 1
    assert isinstance(report["totals"], dict)
    assert len(report["scenes"]) == 1
    scene = report["scenes"][0]
    assert "category_statuses" in scene
    assert "blocker_reasons" in scene
    assert "recommended_next_action" in scene
    assert "command_records" in scene
    assert report["command_records"] == scene["command_records"]
    return scene


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
    assert report["overall_status"] == "PASS"
    assert report["totals"]["PASS"] == 1
    assert scene["overall_status"] == "PASS"
    assert all(status == "PASS" for status in scene["category_statuses"].values())
    assert scene["blocker_reasons"] == []


def test_missing_package_xml_records_missing_file_blocker(tmp_path: Path, minimal_scene_factory: Any) -> None:
    scene_dir, entry = minimal_scene_factory("missing_package_xml", repo_root=tmp_path)
    (scene_dir / "package.xml").unlink()
    catalog = _write_catalog(tmp_path, [entry])

    report = matrix.build_readiness_matrix(tmp_path, catalog_path=catalog, ros_available=True)

    scene = _only_scene(report)
    assert scene["overall_status"] == "BLOCKED"
    assert scene["category_statuses"]["file_presence"] == "BLOCKED"
    assert "missing_file: package.xml" in scene["blocker_reasons"]


def test_missing_cell_definition_records_schema_or_file_blocker(tmp_path: Path, minimal_scene_factory: Any) -> None:
    report = _single_scene_report(
        tmp_path,
        minimal_scene_factory,
        "missing_cell_definition",
        missing_generated_files=["cell_definition.yaml"],
    )

    scene = _only_scene(report)
    assert scene["overall_status"] == "BLOCKED"
    assert scene["category_statuses"]["schema_validation"] == "BLOCKED"
    assert any("cell_definition.yaml" in reason for reason in scene["blocker_reasons"])
    assert any(
        reason.startswith("schema_validation_blocker:") or reason.startswith("missing_file:")
        for reason in scene["blocker_reasons"]
    )


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

    report = matrix.build_readiness_matrix(tmp_path, catalog_path=catalog, ros_available=True)

    scene = _only_scene(report)
    assert scene["overall_status"] == "BLOCKED"
    assert scene["category_statuses"]["manifest_references"] == "BLOCKED"
    assert any(reason.startswith("manifest_reference_failure:") for reason in scene["blocker_reasons"])
    assert any("urdf/does_not_exist.xacro" in reason for reason in scene["blocker_reasons"])


def test_missing_visual_mesh_index_blocks_visual_evidence(tmp_path: Path, minimal_scene_factory: Any) -> None:
    report = _single_scene_report(
        tmp_path,
        minimal_scene_factory,
        "missing_visual_mesh_index",
        missing_generated_files=["generated/scene_visual_mesh_index.json"],
    )

    scene = _only_scene(report)
    assert scene["overall_status"] == "BLOCKED"
    assert scene["category_statuses"]["visual_evidence"] == "BLOCKED"
    assert any("visual_evidence_blocked" in reason for reason in scene["blocker_reasons"])
    assert any("generated/scene_visual_mesh_index.json" in reason for reason in scene["blocker_reasons"])


def test_visual_quality_blocked_scene_propagates_blocked_visual_status(
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
    assert scene["overall_status"] == "BLOCKED"
    assert scene["category_statuses"]["visual_quality"] == "BLOCKED"
    assert any("visual_quality_blocked" in reason for reason in scene["blocker_reasons"])


def test_ros_humble_unavailable_records_launch_smoke_as_safely_skipped(
    tmp_path: Path,
    minimal_scene_factory: Any,
) -> None:
    report = _single_scene_report(tmp_path, minimal_scene_factory, "ros_unavailable_scene", ros_available=False)

    scene = _only_scene(report)
    launch_record = scene["command_records"][0]
    assert scene["overall_status"] == "PASS"
    assert scene["category_statuses"]["launch_smoke"] == "SKIP"
    assert launch_record["status"] == "SKIP"
    assert launch_record["executed"] is False
    assert launch_record["safely_skipped"] is True
    assert "ROS Humble unavailable" in launch_record["reason"]


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

    report = matrix.build_readiness_matrix(tmp_path, catalog_path=catalog, ros_available=True)

    scene = _only_scene(report)
    command = scene["command_records"][0]["command"]
    assert scene_dir.name == "derived_fake_hardware_command"
    assert "ros2 launch derived_fake_hardware_command demo.launch.py" in command
    assert "use_fake_hardware:=true" in command
    assert "use_fake_hardware:=false" not in command
