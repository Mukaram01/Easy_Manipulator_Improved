from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import yaml

import scripts.run_scene3d_visual_quality_screenshots as runner

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "run_scene3d_visual_quality_screenshots.py"


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _write_scene(repo: Path, name: str, *, failure_reason: str | None = None) -> Path:
    scene = repo / "scenes" / name
    for rel in ("package.xml", "scene_manifest.yaml", "cell_definition.yaml", "launch/demo.launch.py"):
        path = scene / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("<package/>\n" if rel == "package.xml" else "# fixture\n", encoding="utf-8")
    visual_items = [
        {"id": "mesh", "geometry": {"mesh": {"filename": "package://demo/meshes/tool.stl"}}, "resolved": True},
        {"id": "primitive", "geometry": {"box": {"size": [1, 1, 1]}}, "resolved": True},
    ]
    if failure_reason:
        visual_items.append({"id": "missing_mesh", "mesh_path": "missing.stl", "resolved": False, "reason_code": failure_reason})
    _write_json(scene / "generated" / "scene_visual_mesh_index.json", {"safe_for_preview": True, "visual_items": visual_items})
    return scene


def _write_fake_executable(path: Path) -> None:
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import json, pathlib, sys\n"
        "args=sys.argv[1:]\n"
        "out=pathlib.Path(args[args.index('--smoke-output')+1])\n"
        "png=pathlib.Path(args[args.index('--smoke-screenshot')+1]) if '--smoke-screenshot' in args else None\n"
        "scene=args[args.index('--scene')+1] if '--scene' in args "
        "else pathlib.Path(args[args.index('--scene-path')+1]).name\n"
        "payload={'schema':'workcell_studio_scene3d_gui_smoke/v1','status':'PASS','scene':scene,'counters':{"
        "'rendered_count':2,'mesh_source_count':1,'mesh_rendered_count':1,'urdf_primitive_source_count':1,"
        "'urdf_primitive_rendered_count':1,'primitive_fallback_count':1,'placeholder_count':0,"
        "'missing_geometry_count':0,'wireframe_fallback_count':0,'visual_quality_status':'PASS'}}\n"
        "out.parent.mkdir(parents=True, exist_ok=True); out.write_text(json.dumps(payload))\n"
        "png.parent.mkdir(parents=True, exist_ok=True); png.write_bytes(b'png') if png else None\n"
        "sys.exit(0)\n",
        encoding="utf-8",
    )
    path.chmod(path.stat().st_mode | 0o111)


def _write_failing_fake_executable(path: Path) -> None:
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import json, pathlib, sys\n"
        "args=sys.argv[1:]\n"
        "out=pathlib.Path(args[args.index('--smoke-output')+1])\n"
        "scene=args[args.index('--scene')+1] if '--scene' in args "
        "else pathlib.Path(args[args.index('--scene-path')+1]).name\n"
        "payload={'schema':'workcell_studio_scene3d_gui_smoke/v1','status':'PASS','scene':scene,'counters':{"
        "'rendered_count':4,'mesh_source_count':1,'mesh_rendered_count':0,'urdf_primitive_source_count':1,"
        "'urdf_primitive_rendered_count':0,'primitive_rendered_count':0,'primitive_fallback_count':0,'placeholder_count':0,"
        "'missing_geometry_count':2,'wireframe_fallback_count':0,'overlay_helper_count':4,'overlay_count':4,"
        "'label_count':1,'warning_anchor_count':1,'fov_helper_count':1,'reach_helper_count':1,'safety_zone_count':1,"
        "'physical_fit_bounds_count':1,'helper_overlay_fit_bounds_count':4,'visual_quality_status':'FAIL'}}\n"
        "out.parent.mkdir(parents=True, exist_ok=True); out.write_text(json.dumps(payload))\n"
        "sys.exit(0)\n",
        encoding="utf-8",
    )
    path.chmod(path.stat().st_mode | 0o111)


def test_collect_targets_from_explicit_catalog_and_synthetic(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    _write_scene(repo, "explicit_scene")
    _write_scene(repo, "catalog_scene")
    fixture = _write_scene(repo, "synthetic_visual_quality_fixture")
    catalog = repo / "scenes" / "supported_scenes.yaml"
    catalog.write_text(
        yaml.safe_dump(
            {
                "schema_version": "workcell_studio_supported_scenes/v1",
                "scenes": [
                    {"scene_name": "catalog_scene", "package_name": "catalog_scene", "scene_path": "scenes/catalog_scene", "support_level": "supported", "status": "supported"},
                    {"scene_name": "ignored_scene", "support_level": "ignored", "status": "ignored"},
                ],
            }
        ),
        encoding="utf-8",
    )

    targets = runner.collect_targets(repo, ["explicit_scene"], catalog, fixture)

    assert [(t["target_kind"], t["scene_name"]) for t in targets] == [
        ("explicit_scene", "explicit_scene"),
        ("supported_scene_catalog", "catalog_scene"),
        ("synthetic_fixture", "synthetic_visual_quality_fixture"),
    ]


def test_visual_quality_screenshot_runner_captures_required_summaries(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    os.symlink(ROOT / "scripts", repo / "scripts", target_is_directory=True)
    (repo / "workcell_builder").mkdir()
    _write_scene(repo, "demo_scene")
    fake_exe = tmp_path / "workcell_builder"
    _write_fake_executable(fake_exe)
    out = tmp_path / "out"

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--repo-root",
            str(repo),
            "--workspace-root",
            str(repo),
            "--executable",
            str(fake_exe),
            "--scene",
            "demo_scene",
            "--output-dir",
            str(out),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 0, proc.stdout + proc.stderr
    summary = json.loads((out / "scene3d_visual_quality_screenshots_summary.json").read_text(encoding="utf-8"))
    result = summary["results"][0]
    assert result["smoke_json"].endswith("scene3d_gui_smoke_demo_scene.json")
    assert result["screenshot_path"].endswith("scene3d_gui_smoke_demo_scene.png")
    assert result["visual_quality_counter_summary"]["mesh_rendered_count"] == 1
    assert result["primitive_render_summary"]["primitive_rendered_count"] == 1
    assert result["mesh_failure_summary_by_reason_code"]["by_reason_code"] == {}
    assert Path(result["smoke_json"]).exists()
    assert Path(result["screenshot_path"]).exists()


def test_visual_quality_screenshot_runner_reports_blockers_in_json_and_markdown_for_synthetic_failures(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    os.symlink(ROOT / "scripts", repo / "scripts", target_is_directory=True)
    (repo / "workcell_builder").mkdir()
    _write_scene(repo, "synthetic_failing_a")
    _write_scene(repo, "synthetic_failing_b")
    fake_exe = tmp_path / "failing_workcell_builder"
    _write_failing_fake_executable(fake_exe)
    out = tmp_path / "out"

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--repo-root",
            str(repo),
            "--workspace-root",
            str(repo),
            "--executable",
            str(fake_exe),
            "--scene",
            "synthetic_failing_a",
            "--scene",
            "synthetic_failing_b",
            "--output-dir",
            str(out),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 1, proc.stdout + proc.stderr
    summary = json.loads((out / "scene3d_visual_quality_screenshots_summary.json").read_text(encoding="utf-8"))
    assert summary["missing_screenshot"] == 2
    assert summary["smoke_json_missing"] == 0
    assert summary["smoke_json_unreadable"] == 0
    assert summary["blocker_reason_summary"]["missing_screenshot"] == 2
    assert summary["blocker_reason_summary"]["mesh_source_not_rendered"] == 2
    assert summary["blocker_reason_summary"]["urdf_primitive_source_not_rendered"] == 2
    assert summary["blocker_reason_summary"]["placeholder_missing_geometry_dominates"] == 2
    assert summary["blocker_reason_summary"]["overlay_helper_dominates"] == 2
    assert summary["blocker_reason_summary"]["no_physical_scene_items_rendered"] == 2
    for result in summary["results"]:
        assert result["status"] == "BLOCKED"
        assert result["blocker_categories"]["missing_screenshot"] is True
        assert "missing_screenshot" in result["blocker_reasons"]
        assert "mesh_source_not_rendered" in result["blocker_reasons"]
        assert "urdf_primitive_source_not_rendered" in result["blocker_reasons"]
        assert "overlay_helper_dominates" in result["blocker_reasons"]
        assert "no_physical_scene_items_rendered" in result["blocker_reasons"]
        assert result["visual_quality_counter_summary"]["overlay_count"] == 4
        assert result["visual_quality_counter_summary"]["label_count"] == 1
        assert result["visual_quality_counter_summary"]["helper_overlay_fit_bounds_count"] == 4
        assert "capture or attach the Scene3D smoke screenshot" in result["blocker_text"]
        assert "overlay helpers cannot prove visual quality" in result["blocker_text"]

    markdown = (out / "scene3d_visual_quality_screenshots_summary.md").read_text(encoding="utf-8")
    assert "| Scene | Status | Smoke JSON | Screenshot | Blockers | Mesh failure reasons |" in markdown
    assert "- missing_screenshot: 2" in markdown
    assert "- mesh_source_not_rendered: 2" in markdown
    assert "synthetic_failing_a" in markdown
    assert "synthetic_failing_b" in markdown
    assert "capture or attach the Scene3D smoke screenshot" in markdown
    assert "overlay helpers cannot prove visual quality" in markdown
    assert "render at least one physical mesh, primitive, or fallback scene item" in markdown


def _write_fake_executable_for_scene_counters(path: Path, counters_by_scene: dict[str, dict]) -> None:
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import json, pathlib, sys\n"
        f"counters_by_scene={counters_by_scene!r}\n"
        "args=sys.argv[1:]\n"
        "out=pathlib.Path(args[args.index('--smoke-output')+1])\n"
        "png=pathlib.Path(args[args.index('--smoke-screenshot')+1]) if '--smoke-screenshot' in args else None\n"
        "scene=args[args.index('--scene')+1] if '--scene' in args else pathlib.Path(args[args.index('--scene-path')+1]).name\n"
        "payload={'schema':'workcell_studio_scene3d_gui_smoke/v1','status':'PASS','scene':scene,'counters':counters_by_scene[scene]}\n"
        "out.parent.mkdir(parents=True, exist_ok=True); out.write_text(json.dumps(payload))\n"
        "png.parent.mkdir(parents=True, exist_ok=True); png.write_bytes(b'png') if png else None\n"
        "sys.exit(0)\n",
        encoding="utf-8",
    )
    path.chmod(path.stat().st_mode | 0o111)


def test_visual_quality_screenshot_runner_summarizes_healthy_and_blocked_visual_fixtures(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    os.symlink(ROOT / "scripts", repo / "scripts", target_is_directory=True)
    (repo / "workcell_builder").mkdir()
    healthy_mesh_scene = _write_scene(repo, "healthy_mesh_scene")
    healthy_primitive_scene = _write_scene(repo, "healthy_primitive_scene")
    missing_mesh_scene = _write_scene(repo, "missing_mesh_scene", failure_reason="mesh_missing_on_disk")
    overlay_only_scene = _write_scene(repo, "overlay_only_scene")
    raw_bounds_only_scene = _write_scene(repo, "raw_bounds_only_scene")
    _write_json(
        healthy_mesh_scene / "generated" / "scene_visual_mesh_index.json",
        {"safe_for_preview": True, "visual_items": [{"id": "mesh", "geometry": {"mesh": {"filename": "package://demo/meshes/tool.stl"}}, "resolved": True}]},
    )
    _write_json(
        healthy_primitive_scene / "generated" / "scene_visual_mesh_index.json",
        {"safe_for_preview": True, "visual_items": [{"id": "primitive", "geometry": {"box": {"size": [1, 1, 1]}}, "resolved": True}]},
    )
    _write_json(
        overlay_only_scene / "generated" / "scene_visual_mesh_index.json",
        {"safe_for_preview": True, "visual_items": [{"id": "mesh", "geometry": {"mesh": {"filename": "package://demo/meshes/tool.stl"}}, "resolved": True}]},
    )
    _write_json(
        raw_bounds_only_scene / "generated" / "scene_visual_mesh_index.json",
        {"safe_for_preview": True, "visual_items": [{"id": "mesh", "geometry": {"mesh": {"filename": "package://demo/meshes/tool.stl"}}, "resolved": True}]},
    )
    fake_exe = tmp_path / "visual_fixture_workcell_builder"
    _write_fake_executable_for_scene_counters(
        fake_exe,
        {
            "healthy_mesh_scene": {"rendered_count": 1, "mesh_rendered_count": 1, "primitive_rendered_count": 0},
            "healthy_primitive_scene": {"rendered_count": 1, "mesh_rendered_count": 0, "primitive_rendered_count": 1},
            "missing_mesh_scene": {"rendered_count": 0, "mesh_rendered_count": 0, "primitive_rendered_count": 0},
            "overlay_only_scene": {
                "rendered_count": 3,
                "mesh_rendered_count": 0,
                "primitive_rendered_count": 0,
                "overlay_helper_count": 2,
                "label_count": 1,
            },
            "raw_bounds_only_scene": {
                "rendered_count": 1,
                "mesh_rendered_count": 0,
                "primitive_rendered_count": 0,
                "raw_generated_bounds_count": 1,
            },
        },
    )
    out = tmp_path / "out"

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--repo-root",
            str(repo),
            "--workspace-root",
            str(repo),
            "--executable",
            str(fake_exe),
            "--scene",
            "healthy_mesh_scene",
            "--scene",
            "healthy_primitive_scene",
            "--scene",
            "missing_mesh_scene",
            "--scene",
            "overlay_only_scene",
            "--scene",
            "raw_bounds_only_scene",
            "--output-dir",
            str(out),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 1, proc.stdout + proc.stderr
    summary = json.loads((out / "scene3d_visual_quality_screenshots_summary.json").read_text(encoding="utf-8"))
    by_scene = {result["scene"]: result for result in summary["results"]}

    assert by_scene["healthy_mesh_scene"]["status"] == "PASS"
    assert by_scene["healthy_mesh_scene"]["visual_quality_evaluation"]["mesh_source_count"] > 0
    assert by_scene["healthy_mesh_scene"]["visual_quality_evaluation"]["mesh_rendered_count"] > 0
    assert by_scene["healthy_primitive_scene"]["status"] == "PASS"
    assert by_scene["healthy_primitive_scene"]["visual_quality_evaluation"]["primitive_source_count"] > 0
    assert by_scene["healthy_primitive_scene"]["visual_quality_evaluation"]["primitive_rendered_count"] > 0

    assert by_scene["missing_mesh_scene"]["mesh_failure_summary_by_reason_code"]["by_reason_code"] == {"mesh_missing_on_disk": 1}
    assert "mesh_missing_on_disk" in by_scene["missing_mesh_scene"]["blocker_reasons"]
    assert summary["blocker_reason_summary"]["mesh_missing_on_disk"] == 1

    assert by_scene["overlay_only_scene"]["visual_quality_evaluation"]["physical_rendered_count"] == 0
    assert "no_physical_scene_items_rendered" in by_scene["overlay_only_scene"]["blocker_reasons"]
    assert by_scene["raw_bounds_only_scene"]["visual_quality_evaluation"]["physical_rendered_count"] == 0
    assert by_scene["raw_bounds_only_scene"]["visual_quality_evaluation"]["raw_generated_bounds_count"] == 1
    assert by_scene["raw_bounds_only_scene"]["status"] == "BLOCKED"

    markdown = (out / "scene3d_visual_quality_screenshots_summary.md").read_text(encoding="utf-8")
    assert "- mesh_missing_on_disk: 1" in markdown
    assert "mesh_missing_on_disk=1" in markdown
