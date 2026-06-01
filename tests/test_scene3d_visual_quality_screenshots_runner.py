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
        "scene=args[args.index('--scene')+1] if '--scene' in args else pathlib.Path(args[args.index('--scene-path')+1]).name\n"
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
    _write_scene(repo, "demo_scene", failure_reason="mesh_missing_on_disk")
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
    assert result["mesh_failure_summary_by_reason_code"]["by_reason_code"] == {"mesh_missing_on_disk": 1}
    assert Path(result["smoke_json"]).exists()
    assert Path(result["screenshot_path"]).exists()
