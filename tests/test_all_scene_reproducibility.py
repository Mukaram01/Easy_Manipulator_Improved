from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

def test_validator_emits_per_scene_report_and_contract_keys():
    repo_root = Path(__file__).resolve().parents[1]
    script = repo_root / "scripts" / "validate_all_workcell_studio_scenes.py"
    report = repo_root / "build" / "workcell_studio" / "all_scene_reproducibility_report.json"

    if report.exists():
        report.unlink()

    proc = subprocess.run([sys.executable, str(script)], cwd=repo_root, capture_output=True, text=True)

    assert report.exists(), proc.stdout + "\n" + proc.stderr
    payload = json.loads(report.read_text(encoding="utf-8"))
    scenes = payload.get("scenes")
    assert isinstance(scenes, list) and scenes

    import yaml

    catalog = yaml.safe_load((repo_root / "scenes" / "supported_scenes.yaml").read_text(encoding="utf-8"))
    expected_names = {entry["scene_name"] for entry in catalog["scenes"] if entry.get("enabled", True)}
    names = {s.get("scene_name") for s in scenes}
    assert expected_names == names
    assert payload.get("supported_scene_catalog", "").endswith("scenes/supported_scenes.yaml")

    expected_keys = {
        "scene_name",
        "status",
        "files",
        "optional_files",
        "generated_mesh_index_present",
        "mesh_index_regeneration_status",
        "mesh_index_renderable_items",
        "preview_readiness_status",
        "generated_artifacts_present",
        "fake_hardware_smoke_command_available",
        "fake_hardware_smoke_command",
        "blockers",
        "warnings",
        "warning_groups",
    }
    for scene in scenes:
        assert expected_keys.issubset(scene.keys())
        assert scene["status"] in {"PASS", "WARN", "FAIL", "SKIP"}
        assert isinstance(scene["blockers"], list)
        assert isinstance(scene["warnings"], list)

    assert "ModuleNotFoundError: No module named 'yaml'" not in (proc.stdout + proc.stderr)


def test_scene_warnings_are_grouped_and_non_ambiguous():
    repo_root = Path(__file__).resolve().parents[1]
    script = repo_root / "scripts" / "validate_all_workcell_studio_scenes.py"
    report = repo_root / "build" / "workcell_studio" / "all_scene_reproducibility_report.json"

    subprocess.run([sys.executable, str(script)], cwd=repo_root, check=True, capture_output=True, text=True)
    payload = json.loads(report.read_text(encoding="utf-8"))

    for scene in payload.get("scenes", []):
        groups = scene.get("warning_groups")
        assert isinstance(groups, dict)
        assert set(groups.keys()) == {"metadata", "preview", "generation", "launch_simulation", "runtime_smoke"}
        for key, items in groups.items():
            assert isinstance(items, list), f"{scene.get('scene_name')}:{key}"
            for msg in items:
                assert isinstance(msg, str) and msg.strip(), f"{scene.get('scene_name')}:{key}"
        for warning in scene.get("warnings", []):
            assert warning.strip()
            assert any(token in warning.lower() for token in ("missing", "issue", "degraded", "failed", "unreadable", "not in known-scenes"))


def test_moveit_launch_readiness_uses_synthetic_launch_report_statuses(tmp_path):
    from scripts.validate_all_workcell_studio_scenes import audit_scene

    def _mk_scene(name: str, report_status: str):
        scene = tmp_path / name
        (scene / "layout").mkdir(parents=True)
        (scene / "generated").mkdir(parents=True)
        (scene / "launch").mkdir(parents=True)
        (scene / "urdf").mkdir(parents=True)
        (scene / "scene_manifest.yaml").write_text("schema_version: x\n", encoding="utf-8")
        (scene / "environment.yaml").write_text("schema_version: y\nsupport_surfaces: []\ntask_zones: []\n", encoding="utf-8")
        (scene / "layout/workcell_studio_layout.yaml").write_text("schema_version: workcell_studio_layout/v1\nitems: [{id: i1, type: marker}]\n", encoding="utf-8")
        (scene / "launch/demo.launch.py").write_text("# launch\n", encoding="utf-8")
        (scene / "urdf/scene.urdf").write_text("<robot/>\n", encoding="utf-8")
        (scene / "generated/scene_visual_mesh_index.json").write_text(json.dumps({"items": [{"render_expected": True}]}), encoding="utf-8")
        (scene / "generated/fake_hardware_smoke_launch_report.json").write_text(
            json.dumps({"schema": "fake_hardware_smoke_launch_report/v1", "result": {"status": report_status, "warnings": [], "errors": []}}),
            encoding="utf-8",
        )
        return scene

    assert audit_scene(repo_root=Path(__file__).resolve().parents[1], scene_dir=_mk_scene("s1", "PASS")).readiness["moveit_launch_readiness"].status == "PASS"
    assert audit_scene(repo_root=Path(__file__).resolve().parents[1], scene_dir=_mk_scene("s2", "WARN")).readiness["moveit_launch_readiness"].status == "WARN"
    assert audit_scene(repo_root=Path(__file__).resolve().parents[1], scene_dir=_mk_scene("s3", "FAIL")).readiness["moveit_launch_readiness"].status == "FAIL"
