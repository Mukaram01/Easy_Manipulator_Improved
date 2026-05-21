from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "generate_rviz_moveit_plan_preview_session.py"
KNOWN_SCENES = ["ur5_2f_test", "ur5_3f_test", "suction_test"]


def _request(path: Path) -> Path:
    path.write_text(
        json.dumps(
            {
                "schema": "offline_plan_preview_request/v1",
                "request": {
                    "waypoints": [{"name": "w1"}],
                    "pick": {"source_id": "pick_zone"},
                    "place": {"target_id": "place_zone"},
                    "tool": {"grasp_strategy": "top"},
                },
            }
        ),
        encoding="utf-8",
    )
    return path


def _run(scene: str, out: Path, *extra: str) -> subprocess.CompletedProcess[str]:
    req = _request(out / "req.json")
    return subprocess.run(
        [sys.executable, str(SCRIPT), "--scene-package", str(ROOT / "scenes" / scene), "--plan-preview-request", str(req), "--output-dir", str(out), "--json", *extra],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def _session(out: Path) -> dict:
    return json.loads((out / "rviz_moveit_plan_preview_session.json").read_text(encoding="utf-8"))


def test_all_dry_run_like_generation_emits_expected_commands_for_known_scenes(tmp_path):
    for scene in KNOWN_SCENES:
        out = tmp_path / scene
        out.mkdir(parents=True)
        proc = _run(scene, out, "--allow-missing-launch")
        assert proc.returncode == 0, proc.stdout + proc.stderr
        cmd = (((_session(out).get("rviz_moveit") or {}).get("suggested_launch") or {}).get("command") or "")
        assert cmd.startswith("ros2 launch ")
        assert "demo.launch.py" in cmd


def test_use_fake_hardware_true_always_included_and_false_tokens_rejected(tmp_path):
    out = tmp_path / "ok"
    out.mkdir()
    proc = _run("ur5_2f_test", out, "--allow-missing-launch")
    assert proc.returncode == 0
    text = (out / "suggested_commands.sh").read_text(encoding="utf-8") + "\n" + json.dumps(_session(out))
    assert "use_fake_hardware:=true" in text
    assert "use_fake_hardware:=false" not in text
    assert "fake_hardware:=false" not in text


def test_json_output_schema_and_required_scene_keys(tmp_path):
    out = tmp_path / "schema"
    out.mkdir()
    proc = _run("ur5_2f_test", out, "--allow-missing-launch")
    assert proc.returncode == 0
    payload = _session(out)
    assert payload.get("schema") == "rviz_moveit_plan_preview_session/v1"
    for key in ("source", "session", "rviz_moveit", "plan_preview", "safety", "readiness", "next_manual_steps"):
        assert key in payload


def test_missing_launch_file_is_fail_without_allow_missing_launch(tmp_path):
    scene = tmp_path / "scene_no_launch"
    (scene / "config").mkdir(parents=True)
    req = _request(tmp_path / "req.json")
    proc = subprocess.run(
        [sys.executable, str(SCRIPT), "--scene-package", str(scene), "--plan-preview-request", str(req), "--output-dir", str(tmp_path / "out"), "--json"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode != 0
    summary = json.loads(proc.stdout)
    assert any("demo.launch.py not found" in b for b in summary.get("blockers", []))


def test_unsupported_metadata_classified_as_skip_in_scene_audit(tmp_path):
    from scripts.validate_all_workcell_studio_scenes import audit_scene

    scene = tmp_path / "ur5_2f_test"
    (scene / "layout").mkdir(parents=True)
    (scene / "generated").mkdir(parents=True)
    (scene / "launch").mkdir(parents=True)
    (scene / "urdf").mkdir(parents=True)
    (scene / "scene_manifest.yaml").write_text("schema_version: x\n", encoding="utf-8")
    (scene / "environment.yaml").write_text("schema_version: y\nsupport_surfaces: []\ntask_zones: []\n", encoding="utf-8")
    (scene / "layout/workcell_studio_layout.yaml").write_text("schema_version: workcell_studio_layout/v1\nitems: [{id: a, type: marker}]\n", encoding="utf-8")
    (scene / "launch/demo.launch.py").write_text("# launch\n", encoding="utf-8")
    (scene / "urdf/scene.urdf").write_text("<robot/>\n", encoding="utf-8")
    (scene / "generated/scene_visual_mesh_index.json").write_text(json.dumps({"items": [{"render_expected": True}]}), encoding="utf-8")
    (scene / "generated/fake_hardware_smoke_launch_report.json").write_text(json.dumps({"schema": "unsupported/v0", "result": {"status": "PASS"}}), encoding="utf-8")

    audit = audit_scene(ROOT, scene)
    assert audit.readiness["moveit_launch_readiness"].status == "SKIP"
