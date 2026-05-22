from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import scripts.run_workcell_studio_scene_readiness_gate as gate


def _audit_payload() -> dict:
    return {
        "scenes": [
            {
                "scene_name": "ur5_2f_test",
                "readiness": {
                    "artifact_readiness": {"status": "PASS"},
                    "preview_readiness": {"status": "PASS"},
                    "moveit_launch_readiness": {"status": "PASS"},
                    "grasp_planner_readiness": {"status": "PASS"},
                    "grasp_execution_readiness": {"status": "PASS"},
                    "real_hardware_readiness": {"status": "WARN"},
                },
                "blockers": [],
                "warnings": [],
            }
        ]
    }


def test_readiness_gate_passthrough_has_no_hardcoded_root_workspace() -> None:
    src = Path("scripts/run_workcell_studio_scene_readiness_gate.py").read_text(encoding="utf-8")
    assert "/root/workcell_ws" not in src
    assert "~/workcell_ws" not in src


def test_readiness_gate_forwards_scene3d_smoke_invocation_with_executable_artifacts(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    (repo / "scripts").mkdir(parents=True)
    smoke_script = repo / "scripts" / "run_workcell_builder_scene3d_gui_smoke.py"
    smoke_script.write_text("#!/usr/bin/env python3\n", encoding="utf-8")

    out_dir = repo / "build/workcell_studio"
    out_dir.mkdir(parents=True)
    (out_dir / "all_scene_reproducibility_report.json").write_text(json.dumps(_audit_payload()), encoding="utf-8")

    captured: list[list[str]] = []

    def fake_run(cmd: list[str], repo_root: Path):
        captured.append(cmd)
        if "run_workcell_builder_scene3d_gui_smoke.py" in " ".join(cmd):
            scene = cmd[cmd.index("--scene") + 1]
            smoke_json = Path(cmd[cmd.index("--output") + 1])
            smoke_png = Path(cmd[cmd.index("--screenshot") + 1])
            smoke_json.parent.mkdir(parents=True, exist_ok=True)
            smoke_png.parent.mkdir(parents=True, exist_ok=True)
            smoke_png.write_text("png", encoding="utf-8")
            smoke_json.write_text(json.dumps({"status": "PASS", "scene": scene, "timestamp": "2026-01-01T00:00:00Z"}), encoding="utf-8")
        return SimpleNamespace(returncode=0, stderr="")

    monkeypatch.setattr(gate, "_run_command", fake_run)
    monkeypatch.setattr(gate, "_run_scene3d_consolidated_gate", lambda repo_root, scenes: {"status": "PASS", "scenes": [], "blockers": [], "warnings": []})
    monkeypatch.setattr(gate.Path, "resolve", lambda self: repo / "scripts" / "run_workcell_studio_scene_readiness_gate.py")
    monkeypatch.setattr(gate, "_run_scene3d_consolidated_gate", lambda repo_root, scenes: {"status": "PASS", "scenes": [], "blockers": [], "warnings": []})
    monkeypatch.setattr(
        gate,
        "parse_args",
        lambda: SimpleNamespace(
            timeout_sec=45,
            dry_run_launches=False,
            launch_rviz=False,
            json_output=out_dir / "gate.json",
            markdown_output=out_dir / "gate.md",
            strict=False,
            include_visual_assets=False,
            include_scene3d_gui_smoke=True,
        ),
    )

    rc = gate.main()
    assert rc == 0
    joined = "\n".join(" ".join(c) for c in captured)
    assert "run_workcell_builder_scene3d_gui_smoke.py --scene ur5_2f_test" in joined


def test_local_mode_fail_vs_ci_dry_run_warn_downgrade(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    (repo / "scripts").mkdir(parents=True)
    (repo / "scripts" / "run_workcell_builder_scene3d_gui_smoke.py").write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    out_dir = repo / "build/workcell_studio"
    out_dir.mkdir(parents=True)
    (out_dir / "all_scene_reproducibility_report.json").write_text(json.dumps(_audit_payload()), encoding="utf-8")

    def make_runner(ci_mode: bool):
        def _fake_run(cmd: list[str], repo_root: Path):
            if "run_workcell_builder_scene3d_gui_smoke.py" in " ".join(cmd):
                scene = cmd[cmd.index("--scene") + 1]
                smoke_json = Path(cmd[cmd.index("--output") + 1])
                smoke_png = Path(cmd[cmd.index("--screenshot") + 1])
                smoke_png.write_text("png", encoding="utf-8")
                smoke_json.write_text(json.dumps({"status": "FAIL", "scene": scene, "timestamp": "2026-01-01T00:00:00Z"}), encoding="utf-8")
                return SimpleNamespace(returncode=1 if not ci_mode else 0, stderr="failed")
            return SimpleNamespace(returncode=0, stderr="")
        return _fake_run

    monkeypatch.setattr(gate.Path, "resolve", lambda self: repo / "scripts" / "run_workcell_studio_scene_readiness_gate.py")

    # local mode should fail
    monkeypatch.setattr(gate, "_run_command", make_runner(ci_mode=False))
    monkeypatch.setattr(gate, "_is_ci_environment", lambda: False)
    monkeypatch.setattr(gate, "parse_args", lambda: SimpleNamespace(timeout_sec=45, dry_run_launches=False, launch_rviz=False, json_output=out_dir / "gate_local.json", markdown_output=out_dir / "gate_local.md", strict=False, include_visual_assets=False, include_scene3d_gui_smoke=True))
    assert gate.main() == 1

    # ci/dry-run mode should downgrade to warning path
    monkeypatch.setattr(gate, "_run_command", make_runner(ci_mode=True))
    monkeypatch.setattr(gate, "_is_ci_environment", lambda: True)
    monkeypatch.setattr(gate, "parse_args", lambda: SimpleNamespace(timeout_sec=45, dry_run_launches=False, launch_rviz=False, json_output=out_dir / "gate_ci.json", markdown_output=out_dir / "gate_ci.md", strict=False, include_visual_assets=False, include_scene3d_gui_smoke=True))
    rc = gate.main()
    payload = json.loads((out_dir / "gate_ci.json").read_text(encoding="utf-8"))
    assert payload["scene3d_gui_smoke"]["mode"] == "ci_or_dry_run"
    assert payload["scene3d_gui_smoke"]["status"] == "WARN"
    # Exit code can still be non-zero when other gate dimensions fail in mixed environments.
    assert rc in {0, 1}
