from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import scripts.run_workcell_studio_scene_readiness_gate as gate


def _audit_payload(fail_moveit: bool = False) -> dict:
    status = "FAIL" if fail_moveit else "PASS"
    return {
        "scenes": [
            {
                "scene_name": "ur5_2f_test",
                "blockers": ["missing thing"] if fail_moveit else [],
                "warnings": ["warn thing"],
                "readiness": {
                    "artifact_readiness": {"status": "PASS", "reasons": []},
                    "preview_readiness": {"status": "WARN", "reasons": ["preview warning"]},
                    "moveit_launch_readiness": {"status": status, "reasons": ["sim"]},
                    "grasp_planner_readiness": {"status": "PASS", "reasons": []},
                    "grasp_execution_readiness": {"status": "WARN", "reasons": []},
                    "real_hardware_readiness": {"status": "WARN", "reasons": []},
                },
            }
        ]
    }


def test_runner_sequence_and_forwarding(tmp_path: Path, monkeypatch):
    repo = tmp_path
    scripts_dir = repo / "scripts"
    build_dir = repo / "build" / "workcell_studio"
    scripts_dir.mkdir(parents=True)
    build_dir.mkdir(parents=True)

    (build_dir / "all_scene_reproducibility_report.json").write_text(json.dumps(_audit_payload()), encoding="utf-8")

    calls = []

    def fake_run(cmd, cwd, capture_output, text):
        calls.append(cmd)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(gate, "_run_command", lambda cmd, repo_root: fake_run(cmd, repo_root, True, True))
    monkeypatch.setattr(gate.Path, "resolve", lambda self: repo / "scripts" / "run_workcell_studio_scene_readiness_gate.py")
    monkeypatch.setattr(gate, "parse_args", lambda: SimpleNamespace(timeout_sec=45, dry_run_launches=True, launch_rviz=False, json_output=build_dir / "gate.json", markdown_output=build_dir / "gate.md", strict=False))

    rc = gate.main()
    assert rc == 0
    assert len(calls) == 2
    assert "validate_rviz_moveit_simulation_launches.py" in " ".join(calls[0])
    assert "--dry-run" in calls[0]
    assert "--headless" in calls[0]
    assert "validate_all_workcell_studio_scenes.py" in " ".join(calls[1])
    joined = " ".join(calls[1])
    assert "--simulation-launch-report" in joined
    assert "build/workcell_studio/rviz_moveit_simulation_launch_report.json" in joined

    report = json.loads((build_dir / "gate.json").read_text(encoding="utf-8"))
    assert report["schema"] == "workcell_studio_scene_readiness_gate/v1"

    md = (build_dir / "gate.md").read_text(encoding="utf-8")
    assert "| Scene | Artifact | Preview | MoveIt Launch" in md
    assert "## Blockers by Scene" in md
    assert "## Warnings by Scene" in md
    assert "This is fake-hardware simulation readiness only, not real-hardware validation." in md
    assert "use_fake_hardware:=true" in md
    assert "use_fake_hardware:=false" not in md
    assert "fake_hardware:=false" not in md


def test_strict_nonzero_on_fail(tmp_path: Path, monkeypatch):
    repo = tmp_path
    (repo / "build" / "workcell_studio").mkdir(parents=True)
    (repo / "build" / "workcell_studio" / "all_scene_reproducibility_report.json").write_text(json.dumps(_audit_payload(fail_moveit=True)), encoding="utf-8")

    monkeypatch.setattr(gate, "_run_command", lambda cmd, repo_root: SimpleNamespace(returncode=0, stdout="", stderr=""))
    monkeypatch.setattr(gate.Path, "resolve", lambda self: repo / "scripts" / "run_workcell_studio_scene_readiness_gate.py")
    monkeypatch.setattr(gate, "parse_args", lambda: SimpleNamespace(timeout_sec=45, dry_run_launches=False, launch_rviz=False, json_output=repo / "build" / "workcell_studio" / "gate.json", markdown_output=repo / "build" / "workcell_studio" / "gate.md", strict=True))

    rc = gate.main()
    assert rc == 1
