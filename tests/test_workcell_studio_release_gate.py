from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import scripts.run_workcell_studio_release_gate as gate


def test_static_profile_writes_json_markdown_and_reports_required_checks(tmp_path: Path, monkeypatch):
    repo = tmp_path
    (repo / "scripts").mkdir()
    (repo / "scenes" / "demo" / "launch").mkdir(parents=True)
    (repo / "scenes" / "demo" / "launch" / "demo.launch.py").write_text("use_fake_hardware default_value='true'\n", encoding="utf-8")
    monkeypatch.setattr(gate, "_git_sha", lambda _repo: "abc123")
    monkeypatch.setattr(gate, "_run", lambda cmd, cwd, timeout_sec: subprocess.CompletedProcess(cmd, 0, "ok", ""))
    class Args:
        profile = "static"; repo_root = repo; workspace_root = repo; output_dir = tmp_path / "out"; json_output = tmp_path / "gate.json"; markdown_output = tmp_path / "gate.md"; timeout_sec = 5; scene = "demo"; dry_run_runtime = False
    report = gate.run_gate(Args)
    assert report["overall_status"] == gate.PASS
    assert report["environment"]["commit_sha"] == "abc123"
    statuses = {row["id"]: row["status"] for row in report["checks"]}
    assert statuses["supported_scene_reproducibility"] == gate.PASS
    assert statuses["safety_defaults"] == gate.PASS
    assert statuses["workcell_builder_build"] == gate.NA
    gate.write_markdown(tmp_path / "gate.md", report)
    assert "Manual runtime checklist" in (tmp_path / "gate.md").read_text(encoding="utf-8")


def test_runtime_missing_ros_is_blocked_not_pass(tmp_path: Path, monkeypatch):
    (tmp_path / "scenes").mkdir()
    monkeypatch.setattr(gate.shutil, "which", lambda name: None)
    monkeypatch.delenv("ROS_DISTRO", raising=False)
    monkeypatch.setattr(gate, "_run", lambda cmd, cwd, timeout_sec: subprocess.CompletedProcess(cmd, 0, "ok", ""))
    class Args:
        profile = "runtime"; repo_root = tmp_path; workspace_root = tmp_path; output_dir = tmp_path / "out"; json_output = None; markdown_output = None; timeout_sec = 5; scene = "ur5_2f_test"; dry_run_runtime = False
    report = gate.run_gate(Args)
    by_id = {row["id"]: row for row in report["checks"]}
    assert by_id["workcell_builder_build"]["status"] == gate.BLOCKED
    assert "ros2 executable not found" in by_id["workcell_builder_build"]["reason"]
    assert report["overall_status"] == gate.BLOCKED


def test_safety_scan_fails_unsafe_defaults(tmp_path: Path):
    launch = tmp_path / "scenes" / "bad" / "launch" / "demo.launch.py"
    launch.parent.mkdir(parents=True)
    launch.write_text("DeclareLaunchArgument('use_fake_hardware', default_value='false')\n", encoding="utf-8")
    status, blockers = gate.safety_check(tmp_path)
    assert status == gate.FAIL
    assert blockers and "use_fake_hardware" in blockers[0]


def test_cli_returns_nonzero_for_required_blocker(tmp_path: Path):
    result = subprocess.run([sys.executable, "scripts/run_workcell_studio_release_gate.py", "--profile", "runtime", "--repo-root", str(tmp_path), "--workspace-root", str(tmp_path), "--output-dir", str(tmp_path / "out")], cwd=Path(__file__).resolve().parents[1], text=True, capture_output=True, check=False)
    assert result.returncode != 0
    payload = json.loads((tmp_path / "out" / "release_gate_runtime.json").read_text(encoding="utf-8"))
    assert any(row["status"] == gate.BLOCKED for row in payload["checks"])
