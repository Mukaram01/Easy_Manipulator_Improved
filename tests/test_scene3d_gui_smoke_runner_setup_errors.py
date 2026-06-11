import json
import os
import subprocess
from pathlib import Path


SCRIPT = "scripts/run_workcell_builder_scene3d_gui_smoke.py"


def _repo() -> Path:
    return Path(__file__).resolve().parents[1]


def _run(cmd: list[str], *, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env.pop("WORKCELL_BUILDER_EXECUTABLE", None)
    env["PATH"] = "/usr/bin:/bin"
    return subprocess.run(cmd, capture_output=True, text=True, cwd=cwd, env=env, check=False)


def test_gui_smoke_missing_explicit_workspace_writes_truthful_fail_json(tmp_path):
    repo = _repo()
    missing_ws = tmp_path / "missing_ws"
    out = tmp_path / "smoke.json"
    shot = tmp_path / "smoke.png"
    cmd = [
        "python3",
        str(repo / SCRIPT),
        "--repo-root",
        str(repo),
        "--workspace-root",
        str(missing_ws),
        "--scene",
        "ur5_2f_test",
        "--output",
        str(out),
        "--screenshot",
        str(shot),
    ]
    proc = _run(cmd)
    assert proc.returncode != 0
    assert out.exists()
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["schema"] == "workcell_studio_scene3d_gui_smoke/v1"
    assert payload["status"] == "BLOCKED"
    assert payload["workspace_root"] == str(missing_ws.resolve())
    assert "explicit_workspace_root_does_not_exist" in payload["warnings"]
    assert any(str(missing_ws.resolve() / "install" / "workcell_builder") in p for p in payload["searched_paths"])
    assert all(str(repo / "install" / "workcell_builder") not in p for p in payload["searched_paths"])
    assert payload.get("screenshot_available") is False
    assert "ros_humble_available" in payload
    assert "ros_distro" in payload
    assert payload.get("ros_humble_setup_path") == "/opt/ros/humble/setup.bash"


def test_gui_smoke_unresolved_workspace_records_truthful_workspace_and_search_path_evidence(tmp_path):
    repo = _repo()
    out = tmp_path / "smoke.json"
    shot = tmp_path / "smoke.png"
    cmd = [
        "python3",
        str(repo / SCRIPT),
        "--repo-root",
        str(repo),
        "--scene",
        "ur5_2f_test",
        "--output",
        str(out),
        "--screenshot",
        str(shot),
    ]
    proc = _run(cmd, cwd=tmp_path)
    assert proc.returncode != 0
    assert out.exists()
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["status"] == "BLOCKED"
    assert payload["repo_root"] == str(repo)
    assert payload["workspace_root"] is None
    assert payload["explicit_executable"] is None
    assert payload["blockers"] == ["workcell_builder_executable_missing"]
    assert "workspace_root_inference_failed_path_only_executable_search" in payload["warnings"]
    searched_paths = payload["searched_paths"]
    assert searched_paths
    assert all(Path(p).is_absolute() for p in searched_paths)
    assert all(str(repo / "install" / "workcell_builder") not in p for p in searched_paths)
    assert any("/home/user/workcell_ws/install/workcell_builder" in p for p in searched_paths)


def test_gui_smoke_invalid_explicit_executable_writes_blocked_json(tmp_path):
    repo = _repo()
    out = tmp_path / "smoke.json"
    shot = tmp_path / "smoke.png"
    bad_exe = tmp_path / "not_executable_workcell_builder"
    bad_exe.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    bad_exe.chmod(0o644)

    cmd = [
        "python3",
        str(repo / SCRIPT),
        "--repo-root",
        str(repo),
        "--workspace-root",
        str(repo),
        "--executable",
        str(bad_exe),
        "--scene",
        "ur5_2f_test",
        "--output",
        str(out),
        "--screenshot",
        str(shot),
    ]
    proc = _run(cmd, cwd=tmp_path)
    assert proc.returncode != 0
    assert out.exists()
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["status"] == "BLOCKED"
    assert payload["explicit_executable"] == str(bad_exe)
    assert payload["searched_paths"] == [str(bad_exe.resolve())]
    assert payload["blockers"] == ["explicit_workcell_builder_executable_missing_or_not_executable"]
    assert "guidance" in payload
