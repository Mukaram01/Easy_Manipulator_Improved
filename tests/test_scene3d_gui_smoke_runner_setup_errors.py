import json
import subprocess
from pathlib import Path


def test_gui_smoke_missing_workspace_writes_fail_json(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / "smoke.json"
    shot = tmp_path / "smoke.png"
    cmd = [
        "python3",
        str(repo / "scripts/run_workcell_builder_scene3d_gui_smoke.py"),
        "--repo-root",
        str(repo),
        "--workspace-root",
        str(tmp_path / "missing_ws"),
        "--scene",
        "ur5_2f_test",
        "--output",
        str(out),
        "--screenshot",
        str(shot),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    assert proc.returncode != 0
    assert out.exists()
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["schema"] == "workcell_studio_scene3d_gui_smoke/v1"
    assert payload["status"] == "FAIL"
    assert isinstance(payload.get("blockers"), list) and payload["blockers"]
    assert payload.get("repo_root")
    assert "workspace_root" in payload
    assert "executable" in payload
    assert "searched_paths" in payload
    assert payload.get("screenshot_available") is False


def test_gui_smoke_explicit_workspace_no_unboundlocalerror(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / "smoke.json"
    cmd = [
        "python3",
        str(repo / "scripts/run_workcell_builder_scene3d_gui_smoke.py"),
        "--repo-root",
        str(repo),
        "--workspace-root",
        str(repo),
        "--scene",
        "ur5_2f_test",
        "--output",
        str(out),
        "--dry-run",
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    combined = (proc.stdout or "") + (proc.stderr or "")
    assert "UnboundLocalError" not in combined
