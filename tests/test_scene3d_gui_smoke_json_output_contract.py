import json, subprocess
from pathlib import Path


def test_wrapper_writes_fail_json_with_child_diagnostics(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / "smoke.json"
    shot = tmp_path / "shot.png"
    cmd = [
        "python3", str(repo / "scripts/run_workcell_builder_scene3d_gui_smoke.py"),
        "--repo-root", str(repo), "--workspace-root", str(tmp_path), "--scene", "ur5_2f_test",
        "--output", str(out), "--screenshot", str(shot), "--executable", "/bin/false", "--timeout-sec", "2",
    ]
    rc = subprocess.run(cmd, text=True, capture_output=True)
    assert rc.returncode != 0
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["status"] == "FAIL"
    assert "--scene3d-smoke" in payload["child_command"]
    assert "--smoke-output" in payload["child_command"]
    assert "--smoke-screenshot" in payload["child_command"]
    assert "--exit-after-smoke" in payload["child_command"]
    assert "child_returncode" in payload and "stdout_tail" in payload and "stderr_tail" in payload
    assert "blockers" in payload and "app_smoke_json_missing" in payload["blockers"]
