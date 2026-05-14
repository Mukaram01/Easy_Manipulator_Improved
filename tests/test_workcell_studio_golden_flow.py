import json
import subprocess
import sys
from pathlib import Path


def test_golden_flow_runs_and_writes_reports(tmp_path: Path):
    scene = tmp_path / "ur5_robotiq_pick_place"
    cmd = [sys.executable, "scripts/run_workcell_studio_golden_flow.py", "--scene-dir", str(scene), "--json"]
    p = subprocess.run(cmd, text=True, capture_output=True, check=False)
    assert p.returncode == 0, p.stderr
    report = json.loads((scene / "golden_flow" / "workcell_studio_golden_flow_report.json").read_text(encoding="utf-8"))
    assert report["generated_files_present"]["merge"]
    assert report["generated_files_present"]["acceptance"]
    assert report["generated_files_present"]["demo"]
    assert report["generated_files_present"]["preview"]
    assert report["safety_flags"]["fake_hardware_first"] is True
