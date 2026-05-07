from __future__ import annotations
import json, subprocess, sys
from pathlib import Path


def test_generate_and_validate_readiness_pack(tmp_path: Path):
    out = tmp_path / "pack"
    cmd = [sys.executable, "scripts/workcell_studio.py", "generate-readiness-pack", "--scene-package", "scenes/ur5_2f_test", "--output-dir", str(out), "--project-name", "demo", "--validate", "--smoke-dry-run", "--force", "--json"]
    run = subprocess.run(cmd, capture_output=True, text=True, check=False)
    assert run.returncode in (0,1)
    manifest = out / "readiness_pack_manifest.json"
    assert manifest.exists()
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    assert payload["schema"] == "workcell_studio_readiness_pack/v1"
    assert payload["safety"]["motion_command_sent"] is False
    vrun = subprocess.run([sys.executable, "scripts/workcell_studio.py", "validate-readiness-pack", "--manifest", str(manifest), "--json"], capture_output=True, text=True, check=False)
    assert vrun.returncode == 0


def test_unsafe_manifest_fails(tmp_path: Path):
    manifest = tmp_path / "m.json"
    manifest.write_text(json.dumps({"schema":"workcell_studio_readiness_pack/v1","safety":{"motion_command_sent":True},"results":{"final_readiness":"FAIL"},"artifacts":{}}), encoding="utf-8")
    run = subprocess.run([sys.executable, "scripts/validate_workcell_studio_readiness_pack.py", str(manifest), "--json"], capture_output=True, text=True, check=False)
    assert run.returncode != 0


def test_no_dashboard_option_and_generate_dashboard_cli(tmp_path: Path):
    out = tmp_path / 'pack2'
    run = subprocess.run([sys.executable,'scripts/workcell_studio.py','generate-readiness-pack','--scene-package','scenes/ur5_2f_test','--output-dir',str(out),'--project-name','demo','--no-dashboard','--force','--json'], capture_output=True, text=True, check=False)
    assert run.returncode in (0,1)
    assert not (out/'readiness_dashboard.html').exists()
    rerun = subprocess.run([sys.executable,'scripts/workcell_studio.py','generate-readiness-dashboard','--manifest',str(out/'readiness_pack_manifest.json'),'--output',str(out/'readiness_dashboard_regenerated.html'),'--json'], capture_output=True, text=True, check=False)
    assert rerun.returncode == 0
