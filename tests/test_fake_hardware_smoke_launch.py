from __future__ import annotations
import json, subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

def _run(*args:str):
    return subprocess.run(["python3", *args], cwd=REPO_ROOT, capture_output=True, text=True, check=False)

def _session(tmp_path: Path, cmd: str) -> Path:
    p = tmp_path / "session.json"
    p.write_text(json.dumps({"schema":"rviz_moveit_plan_preview_session/v1","source":{"scene_package":"scenes/ur5_2f_test"},"session":{"launch_allowed":False,"generated_commands_only":True},"rviz_moveit":{"suggested_launch":{"command":cmd}},"safety":{"motion_started":False,"moveit_service_called":False,"ros_launch_started":False,"runtime_io_applied":False,"fake_hardware_required":True}}, indent=2), encoding="utf-8")
    return p

def test_dry_run_smoke_check(tmp_path):
    s=_session(tmp_path,"ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=true")
    out=tmp_path/"smoke"
    r=_run("scripts/run_fake_hardware_smoke_launch.py","--session",str(s),"--output-dir",str(out),"--dry-run","--json")
    assert r.returncode==0
    rep=json.loads((out/"fake_hardware_smoke_launch_report.json").read_text())
    assert rep["run"]["actually_launched"] is False
    assert rep["run"]["dry_run_only"] is True

def test_refuse_unsafe_command(tmp_path):
    s=_session(tmp_path,"ros2 launch x demo.launch.py use_fake_hardware:=false")
    out=tmp_path/"smoke"
    r=_run("scripts/run_fake_hardware_smoke_launch.py","--session",str(s),"--output-dir",str(out),"--execute","--json")
    assert r.returncode!=0

def test_mocked_execute_success(tmp_path):
    s=_session(tmp_path,"python3 -c \"print('rviz robot_state_publisher movegroup')\" use_fake_hardware:=true")
    out=tmp_path/"smoke"
    r=_run("scripts/run_fake_hardware_smoke_launch.py","--session",str(s),"--output-dir",str(out),"--execute","--timeout-s","2","--json")
    rep=json.loads((out/"fake_hardware_smoke_launch_report.json").read_text())
    assert rep["checks"]["launch_started"] is True

def test_validator(tmp_path):
    s=_session(tmp_path,"python3 -c \"print('ok')\"")
    out=tmp_path/"smoke"
    _run("scripts/run_fake_hardware_smoke_launch.py","--session",str(s),"--output-dir",str(out),"--dry-run","--json")
    r=_run("scripts/validate_fake_hardware_smoke_launch_report.py",str(out/"fake_hardware_smoke_launch_report.json"),"--json")
    assert r.returncode==0

def test_workcell_studio_commands(tmp_path):
    s=_session(tmp_path,"python3 -c \"print('ok')\" use_fake_hardware:=true")
    out=tmp_path/"smoke"
    r1=_run("scripts/workcell_studio.py","smoke-launch-preview","--session",str(s),"--output-dir",str(out),"--dry-run","--json")
    assert r1.returncode==0
    r2=_run("scripts/workcell_studio.py","validate-smoke-launch-report","--report",str(out/"fake_hardware_smoke_launch_report.json"),"--json")
    assert r2.returncode==0
