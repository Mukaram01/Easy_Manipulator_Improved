import json, shutil, subprocess, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
SCRIPT=ROOT/'scripts/workcell_studio_demo_mode.py'

def test_demo_mode_outputs(tmp_path: Path):
    src=next((ROOT/'scenes').iterdir())
    scene=tmp_path/'ur5_robotiq_2f_demo'
    shutil.copytree(src, scene)
    p=subprocess.run([sys.executable,str(SCRIPT),str(scene),'--json'],capture_output=True,text=True,check=False)
    assert p.returncode in (0,1)
    out=json.loads(p.stdout)
    demo=scene/'demo'
    assert (demo/'workcell_studio_demo_report.json').is_file()
    assert (demo/'workcell_studio_demo_dashboard.html').is_file()
    assert (demo/'workcell_studio_demo_summary.txt').is_file()
    report=json.loads((demo/'workcell_studio_demo_report.json').read_text())
    text=(demo/'workcell_studio_demo_summary.txt').read_text()
    assert 'no robot motion commanded' in text
    assert 'use_fake_hardware:=true' in text
    assert 'runtime_execution_enabled false' in text
    assert report['scene_name']
    assert 'acceptance_status' in report and 'smoke_status' in report

def test_missing_scene_blocked(tmp_path: Path):
    p=subprocess.run([sys.executable,str(SCRIPT),str(tmp_path/'missing'),'--json'],capture_output=True,text=True,check=False)
    out=json.loads(p.stdout)
    assert out['status']=='BLOCKED'
