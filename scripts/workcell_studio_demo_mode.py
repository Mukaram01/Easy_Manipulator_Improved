#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess, sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'


def _run_json(cmd:list[str], label:str)->dict[str,Any]:
    p = subprocess.run(cmd,capture_output=True,text=True,check=False)
    try:
        out = json.loads(p.stdout) if p.stdout.strip() else {}
    except Exception:
        out = {"status":"BLOCKED","blockers":[f"{label} returned non-JSON output"]}
    out["returncode"]=p.returncode
    out["stdout"]=p.stdout.strip(); out["stderr"]=p.stderr.strip()
    return out


def _write_dashboard(report:dict[str,Any], path:Path)->None:
    html=f"""<html><head><meta charset='utf-8'><title>Workcell Studio Demo Readiness</title>
<style>body{{font-family:Arial;margin:20px;background:#101820;color:#EAF0F6}}.card{{background:#172230;padding:14px;border-radius:8px;margin:10px 0}}.ok{{color:#35d07f}}.warn{{color:#f8c24e}}.bad{{color:#ff6767}} code{{background:#263548;padding:2px 4px}}</style></head><body>
<h1>Workcell Studio Demo Readiness</h1>
<div class='card'><h2>Safety Banner</h2><ul><li>No robot motion commanded</li><li>Offline/fake-hardware preview only</li><li>Runtime execution remains disabled unless explicitly enabled elsewhere</li></ul></div>
<div class='card'><h2>Scene Overview</h2><p><b>Scene:</b> {report.get('scene_name')}</p><p><b>Path:</b> {report.get('scene_path')}</p><p><b>Status:</b> {report.get('status')}</p></div>
<div class='card'><h2>Robot / Tool / Layout</h2><p>Robot/Tool: {report.get('robot_tool','unknown')}</p><p>Gripper Mount RPY: {report.get('gripper_mount_rpy')}</p><p>Layout Merge: {report.get('layout_merge_status')}</p><p>Saved Layout Timestamp: {report.get('saved_layout_timestamp')}</p><p>Merge Timestamp: {report.get('merge_timestamp')}</p><p>Layout Applied: {report.get('layout_applied')}</p><p>Stale: {report.get('layout_stale')}</p></div>
<div class='card'><h2>Acceptance</h2><p>{report.get('acceptance_status')}</p></div>
<div class='card'><h2>Smoke Check</h2><p>{report.get('smoke_status')}</p></div>
<div class='card'><h2>Preview Artifacts</h2><pre>{json.dumps(report.get('preview_paths',{}),indent=2)}</pre></div>
<div class='card'><h2>Commands</h2><p><code>{report.get('build_command')}</code></p><p><code>{report.get('fake_hardware_launch_command')}</code></p></div>
</body></html>"""
    path.write_text(html,encoding='utf-8')


def run(scene:Path)->dict[str,Any]:
    demo_dir = scene/'demo'; demo_dir.mkdir(parents=True,exist_ok=True)
    blockers=[]; warnings=[]
    if not scene.is_dir():
        blockers.append(f"Missing scene directory: {scene}")
    acceptance={"status":"BLOCKED"}
    smoke={"status":"BLOCKED"}
    if not blockers:
        if (scene/'layout'/'workcell_studio_layout.yaml').is_file():
            _run_json([sys.executable,str(SCRIPTS/'workcell_studio_layout_merge.py'),str(scene),'--json'],'layout_merge')
        acceptance=_run_json([sys.executable,str(SCRIPTS/'validate_workcell_studio_generated_scene.py'),str(scene),'--json'],'acceptance')
        smoke_json = scene/'smoke'/'offline_smoke_report.json'
        if smoke_json.is_file():
            smoke=json.loads(smoke_json.read_text(encoding='utf-8'))
        else:
            warnings.append('Offline smoke report missing (smoke/offline_smoke_report.json)')
            smoke={"status":"BLOCKED"}
    status='READY'
    if blockers: status='BLOCKED'
    elif acceptance.get('status') in {'BLOCKED','MISSING_ENVIRONMENT_YAML'}: status='BLOCKED'
    elif acceptance.get('status')=='PREVIEW_ONLY': status='PREVIEW_ONLY'

    scene_name=scene.name
    build_cmd=f"colcon build --symlink-install --packages-select {scene_name}"
    launch_cmd=f"ros2 launch {scene_name} demo.launch.py use_fake_hardware:=true"
    report={
      'scene_name':scene_name,'scene_path':str(scene),'status':status,
      'acceptance_status':acceptance.get('status','BLOCKED'),'smoke_status':smoke.get('status','BLOCKED'),
      'preview_paths':{'html':str(scene/'preview'/'static_preview.html'),'svg':str(scene/'preview'/'static_preview.svg')},
      'readiness_paths':{'pack_manifest':str(scene/'readiness_pack'/'readiness_pack_manifest.json')},
      'dashboard_link':str(demo_dir/'workcell_studio_demo_dashboard.html'),'build_command':build_cmd,
      'fake_hardware_launch_command':launch_cmd,'robot_tool':'unknown','gripper_mount_rpy':'-1.5708 -1.5708 0',
      'blockers':blockers,'warnings':warnings,'next_commands':[build_cmd,launch_cmd],
      'safety_flags':{'no_robot_motion_commanded':True,'use_fake_hardware':True,'runtime_execution_enabled':False},
      'layout_merge_status':'READY' if acceptance.get('layout_applied') else 'MISSING',
      'saved_layout_timestamp': acceptance.get('saved_layout_timestamp','unknown'),
      'merge_timestamp': acceptance.get('merge_timestamp','unknown'),
      'layout_applied': acceptance.get('layout_applied',False),
      'layout_stale': acceptance.get('layout_stale',False)
    }
    (demo_dir/'workcell_studio_demo_report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    summary='\n'.join([
      f"scene_name={scene_name}",f"status={status}",f"acceptance_status={report['acceptance_status']}",f"smoke_status={report['smoke_status']}",
      'no robot motion commanded','use_fake_hardware:=true','runtime_execution_enabled false',launch_cmd
    ])
    (demo_dir/'workcell_studio_demo_summary.txt').write_text(summary+'\n',encoding='utf-8')
    _write_dashboard(report,demo_dir/'workcell_studio_demo_dashboard.html')
    return report

if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('scene_dir',type=Path); ap.add_argument('--json',action='store_true')
    a=ap.parse_args(); r=run(a.scene_dir)
    if a.json: print(json.dumps(r,indent=2))
    else: print(f"{r['status']}: {r['scene_name']}")
    raise SystemExit(0 if r['status'] in {'READY','PREVIEW_ONLY'} else 1)
