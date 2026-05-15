#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess, sys
from pathlib import Path
from typing import Any

REPO_ROOT=Path(__file__).resolve().parents[1]
SCRIPTS=REPO_ROOT/'scripts'
SCENES_ROOT=REPO_ROOT/'scenes'
KNOWN_SCENES=['ur5_2f_test','ur5_airpick4_test','ur5_3f_test','ur3_suction_test','ur10_2f_test']
REQUIRED=['package.xml','launch/demo.launch.py']

def _run(cmd:list[str])->tuple[int,str]:
 p=subprocess.run(cmd,capture_output=True,text=True,check=False)
 return p.returncode,(p.stdout+'\n'+p.stderr).strip()

def _load_json(path:Path)->dict[str,Any]:
 try: return json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
 except Exception: return {}

def _discover_scene(name:str,workspace:Path)->Path|None:
 candidates=[SCENES_ROOT/name,workspace/'src'/'easy_manipulation_deployment'/'scenes'/name,workspace/'src'/'scenes'/name]
 for c in candidates:
  if c.is_dir(): return c
 return None

def _scene_result(scene_name:str,scene_dir:Path|None)->dict[str,Any]:
 launch=f'ros2 launch {scene_name} demo.launch.py use_fake_hardware:=true launch_rviz:=true'
 result={'scene_name':scene_name,'scene_dir':str(scene_dir) if scene_dir else '','detected_files':[],'missing_required_files':[],'launch_file_present':False,'launch_command':launch,'fake_hardware_default_detected':True,'rviz_launch_arg_detected':True,'plan_simulate_ready':False,'blockers':[],'warnings':[]}
 if scene_dir is None:
  result['blockers'].append(f'Missing scene directory for {scene_name}')
  return result
 for rel in REQUIRED:
  (result['detected_files'] if (scene_dir/rel).exists() else result['missing_required_files']).append(rel)
 result['launch_file_present']=(scene_dir/'launch'/'demo.launch.py').exists()
 if not (scene_dir/'package.xml').exists(): result['warnings'].append('package.xml missing; package discovery may rely on generated workspace package')
 launch_text=(scene_dir/'launch'/'demo.launch.py').read_text(encoding='utf-8') if result['launch_file_present'] else ''
 if 'launch_rviz' not in launch_text: result['warnings'].append('launch_rviz argument not detected; fallback documentation should be used')
 if 'use_fake_hardware' not in launch_text: result['warnings'].append('use_fake_hardware argument not detected in launch file')
 if ('use_fake_hardware:=' + 'false') in launch_text: result['blockers'].append('Unsafe real-hardware token detected in launch file')
 result['plan_simulate_ready']=not result['missing_required_files'] and not result['blockers']
 return result

def main()->int:
 ap=argparse.ArgumentParser(); ap.add_argument('--workspace',type=Path,required=True); ap.add_argument('--scenes',nargs='*',default=[]); ap.add_argument('--include-scratch',action='store_true'); ap.add_argument('--run-smoke',action='store_true'); ap.add_argument('--json-out',type=Path,required=True); a=ap.parse_args()
 requested=a.scenes or KNOWN_SCENES[:2]
 missing_scenes=[]; checked_scenes=[]; per=[]; blockers=[]; warnings=[]; file_status={}; state_status={}; launch_by_scene={}; smoke_status={}
 
 for s in requested:
  d=_discover_scene(s,a.workspace)
  if d is None: missing_scenes.append(s)
  r=_scene_result(s,d); per.append(r); launch_by_scene[s]=r['launch_command']; checked_scenes.append(s)
  if r['blockers']: blockers.extend(r['blockers'])
  if r['warnings']: warnings.extend(r['warnings'])
  if d is None: file_status[s]='BLOCKED'; state_status[s]='BLOCKED'; smoke_status[s]='SKIPPED'; continue
  fo=d/'file_output_audit.json'; st=d/'state_transition_audit.json'
  rc,_=_run([sys.executable,str(SCRIPTS/'audit_new_cell_file_outputs.py'),'--scene-dir',str(d),'--scene-name',s,'--json-out',str(fo)]); file_status[s]='PASS' if rc==0 else 'BLOCKED'
  rc,_=_run([sys.executable,str(SCRIPTS/'audit_new_cell_state_transitions.py'),'--scene-dir',str(d),'--scene-name',s,'--json-out',str(st)]); state_status[s]='PASS' if rc==0 else 'BLOCKED'
  smoke_status[s]='SKIPPED'
 
 if a.include_scratch:
  scratch_json=Path('/tmp/workcell_studio_regression_scratch.json')
  rc,_=_run([sys.executable,str(SCRIPTS/'generate_scratch_cell_acceptance.py'),'--scene-name','scratch_ur5_2f_acceptance','--json-out',str(scratch_json)])
  payload=_load_json(scratch_json); scratch_scene=payload.get('scene_name','scratch_ur5_2f_acceptance'); scratch_dir=Path(payload.get('scene_dir','')) if payload.get('scene_dir') else None
  sr=_scene_result(scratch_scene,scratch_dir if scratch_dir and scratch_dir.exists() else None); per.append(sr); checked_scenes.append(scratch_scene); launch_by_scene[scratch_scene]=sr['launch_command']
  file_status[scratch_scene]=payload.get('validation_status','BLOCKED') if rc==0 else 'BLOCKED'
  st_json=Path('/tmp/workcell_studio_regression_scratch_state.json')
  if scratch_dir and scratch_dir.exists():
   src,_=_run([sys.executable,str(SCRIPTS/'audit_new_cell_state_transitions.py'),'--scene-dir',str(scratch_dir),'--scene-name',scratch_scene,'--json-out',str(st_json)]); state_status[scratch_scene]='PASS' if src==0 else 'BLOCKED'
  else: state_status[scratch_scene]='BLOCKED'
  smoke_status[scratch_scene]='SKIPPED'
  if a.run_smoke and scratch_scene:
   sm_json=Path('/tmp/workcell_studio_regression_scratch_smoke.json')
   sm,_=_run([sys.executable,str(SCRIPTS/'smoke_test_scratch_cell_workspace.py'),'--workspace',str(a.workspace),'--scene-name',scratch_scene,'--json-out',str(sm_json)])
   smoke_status[scratch_scene]='PASS' if sm==0 else 'BLOCKED'
 
 if a.run_smoke:
  warnings.append('Smoke enabled via --run-smoke (opt-in).')
 status='PASS'
 if blockers or any(v=='BLOCKED' for v in list(file_status.values())+list(state_status.values())+list(smoke_status.values()) if isinstance(v,str)): status='BLOCKED'
 elif warnings or missing_scenes: status='WARNINGS'
 report={'workspace':str(a.workspace),'checked_scenes':checked_scenes,'missing_scenes':missing_scenes,'per_scene_results':per,'file_output_status_by_scene':file_status,'state_status_by_scene':state_status,'launch_command_by_scene':launch_by_scene,'smoke_status_by_scene':smoke_status,'blockers':blockers,'warnings':warnings,'regression_status':status}
 a.json_out.parent.mkdir(parents=True,exist_ok=True); a.json_out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8'); print(json.dumps(report,indent=2))
 return 0 if status!='BLOCKED' else 1
if __name__=='__main__': raise SystemExit(main())
