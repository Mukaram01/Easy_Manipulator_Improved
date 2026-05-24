#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess, sys, re
from pathlib import Path
from workcell_studio_error_messages import get_message
REPO_ROOT=Path(__file__).resolve().parents[1]; SCRIPTS=REPO_ROOT/'scripts'; DEFAULT_SCENE='scratch_ur5_2f_acceptance'
REQUIRED=['environment.yaml','environment_layout.yaml','config/workcell_builder_task_intent.yaml','cell_definition.yaml','scene_manifest.yaml','package.xml','CMakeLists.txt','launch/demo.launch.py']

def _safe_scene_dir(root:Path,name:str)->Path:
 d=root/name; i=1
 while d.exists(): d=root/f"{name}_{i}"; i+=1
 return d

def _run(cmd:list[str]):
 p=subprocess.run(cmd,capture_output=True,text=True,check=False); return p.returncode,p.stdout,p.stderr
CELL_TMPL='''schema_version: cell_definition/v1\ncell:\n  id: {scene}\n  name: {scene}\nrobot:\n  model: ur5\n  safe_joint_state: []\n  home_named_target: home\nend_effector:\n  id: robotiq_2f\nenvironment:\n  layout: environment_layout.yaml\ntask:\n  type: pick_place\n  pick:\n    source: pick_zone\n  place:\n    destination: place_bin\n  destinations:\n    - id: place_bin\n      frame: world\n      pose_xyz: [0.65, -0.25, 0.75]\n      pose_rpy: [0.0, 0.0, 0.0]\nassets:\n  - id: table_support\n    kind: table\n  - id: pick_zone\n    kind: pick_zone\n  - id: place_bin\n    kind: bin\ncommissioning:\n  self_test_enabled: true\n  export_bundle: true\n  require_operator_review: true\n  fake_hardware_default: true\n  demo_template_id: scratch_ur5_2f_acceptance\n'''

def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--scene-name',default=DEFAULT_SCENE); ap.add_argument('--output-root',type=Path,default=Path('/tmp/workcell_studio_scratch_acceptance')); ap.add_argument('--json-out',type=Path); a=ap.parse_args()
 r={'scene_name':a.scene_name,'scene_dir':'','messages_format':'standard_v1','generated_files':[],'missing_files':[],'validation_status':'BLOCKED','blockers':[],'warnings':[],'build_command':'','source_command':'source install/setup.bash','launch_command':'','ready_for_plan_simulate':False,'step':'initialization','schema_preflight':{}}
 if not re.fullmatch(r'[a-z][a-z0-9_]*', a.scene_name): r['blockers'].append(get_message('INVALID_SCENE_NAME'))
 try: a.output_root.mkdir(parents=True,exist_ok=True)
 except Exception as exc: r['blockers'].append(get_message('MISSING_WORKSPACE',detail=f'invalid output root: {a.output_root} ({exc})')); print(json.dumps(r,indent=2)); return 1
 sd=_safe_scene_dir(a.output_root,a.scene_name); sd.mkdir(parents=True,exist_ok=True); r['scene_dir']=str(sd)
 if sd.name!=a.scene_name: r['warnings'].append(get_message('SCENE_ALREADY_EXISTS',detail=f'Scene already existed; using {sd.name}'))
 cell=sd/'cell_definition.yaml'; cell.write_text(CELL_TMPL.format(scene=sd.name),encoding='utf-8')
 preflight_cmd=[sys.executable,str(SCRIPTS/'validate_cell_definition.py'),str(cell),'--json']
 preflight_rc,preflight_stdout,preflight_stderr=_run(preflight_cmd)
 raw_combined=((preflight_stdout or '')+'\n'+(preflight_stderr or '')).strip()
 payload={}
 try:
  payload=json.loads(raw_combined) if raw_combined else {}
 except Exception:
  payload={}
 schema_blockers=[]
 for msg in payload.get('errors',[]) if isinstance(payload,dict) else []:
  schema_blockers.append({'severity':'error','message':str(msg)})
 for msg in payload.get('warnings',[]) if isinstance(payload,dict) else []:
  schema_blockers.append({'severity':'warning','message':str(msg)})
 if not schema_blockers and raw_combined:
  for line in raw_combined.splitlines():
   if line.startswith('FAIL: '): schema_blockers.append({'severity':'error','message':line[6:].strip()})
   elif line.startswith('WARN: '): schema_blockers.append({'severity':'warning','message':line[6:].strip()})
 r['schema_preflight']={
  'command':' '.join(preflight_cmd),
  'returncode':preflight_rc,
  'stdout_tail':'\n'.join(preflight_stdout.splitlines()[-200:]),
  'stderr_tail':'\n'.join(preflight_stderr.splitlines()[-200:]),
  'output_tail':'\n'.join(raw_combined.splitlines()[-200:]),
  'schema_blockers':schema_blockers,
 }
 if preflight_rc!=0:
  r['validation_status']='BLOCKED'; r['step']='schema_preflight'; r['blockers'].append(get_message('VALIDATION_BLOCKED',title='Schema preflight failed',detail='cell_definition.yaml failed validation preflight.')); r['blockers'].extend([f"{b['severity']}: {b['message']}" for b in schema_blockers]); out=json.dumps(r,indent=2)
  if a.json_out: a.json_out.parent.mkdir(parents=True,exist_ok=True); a.json_out.write_text(out+'\n',encoding='utf-8')
  print(out); return 1
 r['step']='generation'
 rc,gen_stdout,gen_stderr=_run([sys.executable,str(SCRIPTS/'generate_workcell_from_cell_definition.py'),str(cell),'--output-dir',str(a.output_root),'--package-name',sd.name,'--force'])
 gen_out=((gen_stdout or '')+'\n'+(gen_stderr or '')).strip()
 if rc!=0: r['blockers'].append(get_message('VALIDATION_BLOCKED',title='Generate Scene Package failed',detail=gen_out.splitlines()[-1] if gen_out else 'Generator failed.'))
 for rel,txt in [('environment_layout.yaml','schema_version: environment_layout/v1\nassets: []\n'),('config/workcell_builder_task_intent.yaml','task:\n  type: pick_place\n'),('environment.yaml',f'scene_name: {sd.name}\n')]:
  p=sd/rel; p.parent.mkdir(parents=True,exist_ok=True)
  if not p.exists(): p.write_text(txt,encoding='utf-8')
 r['launch_command']=f'ros2 launch {sd.name} demo.launch.py use_fake_hardware:=true launch_rviz:=true'; r['build_command']=f'colcon build --symlink-install --packages-select {sd.name}'
 for rel in REQUIRED: (r['generated_files'] if (sd/rel).exists() else r['missing_files']).append(rel)
 if 'launch/demo.launch.py' in r['missing_files']: r['blockers'].append(get_message('MISSING_DEMO_LAUNCH'))
 if 'use_fake_hardware:=false' in r['launch_command']: r['blockers'].append(get_message('MISSING_FAKE_HARDWARE_ARG', detail='Unsafe launch command includes use_fake_hardware:=false'))
 if not list(REPO_ROOT.rglob('*ur5*')): r['warnings'].append(get_message('VALIDATION_BLOCKED', title='Missing UR5 assets', detail='Repository search did not find ur5 tokens.'))
 if not (list(REPO_ROOT.rglob('*robotiq*2f*'))+list(REPO_ROOT.rglob('*2f_85*'))): r['warnings'].append(get_message('VALIDATION_BLOCKED', title='Missing Robotiq 2F assets', detail='Repository search did not find robotiq 2f tokens.'))
 _rc,_out,_err=_run([sys.executable,str(SCRIPTS/'audit_new_cell_file_outputs.py'),'--scene-dir',str(sd),'--scene-name',sd.name,'--json-out',str(sd/'file_output_audit.json')])
 r['validation_status']='PASS' if not r['blockers'] and not r['missing_files'] else 'BLOCKED'; r['ready_for_plan_simulate']=r['validation_status']=='PASS'
 out=json.dumps(r,indent=2)
 if a.json_out: a.json_out.parent.mkdir(parents=True,exist_ok=True); a.json_out.write_text(out+'\n',encoding='utf-8')
 print(out); return 0 if r['ready_for_plan_simulate'] else 1
if __name__=='__main__': raise SystemExit(main())
