#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess, sys, re, hashlib, shutil
from datetime import datetime, timezone
from pathlib import Path
import yaml
from workcell_studio_error_messages import get_message
REPO_ROOT=Path(__file__).resolve().parents[1]; SCRIPTS=REPO_ROOT/'scripts'; DEFAULT_SCENE='scratch_ur5_2f_acceptance'
REQUIRED=['environment.yaml','layout/workcell_studio_layout.yaml','config/workcell_builder_task_intent.yaml','cell_definition.yaml','scene_manifest.yaml','package.xml','CMakeLists.txt','launch/demo.launch.py']

def _safe_scene_dir(root:Path,name:str)->Path:
 d=root/name; i=1
 while d.exists(): d=root/f"{name}_{i}"; i+=1
 return d

def _run(cmd:list[str]):
 p=subprocess.run(cmd,capture_output=True,text=True,check=False); return p.returncode,p.stdout,p.stderr
CELL_TMPL='''schema_version: cell_definition/v1
cell:
  id: {scene}
  name: {scene}
  description: Scratch acceptance cell definition
robot:
  model: ur5
  planning_group: manipulator
  base_frame: world
  tool_link: tool0
  home_named_target: home
  safe_joint_state: []
end_effector:
  id: robotiq_2f
  type: finger
  brand: robotiq
  grasp_frame: tool0
  allowed_touch_links: [left_inner_finger, right_inner_finger]
camera:
  id: realsense_d435i
  type: depth_camera
  frame: camera_depth_optical_frame
  capability: sensor/depth_camera/realsense_d435i
environment:
  frame: world
  layout: layout/workcell_studio_layout.yaml
  support_surfaces:
    - id: table_main
      type: table
      frame: world
      pose_xyz: [0.0, 0.0, 0.0]
      pose_rpy: [0.0, 0.0, 0.0]
      dimensions: [1.0, 1.0, 0.05]
objects:
  - id: pick_part
    role: pick
    class: part
    shape: box
    color: unknown
    material: plastic
    frame: world
    dimensions: [0.05, 0.05, 0.05]
    pose_xyz: [0.45, 0.0, 0.08]
    pose_rpy: [0.0, 0.0, 0.0]
  - id: place_bin
    role: destination
    class: bin
    shape: box
    color: blue
    material: plastic
    frame: world
    dimensions: [0.20, 0.20, 0.12]
    pose_xyz: [0.65, -0.25, 0.06]
    pose_rpy: [0.0, 0.0, 0.0]
task:
  id: scratch_pick_place
  type: pick_place
  source_object: pick_part
  destinations:
    - id: place_bin
      frame: world
      pose_xyz: [0.65, -0.25, 0.12]
      pose_rpy: [0.0, 0.0, 0.0]
  rules:
    - id: default_place
      when:
        always: true
      destination: place_bin
commissioning:
  self_test_enabled: true
  export_bundle: true
  require_operator_review: true
  fake_hardware_default: true
  demo_template_id: scratch_ur5_2f_acceptance
'''

def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--scene-name',default=DEFAULT_SCENE); ap.add_argument('--output-root',type=Path,default=Path('/tmp/workcell_studio_scratch_acceptance')); ap.add_argument('--json-out',type=Path); ap.add_argument('--run-id',default=''); a=ap.parse_args()
 run_id=a.run_id or datetime.now(timezone.utc).isoformat()
 r={'run_id':run_id,'scene_name':a.scene_name,'scene_dir':'','messages_format':'standard_v1','generated_files':[],'missing_files':[],'validation_status':'BLOCKED','blockers':[],'warnings':[],'build_command':'','source_command':'source install/setup.bash','launch_command':'','ready_for_plan_simulate':False,'failure_step':'generation','failure_summary':'','next_failed_step':'generation','schema_preflight':{},'generator':{},'file_output_audit':{}}
 if not re.fullmatch(r'[a-z][a-z0-9_]*', a.scene_name): r['blockers'].append(get_message('INVALID_SCENE_NAME'))
 try: a.output_root.mkdir(parents=True,exist_ok=True)
 except Exception as exc: r['blockers'].append(get_message('MISSING_WORKSPACE',detail=f'invalid output root: {a.output_root} ({exc})')); print(json.dumps(r,indent=2)); return 1
 sd=_safe_scene_dir(a.output_root,a.scene_name); sd.mkdir(parents=True,exist_ok=True); r['scene_dir']=str(sd)
 if sd.name!=a.scene_name: r['warnings'].append(get_message('SCENE_ALREADY_EXISTS',detail=f'Scene already existed; using {sd.name}'))
 cell_tmp=a.output_root/f".{sd.name}.cell_definition.yaml"; cell_tmp.write_text(CELL_TMPL.format(scene=sd.name),encoding='utf-8')
 cell=cell_tmp
 cell_text=cell.read_text(encoding='utf-8')
 cell_sha256=hashlib.sha256(cell_text.encode('utf-8')).hexdigest()
 cell_doc=yaml.safe_load(cell_text) if cell_text.strip() else {}
 top_level_keys=sorted(list(cell_doc.keys())) if isinstance(cell_doc,dict) else []
 objects=cell_doc.get('objects',[]) if isinstance(cell_doc,dict) else []
 task_destinations=((cell_doc.get('task') or {}).get('destinations') or []) if isinstance(cell_doc,dict) else []
 preflight_cmd=[sys.executable,str(SCRIPTS/'validate_cell_definition.py'),str(cell),'--json']
 preflight_rc,preflight_stdout,preflight_stderr=_run(preflight_cmd)
 preflight_out=(preflight_stdout+'\n'+preflight_stderr).strip()
 preflight_json={}
 try:
  preflight_json=json.loads(preflight_out)
 except json.JSONDecodeError:
  preflight_json={}
 preflight_errors=list(preflight_json.get('errors',[])) if isinstance(preflight_json,dict) else []
 preflight_warnings=list(preflight_json.get('warnings',[])) if isinstance(preflight_json,dict) else []
 r['schema_preflight']={
  'validator_command':' '.join(preflight_cmd),
  'validator_returncode':preflight_rc,
  'cell_definition_path':str(cell.resolve()),
  'cell_definition_sha256':cell_sha256,
  'cell_definition_top_level_keys':top_level_keys,
  'object_count':len(objects) if isinstance(objects,list) else 0,
  'task_destination_count':len(task_destinations) if isinstance(task_destinations,list) else 0,
  'stdout_stderr':preflight_out,
  'stdout_stderr_tail':'\n'.join(preflight_out.splitlines()[-120:]),
  'schema_blockers':preflight_errors,
  'warnings':preflight_warnings,
  'generated_file_list': [str(p.relative_to(sd)) for p in sorted(sd.rglob('*')) if p.is_file()],
  'missing_package_files': [],
  'next_failed_step': 'generation',
 }
 if preflight_rc!=0 or preflight_errors:
  r['failure_step']='schema_preflight'
  r['next_failed_step']='schema_preflight'
  r['blockers'].extend([f"schema_preflight: {e}" for e in preflight_errors] or ['schema_preflight: validator returned non-zero exit code'])
  out=json.dumps(r,indent=2)
  if a.json_out: a.json_out.parent.mkdir(parents=True,exist_ok=True); a.json_out.write_text(out+'\n',encoding='utf-8')
  print(out); return 1
 generator_cmd=[sys.executable,str(SCRIPTS/'generate_workcell_from_cell_definition.py'),str(cell),'--output-dir',str(a.output_root),'--package-name',sd.name,'--force']
 rc,generator_stdout,generator_stderr=_run(generator_cmd)
 generator_stdout_tail='\n'.join(generator_stdout.splitlines()[-120:])
 generator_stderr_tail='\n'.join(generator_stderr.splitlines()[-120:])
 r['generator']={'command':' '.join(generator_cmd),'returncode':rc,'stdout_tail':generator_stdout_tail,'stderr_tail':generator_stderr_tail}
 if rc==0 and not (sd/'cell_definition.yaml').exists() and cell.exists():
  shutil.copy2(cell, sd/'cell_definition.yaml')
 if rc!=0:
  summary=(generator_stderr or generator_stdout).splitlines()[-1] if (generator_stderr or generator_stdout) else 'Generator failed.'
  r['failure_step']='generation'
  r['failure_summary']=f'generation failed (rc={rc}): {summary}'
  r['generator']['failure_summary']=r['failure_summary']
  r['blockers'].append(get_message('VALIDATION_BLOCKED',title='Generate Scene Package failed',detail=f"{summary} (see JSON: generator.returncode/stdout_tail/stderr_tail/failure_summary)"))
 for rel,txt in [('layout/workcell_studio_layout.yaml',f'schema_version: workcell_studio_layout/v1\nscene_name: {sd.name}\nitems: []\n'),('config/workcell_builder_task_intent.yaml','task:\n  type: pick_place\n'),('environment.yaml',f'scene_name: {sd.name}\n')]:
  p=sd/rel; p.parent.mkdir(parents=True,exist_ok=True)
  if not p.exists(): p.write_text(txt,encoding='utf-8')
 r['launch_command']=f'ros2 launch {sd.name} demo.launch.py use_fake_hardware:=true launch_rviz:=true'; r['build_command']=f'colcon build --symlink-install --packages-select {sd.name}'
 for rel in REQUIRED: (r['generated_files'] if (sd/rel).exists() else r['missing_files']).append(rel)
 r['schema_preflight']['missing_package_files']=list(r['missing_files'])
 if 'launch/demo.launch.py' in r['missing_files']: r['blockers'].append(get_message('MISSING_DEMO_LAUNCH'))
 if 'use_fake_hardware:=false' in r['launch_command']: r['blockers'].append(get_message('MISSING_FAKE_HARDWARE_ARG', detail='Unsafe launch command includes use_fake_hardware:=false'))
 if not list(REPO_ROOT.rglob('*ur5*')): r['warnings'].append(get_message('VALIDATION_BLOCKED', title='Missing UR5 assets', detail='Repository search did not find ur5 tokens.'))
 if not (list(REPO_ROOT.rglob('*robotiq*2f*'))+list(REPO_ROOT.rglob('*2f_85*'))): r['warnings'].append(get_message('VALIDATION_BLOCKED', title='Missing Robotiq 2F assets', detail='Repository search did not find robotiq 2f tokens.'))
 audit_cmd=[sys.executable,str(SCRIPTS/'audit_new_cell_file_outputs.py'),'--scene-dir',str(sd),'--scene-name',sd.name,'--json-out',str(sd/'file_output_audit.json')]
 audit_rc,audit_stdout,audit_stderr=_run(audit_cmd)
 r['file_output_audit']={'command':' '.join(audit_cmd),'returncode':audit_rc,'stdout_tail':'\n'.join(audit_stdout.splitlines()[-120:]),'stderr_tail':'\n'.join(audit_stderr.splitlines()[-120:])}
 if audit_rc!=0:
  r['blockers'].append(f'file_output_audit failed (rc={audit_rc})')
  r['next_failed_step']='file_output_audit'
 r['validation_status']='PASS' if not r['blockers'] and not r['missing_files'] else 'BLOCKED'; r['ready_for_plan_simulate']=r['validation_status']=='PASS'
 out=json.dumps(r,indent=2)
 if a.json_out: a.json_out.parent.mkdir(parents=True,exist_ok=True); a.json_out.write_text(out+'\n',encoding='utf-8')
 print(out); return 0 if r['ready_for_plan_simulate'] else 1
if __name__=='__main__': raise SystemExit(main())
