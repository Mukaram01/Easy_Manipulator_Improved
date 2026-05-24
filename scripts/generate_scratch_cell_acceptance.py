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
 p=subprocess.run(cmd,capture_output=True,text=True,check=False); return p.returncode,(p.stdout+'\n'+p.stderr).strip()
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
environment:
  frame: world
  layout: environment_layout.yaml
  support_surfaces:
    - id: table_main
      type: table
      frame: world
      pose_xyz: [0.0, 0.0, 0.0]
      pose_rpy: [0.0, 0.0, 0.0]
      dimensions: [1.0, 1.0, 0.05]
objects:
  - id: pick_part
    class: part
    shape: box
    color: unknown
    material: plastic
    frame: world
    dimensions: [0.05, 0.05, 0.05]
    pose_xyz: [0.45, 0.0, 0.08]
    pose_rpy: [0.0, 0.0, 0.0]
  - id: place_bin
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
 ap=argparse.ArgumentParser(); ap.add_argument('--scene-name',default=DEFAULT_SCENE); ap.add_argument('--output-root',type=Path,default=Path('/tmp/workcell_studio_scratch_acceptance')); ap.add_argument('--json-out',type=Path); a=ap.parse_args()
 r={'scene_name':a.scene_name,'scene_dir':'','messages_format':'standard_v1','generated_files':[],'missing_files':[],'validation_status':'BLOCKED','blockers':[],'warnings':[],'build_command':'','source_command':'source install/setup.bash','launch_command':'','ready_for_plan_simulate':False}
 if not re.fullmatch(r'[a-z][a-z0-9_]*', a.scene_name): r['blockers'].append(get_message('INVALID_SCENE_NAME'))
 try: a.output_root.mkdir(parents=True,exist_ok=True)
 except Exception as exc: r['blockers'].append(get_message('MISSING_WORKSPACE',detail=f'invalid output root: {a.output_root} ({exc})')); print(json.dumps(r,indent=2)); return 1
 sd=_safe_scene_dir(a.output_root,a.scene_name); sd.mkdir(parents=True,exist_ok=True); r['scene_dir']=str(sd)
 if sd.name!=a.scene_name: r['warnings'].append(get_message('SCENE_ALREADY_EXISTS',detail=f'Scene already existed; using {sd.name}'))
 cell=sd/'cell_definition.yaml'; cell.write_text(CELL_TMPL.format(scene=sd.name),encoding='utf-8')
 rc,out=_run([sys.executable,str(SCRIPTS/'generate_workcell_from_cell_definition.py'),str(cell),'--output-dir',str(a.output_root),'--package-name',sd.name,'--force'])
 if rc!=0: r['blockers'].append(get_message('VALIDATION_BLOCKED',title='Generate Scene Package failed',detail=out.splitlines()[-1] if out else 'Generator failed.'))
 for rel,txt in [('environment_layout.yaml','schema_version: environment_layout/v1\nassets: []\n'),('config/workcell_builder_task_intent.yaml','task:\n  type: pick_place\n'),('environment.yaml',f'scene_name: {sd.name}\n')]:
  p=sd/rel; p.parent.mkdir(parents=True,exist_ok=True)
  if not p.exists(): p.write_text(txt,encoding='utf-8')
 r['launch_command']=f'ros2 launch {sd.name} demo.launch.py use_fake_hardware:=true launch_rviz:=true'; r['build_command']=f'colcon build --symlink-install --packages-select {sd.name}'
 for rel in REQUIRED: (r['generated_files'] if (sd/rel).exists() else r['missing_files']).append(rel)
 if 'launch/demo.launch.py' in r['missing_files']: r['blockers'].append(get_message('MISSING_DEMO_LAUNCH'))
 if 'use_fake_hardware:=false' in r['launch_command']: r['blockers'].append(get_message('MISSING_FAKE_HARDWARE_ARG', detail='Unsafe launch command includes use_fake_hardware:=false'))
 if not list(REPO_ROOT.rglob('*ur5*')): r['warnings'].append(get_message('VALIDATION_BLOCKED', title='Missing UR5 assets', detail='Repository search did not find ur5 tokens.'))
 if not (list(REPO_ROOT.rglob('*robotiq*2f*'))+list(REPO_ROOT.rglob('*2f_85*'))): r['warnings'].append(get_message('VALIDATION_BLOCKED', title='Missing Robotiq 2F assets', detail='Repository search did not find robotiq 2f tokens.'))
 _rc,_out=_run([sys.executable,str(SCRIPTS/'audit_new_cell_file_outputs.py'),'--scene-dir',str(sd),'--scene-name',sd.name,'--json-out',str(sd/'file_output_audit.json')])
 r['validation_status']='PASS' if not r['blockers'] and not r['missing_files'] else 'BLOCKED'; r['ready_for_plan_simulate']=r['validation_status']=='PASS'
 out=json.dumps(r,indent=2)
 if a.json_out: a.json_out.parent.mkdir(parents=True,exist_ok=True); a.json_out.write_text(out+'\n',encoding='utf-8')
 print(out); return 0 if r['ready_for_plan_simulate'] else 1
if __name__=='__main__': raise SystemExit(main())
