#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess, sys
from pathlib import Path
REPO_ROOT=Path(__file__).resolve().parents[1]; SCRIPTS=REPO_ROOT/'scripts'; DEFAULT_SCENE='scratch_ur5_2f_acceptance'
REQUIRED=['environment.yaml','environment_layout.yaml','config/workcell_builder_task_intent.yaml','cell_definition.yaml','scene_manifest.yaml','package.xml','CMakeLists.txt','launch/demo.launch.py']

def _safe_scene_dir(root:Path,name:str)->Path:
 d=root/name; i=1
 while d.exists(): d=root/f"{name}_{i}"; i+=1
 return d

def _run(cmd:list[str]):
 p=subprocess.run(cmd,capture_output=True,text=True,check=False); return p.returncode,(p.stdout+'\n'+p.stderr).strip()

CELL_TMPL='''cell:\n  id: {scene}\n  name: {scene}\nrobot:\n  model: ur5\n  planning_group: manipulator\n  base_frame: world\n  tool_link: tool0\n  home_named_target: home\n  safe_joint_state: []\nend_effector:\n  id: robotiq_2f\n  type: finger\n  brand: robotiq\n  grasp_frame: ee_palm\n  allowed_touch_links: [gripper_finger1_finger_tip_link, gripper_finger2_finger_tip_link]\ncamera:\n  id: realsense_d435i\n  type: depth_camera\n  frame: camera_depth_optical_frame\n  pointcloud_topic: /camera/camera/depth/color/points\ntask:\n  type: pick_place\nenvironment:\n  layout: environment_layout.yaml\n'''

def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--scene-name',default=DEFAULT_SCENE); ap.add_argument('--output-root',type=Path,default=Path('/tmp/workcell_studio_scratch_acceptance')); ap.add_argument('--json-out',type=Path); ap.add_argument('--run-file-output-audit',action='store_true',default=True); a=ap.parse_args()
 r={'scene_name':a.scene_name,'scene_dir':'','generated_files':[],'missing_files':[],'validation_status':'BLOCKED','blockers':[],'warnings':[],'build_command':'','source_command':'source install/setup.bash','launch_command':'','ready_for_plan_simulate':False,'file_output_audit':{}}
 try:a.output_root.mkdir(parents=True,exist_ok=True)
 except Exception as exc:r['blockers'].append(f'invalid output root: {a.output_root} ({exc})'); print(json.dumps(r,indent=2)); return 1
 sd=_safe_scene_dir(a.output_root,a.scene_name); sd.mkdir(parents=True,exist_ok=True); r['scene_dir']=str(sd)
 if not list(REPO_ROOT.rglob('*ur5*')): r['blockers'].append('Missing UR5 assets; searched repo for *ur5* under: '+str(REPO_ROOT))
 if not (list(REPO_ROOT.rglob('*robotiq*2f*'))+list(REPO_ROOT.rglob('*2f_85*'))): r['blockers'].append('Missing Robotiq 2F assets; searched repo for *robotiq*2f* and *2f_85* under: '+str(REPO_ROOT))
 cell=sd/'cell_definition.yaml'; cell.write_text(CELL_TMPL.format(scene=sd.name),encoding='utf-8')
 rc,out=_run([sys.executable,str(SCRIPTS/'generate_workcell_from_cell_definition.py'),str(cell),'--output-dir',str(a.output_root),'--package-name',sd.name,'--force'])
 if rc!=0:r['blockers'].append('Failed to generate workcell package: '+out.splitlines()[-1])
 layout=sd/'environment_layout.yaml';
 if not layout.exists(): layout.write_text('schema_version: environment_layout/v1\nlayout_id: scratch_default\nassets: []\nzones: []\n',encoding='utf-8'); r['warnings'].append('Generated fallback environment_layout.yaml; review before runtime use.')
 ti=sd/'config'/'workcell_builder_task_intent.yaml';
 if not ti.exists(): ti.parent.mkdir(parents=True,exist_ok=True); ti.write_text('schema: workcell_builder_task_intent/v1\ntask:\n  type: pick_place\n',encoding='utf-8'); r['warnings'].append('Generated minimal task intent fallback file.')
 env=sd/'environment.yaml';
 if not env.exists(): env.write_text(f'scene_name: {sd.name}\n',encoding='utf-8'); r['warnings'].append('Generated legacy environment.yaml compatibility file.')
 r['launch_command']=f'ros2 launch {sd.name} demo.launch.py use_fake_hardware:=true launch_rviz:=true'; r['build_command']=f'colcon build --symlink-install --packages-select {sd.name}'
 for rel in REQUIRED:
  (r['generated_files'] if (sd/rel).exists() else r['missing_files']).append(rel)
 urdf_dir=sd/'urdf'
 if not ((urdf_dir/'environment.urdf.xacro').exists() or (urdf_dir.exists() and list(urdf_dir.glob('*.xacro')))): r['missing_files'].append('urdf/environment.urdf.xacro or equivalent')
 if 'use_fake_hardware:=false' in r['launch_command']: r['blockers'].append('Generated unsafe launch command with use_fake_hardware:=false')

 audit_json=sd/'file_output_audit.json'
 if a.run_file_output_audit:
  arc,aout=_run([sys.executable,str(SCRIPTS/'audit_new_cell_file_outputs.py'),'--scene-dir',str(sd),'--scene-name',sd.name,'--json-out',str(audit_json)])
  if audit_json.exists():
   try:r['file_output_audit']=json.loads(audit_json.read_text(encoding='utf-8'))
   except Exception:r['warnings'].append('file-output audit json could not be parsed')
  if arc!=0:r['warnings'].append('File-output audit reported non-PASS status')
 
 for cmd in [[sys.executable,str(SCRIPTS/'validate_cell_definition.py'),str(cell),'--json'],[sys.executable,str(SCRIPTS/'validate_environment_layout.py'),str(layout),'--json']]:
  vrc,_=_run(cmd)
  if vrc!=0:r['warnings'].append('Validator reported issues: '+' '.join(cmd[1:3]))
 r['validation_status']='PASS' if not r['blockers'] and not r['missing_files'] else 'BLOCKED'; r['ready_for_plan_simulate']=r['validation_status']=='PASS'
 out=json.dumps(r,indent=2)
 if a.json_out: a.json_out.parent.mkdir(parents=True,exist_ok=True); a.json_out.write_text(out+'\n',encoding='utf-8')
 print(out); return 0 if r['ready_for_plan_simulate'] else 1

if __name__=='__main__': raise SystemExit(main())
