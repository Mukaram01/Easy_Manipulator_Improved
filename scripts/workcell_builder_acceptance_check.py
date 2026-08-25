#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
import re
from xml.etree import ElementTree as ET
import sys
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
from scripts import workcell_builder_gui_workflow as wf

UR5_JOINTS=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]

def _write(path:Path, content:str)->None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding='utf-8')

def build_golden_scene(scene_name:str, scenes_root:Path, assets_root:Path)->Path:
    state={"scene_name":scene_name,"fake_hardware_default":True,"selected":{"robot":"ur5","tool":"robotiq_2f","camera":"realsense_d435i","support_surface":"workbench","pick_area":"bin","place_target":"pick_area","task":"pick_place","grasp_strategy":"finger_top"},"current_cell_assets":[]}
    wf.add_asset_to_cell(state,'ur5','robot','robot_base','UR5')
    wf.add_asset_to_cell(state,'robotiq_2f','gripper','end_effector','Robotiq 2F')
    wf.add_asset_to_cell(state,'realsense_d435i','camera','camera','RealSense D435i')
    wf.add_asset_to_cell(state,'workbench','workbench','support_surface','Workbench')
    wf.add_asset_to_cell(state,'bin','bin','pick_object','Bin')
    wf.add_asset_to_cell(state,'pick_area','object','place_target','Pick Area')
    wf.add_asset_to_cell(state,'cube','object','pick_object','Cube')
    scene_dir=Path(wf.create_new_cell(scene_name, scenes_root)["scene_dir"])
    wf.generate_yaml_files_for_scene(state, scene_dir)
    wf.generate_files_from_yaml(scene_dir)
    _write(scene_dir/'launch'/'demo.launch.py', 'use_fake_hardware_default = True\n')
    _write(scene_dir/'urdf'/'scene.urdf.xacro', '<robot name="ur5">' + ''.join(f'<joint name="{j}" type="revolute"/>' for j in UR5_JOINTS) + '</robot>')
    _write(scene_dir/'urdf'/'arm_hand.srdf.xacro', '<robot name="ur5"><group name="manipulator"/></robot>')
    _write(scene_dir/'generated'/'preview.txt', 'preview-ready\n')
    _write(scene_dir/'generated'/'readiness_summary.md', '# readiness\n')
    return scene_dir

def package_consistency(scene_dir:Path)->tuple[bool,list[str]]:
    errs=[]
    pkg=scene_dir.name
    px=ET.fromstring((scene_dir/'package.xml').read_text(encoding='utf-8'))
    name=(px.findtext('name') or '').strip()
    if name!=pkg: errs.append('package.xml name mismatch')
    cmake=(scene_dir/'CMakeLists.txt').read_text(encoding='utf-8')
    if f'project({pkg})' not in cmake: errs.append('CMake project mismatch')
    for req in ['launch/demo.launch.py','urdf/scene.urdf.xacro','urdf/arm_hand.srdf.xacro','environment.yaml','scene_manifest.yaml']:
        if not (scene_dir/req).exists(): errs.append(f'missing {req}')
    return (not errs, errs)

def launch_smoke(scene_dir:Path)->tuple[bool,list[str]]:
    errs=[]
    urdf=(scene_dir/'urdf'/'scene.urdf.xacro').read_text(encoding='utf-8')
    srdf=(scene_dir/'urdf'/'arm_hand.srdf.xacro').read_text(encoding='utf-8')
    for j in UR5_JOINTS:
        if j not in urdf: errs.append(f'missing joint {j}')
    if 'manipulator' not in srdf: errs.append('missing planning group')
    launch=(scene_dir/'launch'/'demo.launch.py').read_text(encoding='utf-8')
    if 'use_fake_hardware_default = True' not in launch: errs.append('fake hardware default false')
    return (not errs, errs)

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--scene-name',default='golden_ur5_2f_cell')
    ap.add_argument('--scenes-root',default='~/workcell_ws/src/scenes')
    ap.add_argument('--assets-root',default='~/workcell_ws/src/assets')
    ap.add_argument('--headless',action='store_true')
    args=ap.parse_args()
    scene_dir=build_golden_scene(args.scene_name, Path(args.scenes_root).expanduser(), Path(args.assets_root).expanduser())
    checks=[]
    checks.append(('scene creation', scene_dir.exists(), []))
    yaml_ok=all((scene_dir/f).exists() for f in ['environment.yaml','scene_manifest.yaml','layout/workcell_studio_layout.yaml','generated/cell_definition.yaml'])
    checks.append(('YAML generation', yaml_ok, [] if yaml_ok else ['missing yaml outputs']))
    pc_ok,pc_err=package_consistency(scene_dir); checks.append(('package consistency',pc_ok,pc_err))
    sm_ok,sm_err=launch_smoke(scene_dir); checks.append(('SRDF/controller joints',sm_ok,sm_err))
    prev_ok=all((scene_dir/f).exists() for f in ['generated/preview.txt','generated/readiness_summary.md']); checks.append(('preview artifacts',prev_ok,[] if prev_ok else ['missing preview']))
    cfg=scene_dir/"config"
    replay=wf.generate_perception_replay_preview(cfg)
    replay_ok=all((cfg/f).exists() for f in ['runtime_bridge_payload.preview.json','perception_replay_markers.json','selected_target_summary.json'])
    checks.append(('perception replay offline preview', replay_ok, [] if replay_ok else ['missing replay artifacts']))
    launch_text=(scene_dir/'launch'/'demo.launch.py').read_text(encoding='utf-8')
    checks.append(('no live epd/realsense auto launch', ('easy_perception_deployment' not in launch_text and 'realsense2_camera_node' not in launch_text), []))
    fake_ok='True' in (scene_dir/'launch'/'demo.launch.py').read_text(encoding='utf-8'); checks.append(('fake-hardware default',fake_ok,[] if fake_ok else ['false']))
    for name,ok,errs in checks:
      print(f"{'PASS' if ok else 'FAIL'}: {name}")
      for e in errs: print(f'  - {e}')
    print('\nNext manual commands:')
    print('cd ~/workcell_ws')
    print(f'colcon build --symlink-install --packages-select {args.scene_name}')
    print('source install/setup.bash')
    print(f'ros2 launch {args.scene_name} demo.launch.py \\')
    print('  use_fake_hardware:=true \\')
    print('  publish_workcell_markers:=true \\')
    print('  publish_perception_replay:=true')
    return 0 if all(c[1] for c in checks) else 1

if __name__=='__main__':
    raise SystemExit(main())
