#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path

PASS='WORKCELL_GOLDEN_DEMO: PASS'
WARN='WORKCELL_GOLDEN_DEMO: WARN'
FAIL='WORKCELL_GOLDEN_DEMO: FAIL'

def _load_ids(d:Path,key:str)->set[str]:
    out=set()
    for p in sorted(d.glob('*.json')):
        try:
            j=json.loads(p.read_text(encoding='utf-8'))
            if key in j: out.add(str(j[key]))
        except Exception:
            pass
    return out

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument('--scene-dir', required=True); ap.add_argument('--repo-root', default='.')
    a=ap.parse_args(); d=Path(a.scene_dir); repo=Path(a.repo_root).resolve()
    blocks=[]; warns=[]
    required=['environment.yaml','config/task_recipe.yaml','workcell_studio_summary.json','workcell_studio_summary.md','preview/workcell_preview.svg','preview/workcell_preview.html','config/perception_metadata.json','config/compatibility_metadata.json','config/readiness_overlay_metadata.json']
    for r in required:
        if not (d/r).exists(): blocks.append(f'missing required file: {r}')
    env=(d/'environment.yaml').read_text(encoding='utf-8') if (d/'environment.yaml').exists() else ''
    task=(d/'config/task_recipe.yaml').read_text(encoding='utf-8') if (d/'config/task_recipe.yaml').exists() else ''
    summaryj={}
    if (d/'workcell_studio_summary.json').exists():
        try: summaryj=json.loads((d/'workcell_studio_summary.json').read_text(encoding='utf-8'))
        except Exception: blocks.append('summary json invalid')
    for tok in ['schema_version: workcell_scene/v1','schema_version: workcell_task/v1','realsense_d435i','robotiq','fake_hardware_first: true','real_hardware_enabled: false','motion_command_sent: false','runtime_execution_enabled: false','moveit_plan_service_called: false']:
        if tok not in (env+'\n'+task): blocks.append(f'missing token: {tok}')
    catalog_root=repo/'workcell_builder/workcell_builder/config/compatibility_profiles'
    robot_ids=_load_ids(catalog_root/'robots','robot_id'); tool_ids=_load_ids(catalog_root/'tools','tool_id'); cam_ids=_load_ids(repo/'workcell_builder/workcell_builder/config/camera_profiles','camera_id')
    if 'ur5' not in robot_ids: blocks.append('catalog missing ur5 robot_id')
    if not ({'robotiq_2f85','robotiq'} & tool_ids): warns.append('catalog missing robotiq/robotiq_2f85 tool id')
    if 'realsense_d435i' not in cam_ids: blocks.append('catalog missing realsense_d435i camera_id')
    launch_cmd=str(summaryj.get('fake_hardware_launch_command',''))
    if 'use_fake_hardware:=true' not in launch_cmd: blocks.append('missing fake-hardware launch command')
    if 'use_fake_hardware:=false' in launch_cmd: blocks.append('real hardware launch must not be default')
    if summaryj.get('golden_demo') is not True: warns.append('golden_demo flag not true in summary json')
    if blocks: print(FAIL)
    elif warns: print(WARN)
    else: print(PASS)
    for b in blocks: print('BLOCKER:',b)
    for w in warns: print('WARNING:',w)
    return 1 if blocks else 0

if __name__=='__main__': raise SystemExit(main())
