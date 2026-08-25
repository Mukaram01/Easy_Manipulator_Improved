#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
from workcell_studio_error_messages import get_message
from workcell_studio_layout_source import inspect_saved_layout
STATES=["NO_WORKSPACE","WORKSPACE_READY","CELL_DRAFT_CREATED","LAYOUT_CREATED","LAYOUT_SAVED","TASK_INTENT_CREATED","SCENE_PACKAGE_GENERATED","FILE_OUTPUTS_CHECKED","VALIDATION_READY","VALIDATION_PASSED","VALIDATION_BLOCKED","PLAN_SIMULATE_READY","SIMULATION_RUNNING","SIMULATION_STOPPED"]
def main()->int:
 ap=argparse.ArgumentParser(); ap.add_argument('--scene-dir',required=True,type=Path); ap.add_argument('--scene-name',required=True); ap.add_argument('--json-out',required=True,type=Path); a=ap.parse_args(); s=a.scene_dir
 saved_layout=inspect_saved_layout(s)
 checks={'saved_layout':saved_layout['saved'],'saved_layout_source':saved_layout['source'],'layout/workcell_studio_layout.yaml':(s/'layout'/'workcell_studio_layout.yaml').exists(),'legacy_environment_layout.yaml':(s/'environment_layout.yaml').exists(),'config/workcell_builder_task_intent.yaml':(s/'config'/'workcell_builder_task_intent.yaml').exists(),'package.xml':(s/'package.xml').exists(),'CMakeLists.txt':(s/'CMakeLists.txt').exists(),'launch/demo.launch.py':(s/'launch'/'demo.launch.py').exists(),'file_output_audit.json':(s/'file_output_audit.json').exists(),'smoke/offline_smoke_report.json':(s/'smoke'/'offline_smoke_report.json').exists()}
 completed=['CELL_DRAFT_CREATED']; current='CELL_DRAFT_CREATED'; next_action='Use Recommended Layout / Add to Canvas'; blockers=[]
 if checks['saved_layout']: completed += ['LAYOUT_CREATED','LAYOUT_SAVED']; current='LAYOUT_SAVED'; next_action='Save Layout'
 else: blockers.append(get_message('MISSING_SAVED_LAYOUT',detail=saved_layout.get('blocker')))
 if checks['config/workcell_builder_task_intent.yaml']: completed += ['TASK_INTENT_CREATED']; current='TASK_INTENT_CREATED'; next_action='Generate/Update Task Intent'
 else: blockers.append(get_message('MISSING_TASK_INTENT'))
 if checks['package.xml'] and checks['CMakeLists.txt'] and checks['launch/demo.launch.py']: completed += ['SCENE_PACKAGE_GENERATED']; current='SCENE_PACKAGE_GENERATED'; next_action='Generate Scene Package'
 else: blockers.append(get_message('MISSING_DEMO_LAUNCH',detail='Scene package outputs are incomplete (package.xml/CMakeLists.txt/launch/demo.launch.py missing).'))
 if checks['file_output_audit.json']: completed += ['FILE_OUTPUTS_CHECKED','VALIDATION_READY']; current='VALIDATION_READY'; next_action='Run Offline Validation'
 if checks['smoke/offline_smoke_report.json']: completed += ['VALIDATION_PASSED','PLAN_SIMULATE_READY']; current='PLAN_SIMULATE_READY'; next_action='Open Plan & Simulate'
 pending=[x for x in STATES if x not in completed and x not in ('NO_WORKSPACE','WORKSPACE_READY')]
 report={'scene_name':a.scene_name,'scene_dir':str(s),'current_state':current,'completed_states':completed,'pending_states':pending,'blocked_states':[current] if blockers else [],'next_recommended_action':next_action,'state_conditions':checks,'blockers':blockers,'warnings':[]}
 a.json_out.parent.mkdir(parents=True,exist_ok=True); a.json_out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8'); print(json.dumps(report,indent=2))
 return 0 if not blockers else 1
if __name__=='__main__': raise SystemExit(main())
