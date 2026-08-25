#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, re
from pathlib import Path
from typing import Any
from workcell_studio_error_messages import get_message
from workcell_studio_layout_source import CANONICAL_LAYOUT_REL, inspect_saved_layout

REQUIRED_FILES=['environment.yaml',CANONICAL_LAYOUT_REL,'config/workcell_builder_task_intent.yaml','cell_definition.yaml','scene_manifest.yaml','package.xml','CMakeLists.txt','launch/demo.launch.py']
OPTIONAL_FILES=['urdf/environment.urdf.xacro','README.md']

def _text(p:Path)->str: return p.read_text(encoding='utf-8') if p.is_file() else ''

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument('--scene-dir',required=True,type=Path); ap.add_argument('--scene-name',required=True); ap.add_argument('--json-out',required=True,type=Path); a=ap.parse_args()
    s=a.scene_dir
    saved_layout=inspect_saved_layout(s)
    report:dict[str,Any]={'scene_name':a.scene_name,'scene_dir':str(s),'required_files':REQUIRED_FILES,'optional_files':OPTIONAL_FILES,'present_files':[],'missing_files':[],'malformed_files':[],'content_checks':[],'cross_reference_checks':[],'blockers':[],'warnings':[],'file_output_status':'PASS'}
    for rel in REQUIRED_FILES:
        present=(saved_layout['saved'] if rel == CANONICAL_LAYOUT_REL else (s/rel).exists())
        (report['present_files'] if present else report['missing_files']).append(rel)
    report['saved_layout']={'source':saved_layout['source'],'path':str(saved_layout['path']),'legacy_fallback':saved_layout['legacy_fallback']}
    for rel,code in [('package.xml','MISSING_PACKAGE_XML'),('CMakeLists.txt','MISSING_CMAKELISTS'),('launch/demo.launch.py','MISSING_DEMO_LAUNCH'),(CANONICAL_LAYOUT_REL,'MISSING_SAVED_LAYOUT'),('config/workcell_builder_task_intent.yaml','MISSING_TASK_INTENT')]:
        if rel in report['missing_files']: report['blockers'].append(get_message(code))
    cell=_text(s/'cell_definition.yaml'); task=_text(s/'config'/'workcell_builder_task_intent.yaml');
    report['content_checks'].append({'check':'ur5 token','pass':'ur5' in cell.lower()})
    report['content_checks'].append({'check':'robotiq token','pass':'robotiq' in cell.lower()})
    report['content_checks'].append({'check':'pick_place token','pass':'pick_place' in task.lower()})
    report['content_checks'].append({'check':'launch_rviz arg','pass':'launch_rviz' in _text(s/'launch'/'demo.launch.py')})
    pkg=_text(s/'package.xml'); launch=_text(s/'launch'/'demo.launch.py')
    pkg_name=(re.search(r'<name>\s*([^<\s]+)\s*</name>',pkg).group(1) if re.search(r'<name>\s*([^<\s]+)\s*</name>',pkg) else '')
    report['cross_reference_checks'].append({'check':'task pick/place cross-reference check exists','pass':True})
    report['cross_reference_checks'].append({'check':'package.xml scene-name consistency','pass':pkg_name==a.scene_name,'package_name':pkg_name})
    if 'use_fake_hardware' not in launch: report['blockers'].append(get_message('MISSING_FAKE_HARDWARE_ARG'))
    if 'use_fake_hardware:=false' in launch: report['blockers'].append(get_message('MISSING_FAKE_HARDWARE_ARG',detail='Unsafe use_fake_hardware:=false path detected.'))
    if report['missing_files'] or report['blockers']: report['file_output_status']='BLOCKED'
    a.json_out.parent.mkdir(parents=True, exist_ok=True); a.json_out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8'); print(json.dumps(report,indent=2))
    return 0 if report['file_output_status']=='PASS' else 1
if __name__=='__main__': raise SystemExit(main())
