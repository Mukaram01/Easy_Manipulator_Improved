#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument('report', type=Path); ap.add_argument('--json', action='store_true'); a=ap.parse_args()
    data=json.loads(a.report.read_text(encoding='utf-8'))
    errs=[]
    if data.get('schema')!='planning_scene_readiness_report/v1': errs.append('schema mismatch')
    checks=(data.get('checks') or {})
    safety=(checks.get('safety') or {})
    required_false=['motion_command_sent','moveit_plan_service_called','runtime_execution_called','real_hardware_enabled','runtime_io_applied']
    for k in required_false:
        if safety.get(k) is not False: errs.append(f'safety.{k} must be false')
    result=data.get('result') or {}
    if result.get('readiness') not in {'PASS','WARN','FAIL'}: errs.append('invalid readiness')
    if not isinstance(result.get('classification'), list): errs.append('classification must be list')
    if result.get('readiness')=='PASS':
        task=checks.get('task') or {}
        if not task.get('offline_plan_preview_request_present'): errs.append('PASS requires offline plan preview request present')
        if task.get('waypoint_count') is None: errs.append('PASS requires waypoint_count')
        scene_objects=checks.get('scene_objects') or {}
        if 'collision_manifest_schema_valid' in scene_objects:
            if scene_objects.get('collision_objects_detected') is not True: errs.append('PASS requires collision objects detected')
            if scene_objects.get('collision_manifest_fresh') is not True: errs.append('PASS requires fresh collision manifest')
            if scene_objects.get('collision_geometry_valid') is not True: errs.append('PASS requires valid collision geometry')
            if not isinstance(scene_objects.get('collision_object_count'), int) or scene_objects.get('collision_object_count') < 1: errs.append('PASS requires collision_object_count >= 1')
    out={'status':'FAIL' if errs else 'PASS','errors':errs}
    print(json.dumps(out, indent=2) if a.json else out['status'])
    return 1 if errs else 0
if __name__=='__main__': raise SystemExit(main())
