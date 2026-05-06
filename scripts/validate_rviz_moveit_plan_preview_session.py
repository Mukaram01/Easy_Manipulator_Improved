#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument('session',type=Path); ap.add_argument('--json',action='store_true'); a=ap.parse_args()
    data=json.loads(a.session.read_text(encoding='utf-8'))
    errs=[]; warns=[]
    if data.get('schema')!='rviz_moveit_plan_preview_session/v1': errs.append('schema mismatch')
    s=data.get('safety',{})
    checks={'motion_started':False,'moveit_service_called':False,'ros_launch_started':False,'runtime_io_applied':False,'fake_hardware_required':True}
    for k,v in checks.items():
        if s.get(k)!=v: errs.append(f'safety.{k} must be {v}')
    sess=data.get('session',{})
    if sess.get('launch_allowed') is not False: errs.append('session.launch_allowed must be false')
    if sess.get('generated_commands_only') is not True: errs.append('session.generated_commands_only must be true')
    cmd=((data.get('rviz_moveit') or {}).get('suggested_launch') or {}).get('command','')
    bad=['use_fake_hardware:=false','real_hardware','runtime execution','execute runtime']
    for b in bad:
        if b in cmd: errs.append(f'forbidden command token: {b}')
    src=data.get('source',{})
    req_path=Path(src.get('offline_plan_preview_request','')) if src.get('offline_plan_preview_request') else None
    if req_path and not req_path.exists(): warns.append('offline plan preview request path missing')
    req_valid=((data.get('rviz_moveit') or {}).get('expected_inputs') or {}).get('offline_plan_preview_request_valid')
    if req_valid and (data.get('plan_preview') or {}).get('waypoint_count') is None: errs.append('waypoint_count required when request valid')
    status='FAIL' if errs else ('WARN' if warns else 'PASS')
    out={'status':status,'errors':errs,'warnings':warns}
    print(json.dumps(out,indent=2) if a.json else status)
    return 1 if status=='FAIL' else 0
if __name__=='__main__': raise SystemExit(main())
