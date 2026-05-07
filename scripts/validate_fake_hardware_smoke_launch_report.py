#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path

def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument('report',type=Path); ap.add_argument('--json',action='store_true'); a=ap.parse_args()
    data=json.loads(a.report.read_text(encoding='utf-8'))
    errs=[]; warns=[]
    if data.get('schema')!='fake_hardware_smoke_launch_report/v1': errs.append('schema mismatch')
    s=data.get('safety',{})
    for k in ['motion_command_sent','moveit_plan_service_called','runtime_execution_called','real_hardware_enabled','runtime_io_applied']:
        if s.get(k) is not False: errs.append(f'safety.{k} must be false')
    run=data.get('run',{}); checks=data.get('checks',{})
    if run.get('actually_launched'):
        if checks.get('command_safety_checked') is not True: errs.append('checks.command_safety_checked must be true when launched')
        if checks.get('forbidden_real_hardware_tokens_absent') is not True: errs.append('checks.forbidden_real_hardware_tokens_absent must be true when launched')
        if checks.get('launch_started') is not True: warns.append('launch did not appear to start')
        art=data.get('artifacts',{})
        for k in ['captured_stdout_log','captured_stderr_log']:
            p=Path(art.get(k,'')) if art.get(k) else None
            if not p or not p.exists(): errs.append(f'missing artifact path: {k}')
    status='FAIL' if errs else ('WARN' if warns else 'PASS')
    out={'status':status,'errors':errs,'warnings':warns}
    print(json.dumps(out,indent=2) if a.json else status)
    return 1 if status=='FAIL' else 0
if __name__=='__main__': raise SystemExit(main())
