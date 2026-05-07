#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('handoff', type=Path)
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    data = json.loads(a.handoff.read_text(encoding='utf-8')) if a.handoff.exists() else {}
    errors = []
    warnings = []

    if data.get('schema') != 'bridge_payload_plan_preview_handoff/v1':
        errors.append('invalid schema')
    status = data.get('status')
    if status not in ('plan_preview_ready', 'plan_preview_partial', 'plan_preview_blocked'):
        errors.append('invalid status')

    src = Path(data.get('source_bridge_payload_path', '')) if data.get('source_bridge_payload_path') else None
    if not src or not src.exists():
        errors.append('referenced bridge payload does not exist')

    sf = data.get('safety_flags', {}) if isinstance(data.get('safety_flags'), dict) else {}
    for key in ('real_hardware_enabled', 'motion_command_sent', 'runtime_execution_called', 'moveit_plan_service_called'):
        if sf.get(key) is not False:
            errors.append(f'safety_flags.{key} must be false')
    if sf.get('fake_hardware_default') is not True:
        errors.append('safety_flags.fake_hardware_default must be true')

    pf = data.get('preview_flags', {}) if isinstance(data.get('preview_flags'), dict) else {}
    for key in ('preview_only', 'no_robot_motion', 'no_runtime_execution', 'use_fake_hardware', 'launch_rviz'):
        if pf.get(key) is not True:
            errors.append(f'preview_flags.{key} must be true')

    cmd = data.get('preview_command')
    if cmd:
        if 'use_fake_hardware:=true' not in cmd:
            errors.append('preview command must include use_fake_hardware:=true')
        for forbidden in ('use_fake_hardware:=false', 'real_hardware:=true', '--execute'):
            if forbidden in cmd:
                errors.append(f'preview command must not contain {forbidden}')
    elif status == 'plan_preview_ready':
        warnings.append('ready status without preview command')

    out = {'status': 'FAIL' if errors else ('WARN' if warnings else 'PASS'), 'errors': errors, 'warnings': warnings}
    if a.json:
        print(json.dumps(out, indent=2))
    return 1 if errors else 0

if __name__ == '__main__':
    raise SystemExit(main())
