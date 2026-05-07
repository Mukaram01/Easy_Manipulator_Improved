#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('payload', type=Path)
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    data = json.loads(a.payload.read_text(encoding='utf-8')) if a.payload.exists() else {}
    errors = []
    warnings = []
    for key in ['source', 'selected_object', 'task_bridge', 'safety_flags', 'status']:
        if key not in data:
            errors.append(f'missing field: {key}')
    if data.get('status') not in ('bridge_preview_ready', 'bridge_preview_partial', 'bridge_preview_blocked'):
        errors.append('invalid status')
    sf = data.get('safety_flags', {}) if isinstance(data.get('safety_flags'), dict) else {}
    for k in ['dry_run', 'preview_only', 'no_robot_motion', 'no_runtime_execution']:
        if sf.get(k) is not True:
            errors.append(f'safety_flags.{k} must be true')
    for k in ['moveit_plan_service_called', 'real_hardware_enabled']:
        if sf.get(k) is not False:
            errors.append(f'safety_flags.{k} must be false')
    tb = data.get('task_bridge', {}) if isinstance(data.get('task_bridge'), dict) else {}
    if not tb.get('release_strategy'):
        warnings.append('release strategy missing')
    if not tb.get('place_target'):
        warnings.append('place target missing')
    status = 'FAIL' if errors else ('WARN' if warnings else 'PASS')
    out = {'status': status, 'errors': errors, 'warnings': warnings}
    if a.json:
        print(json.dumps(out, indent=2))
    return 1 if errors else 0

if __name__ == '__main__':
    raise SystemExit(main())
