#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
from typing import Any


def _load(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--bridge-payload', type=Path, required=True)
    ap.add_argument('--rviz-session', type=Path)
    ap.add_argument('--scene-package')
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()

    bridge = _load(a.bridge_payload)
    rviz = _load(a.rviz_session) if a.rviz_session else {}
    blockers: list[str] = []
    warnings: list[str] = []

    bridge_status = bridge.get('status')
    if bridge_status not in ('bridge_preview_ready', 'bridge_preview_partial'):
        blockers.append('Bridge payload preview is not ready for handoff')

    bridge_safe = all([
        bridge.get('preview_only') is True,
        bridge.get('dry_run') is True,
        bridge.get('no_robot_motion') is True,
        bridge.get('no_runtime_execution') is True,
        bridge.get('real_hardware_enabled') is False,
        bridge.get('runtime_execution_called') is False,
        bridge.get('motion_command_sent') is False,
        bridge.get('moveit_plan_service_called') is False,
    ])
    if not bridge_safe:
        blockers.append('Bridge payload safety flags are not preview-only/no-motion')

    cmd = None
    bridge_arg_supported = False
    session_cmd = ((((rviz.get('rviz_moveit') or {}).get('suggested_launch')) or {}).get('command') if isinstance(rviz, dict) else None)
    if isinstance(session_cmd, str) and session_cmd.strip():
        cmd = session_cmd.strip() + (
            f" explicit_release_pose_source:=bridge_payload explicit_release_pose_bridge_payload_path:={a.bridge_payload}"
        )
        bridge_arg_supported = True
    else:
        warnings.append('RViz/MoveIt launch metadata missing; command not generated')

    status = 'plan_preview_ready'
    if blockers:
        status = 'plan_preview_blocked'
    elif warnings:
        status = 'plan_preview_partial'

    out = {
        'schema': 'bridge_payload_plan_preview_handoff/v1',
        'status': status,
        'source_bridge_payload_path': str(a.bridge_payload),
        'scene_package': a.scene_package,
        'rviz_moveit_plan_preview_session_path': str(a.rviz_session) if a.rviz_session else None,
        'bridge_payload_launch_argument_supported': bridge_arg_supported,
        'preview_command': cmd,
        'preview_flags': {
            'use_fake_hardware': True,
            'launch_rviz': True,
            'preview_only': True,
            'no_robot_motion': True,
            'no_runtime_execution': True,
        },
        'safety_flags': {
            'fake_hardware_default': True,
            'real_hardware_enabled': False,
            'motion_command_sent': False,
            'runtime_execution_called': False,
            'moveit_plan_service_called': False,
        },
        'selected_object': bridge.get('selected_object', {}),
        'task_bridge': bridge.get('task_bridge', {}),
        'warnings': warnings,
        'blockers': blockers,
    }
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(out, indent=2) + '\n', encoding='utf-8')
    if a.json:
        print(json.dumps({'status': status, 'output': str(a.output), 'command': cmd, 'warnings': warnings, 'blockers': blockers}, indent=2))
    return 1 if status == 'plan_preview_blocked' else 0


if __name__ == '__main__':
    raise SystemExit(main())
