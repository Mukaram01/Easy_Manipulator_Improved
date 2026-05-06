#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, sys
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:
    yaml=None

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
from validate_offline_plan_preview_request import validate_request


def _load(path: Path) -> dict[str, Any]:
    if not path or not path.exists():
        return {}
    if yaml is not None:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
        return data if isinstance(data, dict) else {}
    return json.loads(path.read_text(encoding='utf-8'))


def _find_launch(scene_package: Path) -> Path | None:
    for c in [scene_package / 'launch' / 'demo.launch.py', scene_package / 'demo.launch.py']:
        if c.is_file():
            return c
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene-package', required=True)
    ap.add_argument('--plan-preview-request', required=True, type=Path)
    ap.add_argument('--output-dir', required=True, type=Path)
    ap.add_argument('--cell-definition', type=Path)
    ap.add_argument('--task-recipe', type=Path)
    ap.add_argument('--project-dir', type=Path)
    ap.add_argument('--allow-missing-launch', action='store_true')
    ap.add_argument('--package-name', type=str)
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()

    scene_as_path = Path(a.scene_package)
    pkg_name = a.package_name or scene_as_path.name
    launch = _find_launch(scene_as_path) if scene_as_path.exists() else None
    req = _load(a.plan_preview_request)
    req_valid = validate_request(req) if req else {'status': 'FAIL', 'errors': ['missing request']}
    req_ok = req.get('schema') == 'offline_plan_preview_request/v1'
    waypoint_count = len((((req.get('request') or {}).get('waypoints')) or [])) if req_ok else 0
    pick = ((req.get('request') or {}).get('pick') or {}).get('source_id')
    place = ((req.get('request') or {}).get('place') or {}).get('target_id')
    grasp = ((req.get('request') or {}).get('tool') or {}).get('grasp_strategy')

    cmd = f"ros2 launch {pkg_name} demo.launch.py use_fake_hardware:=true launch_rviz:=true"
    blockers, warnings = [], []
    if not scene_as_path.exists() and '/' in a.scene_package:
        blockers.append('scene_package path does not exist')
    if not launch:
        msg = 'demo.launch.py not found'
        if a.allow_missing_launch:
            warnings.append(msg)
        else:
            blockers.append(msg)
    if not req_ok:
        blockers.append('offline_plan_preview_request invalid schema')
    elif req_valid.get('status') == 'FAIL':
        warnings.append('offline_plan_preview_request has validation errors; session remains metadata-only')

    status = 'PASS' if not blockers else ('WARN' if a.allow_missing_launch and blockers == ['demo.launch.py not found'] else 'FAIL')
    report = {
        'schema': 'rviz_moveit_plan_preview_session/v1',
        'source': {
            'scene_package': a.scene_package,
            'offline_plan_preview_request': str(a.plan_preview_request),
            'cell_definition': str(a.cell_definition) if a.cell_definition else '',
            'task_recipe': str(a.task_recipe) if a.task_recipe else '',
        },
        'session': {
            'id': f'plan_preview_session_{pkg_name}',
            'mode': 'fake_hardware_visual_preview',
            'launch_allowed': False,
            'generated_commands_only': True,
            'user_must_run_commands_manually': True,
        },
        'rviz_moveit': {
            'suggested_launch': {'command': cmd, 'requires_sourced_workspace': True},
            'forbidden_defaults': {'use_fake_hardware_false': True, 'real_hardware': True, 'runtime_io': True},
            'expected_inputs': {
                'scene_package_exists': scene_as_path.exists(),
                'demo_launch_exists': bool(launch),
                'offline_plan_preview_request_valid': req_ok,
                'fake_hardware_default': True if launch else 'unknown',
            },
        },
        'plan_preview': {'waypoint_count': waypoint_count, 'pick_source_id': pick, 'place_target_id': place, 'grasp_strategy': grasp, 'warnings': warnings},
        'safety': {'motion_started': False, 'moveit_service_called': False, 'ros_launch_started': False, 'runtime_io_applied': False, 'fake_hardware_required': True},
        'readiness': {'status': status, 'blockers': blockers, 'warnings': warnings},
        'next_manual_steps': ['Review suggested_commands.sh', 'Source workspace manually (if needed)', 'Run ros2 launch command manually only for fake hardware preview.'],
    }

    out = a.output_dir; out.mkdir(parents=True, exist_ok=True)
    js = out / 'rviz_moveit_plan_preview_session.json'; md = out / 'rviz_moveit_plan_preview_session.md'; sh = out / 'suggested_commands.sh'
    js.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    md.write_text("# RViz/MoveIt Plan Preview Session (v1)\n\n"
                  "This is a **session-preparation artifact** only.\n\n"
                  "- Does not start ROS\n- Does not call MoveIt\n- Does not move the robot\n- User must run suggested command manually\n- Real mode remains blocked/guarded\n\n"
                  f"- Readiness: **{status}**\n- Scene package: `{a.scene_package}`\n- Launch file: `{launch or '(missing)'}`\n- Suggested command: `{cmd}`\n- Blockers: {blockers or ['(none)']}\n- Warnings: {warnings or ['(none)']}\n", encoding='utf-8')
    sh.write_text("#!/usr/bin/env bash\n# Fake-hardware-only preview helper. DO NOT use for real hardware.\n# This script is generated and NOT executed automatically.\n# Optionally source your workspace first, e.g.: source install/setup.bash\n"+cmd+"\n", encoding='utf-8')
    sh.chmod(0o755)

    summary = {'status': status, 'json': str(js), 'markdown': str(md), 'suggested_commands': str(sh), 'blockers': blockers, 'warnings': warnings}
    print(json.dumps(summary, indent=2) if a.json else f"{status}: {js}")
    return 1 if status == 'FAIL' else 0


if __name__ == '__main__':
    raise SystemExit(main())
