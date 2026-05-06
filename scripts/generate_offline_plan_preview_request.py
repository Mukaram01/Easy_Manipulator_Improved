#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, sys
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:
    yaml = None

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
from capability_registry import load_structured_data


def _load(path: Path) -> dict[str, Any]:
    if not path or not path.exists():
        return {}
    if yaml is not None:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
        return data if isinstance(data, dict) else {}
    data, _ = load_structured_data(path)
    return data if isinstance(data, dict) else {}


def _dump(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if yaml is None:
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding='utf-8')
    else:
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')


def generate(task_recipe: dict[str, Any], task_recipe_path: Path, task_flow_summary: dict[str, Any], cell: dict[str, Any], env: dict[str, Any]) -> tuple[dict[str, Any], list[str], list[str]]:
    warnings, missing = [], []
    task = task_recipe.get('task') if isinstance(task_recipe.get('task'), dict) else {}
    pick_source = task.get('source_object') or task.get('pick_source') or task.get('source') or ''
    dests = task.get('destinations') if isinstance(task.get('destinations'), list) else []
    place_target = dests[0].get('id') if dests and isinstance(dests[0], dict) else ''
    grasp_block = task_recipe.get('grasp') if isinstance(task_recipe.get('grasp'), dict) else {}
    grasp = grasp_block.get('strategy_ref')
    if not pick_source: missing.append('pick.source_id')
    if not place_target: missing.append('place.target_id')
    if not grasp: missing.append('tool.grasp_strategy')
    robot = (cell.get('robot') if isinstance(cell.get('robot'), dict) else {})
    tool = (cell.get('end_effector') if isinstance(cell.get('end_effector'), dict) else {})
    task_id = task.get('id') or 'task'
    req = {
        'schema': 'offline_plan_preview_request/v1',
        'source': {
            'task_recipe': str(task_recipe_path),
            'task_flow_summary': str(task_flow_summary.get('source') or 'generated')
        },
        'request': {
            'id': f'plan_preview_{task_id}',
            'mode': 'offline_preview',
            'robot': {'id': robot.get('capability') or robot.get('name') or None, 'planning_group': robot.get('planning_group')},
            'tool': {'id': tool.get('capability') or tool.get('id') or None, 'grasp_strategy': grasp},
            'pick': {'source_id': pick_source, 'object_filter': task.get('object_filter') or {}, 'approach_axis': 'z_down', 'approach_distance_m': 0.12, 'retreat_axis': 'z_up', 'retreat_distance_m': 0.10},
            'place': {'target_id': place_target, 'place_offset_xyz': [0,0,0.08], 'release_strategy': 'tool_release', 'retreat_axis': 'z_up', 'retreat_distance_m': 0.12},
            'waypoints': [
                {'id':'home','type':'named_pose','required':False},
                {'id':'pre_pick','type':'computed_offset','from':'pick.source_id','offset_axis':'approach_axis','distance_m':0.12},
                {'id':'pick','type':'target_pose','from':'pick.source_id'},
                {'id':'post_pick','type':'computed_offset','from':'pick.source_id','offset_axis':'retreat_axis','distance_m':0.10},
                {'id':'pre_place','type':'computed_offset','from':'place.target_id','offset_axis':'approach_axis','distance_m':0.12},
                {'id':'place','type':'target_pose','from':'place.target_id'},
                {'id':'post_place','type':'computed_offset','from':'place.target_id','offset_axis':'retreat_axis','distance_m':0.12},
            ],
            'checks': {
                'require_pick_source': True, 'require_place_target': True, 'require_grasp_strategy': True,
                'require_collision_scene': True, 'require_fake_hardware_default': True
            }
        },
        'safety': {'metadata_only': True, 'motion_started': False, 'ros_launch_started': False, 'moveit_service_called': False, 'runtime_io_applied': False}
    }
    return req, warnings, missing


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--task-recipe', required=True, type=Path)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--cell-definition', type=Path)
    ap.add_argument('--environment-layout', type=Path)
    ap.add_argument('--task-flow-summary', type=Path)
    ap.add_argument('--validate', action='store_true')
    ap.add_argument('--allow-incomplete', action='store_true')
    a = ap.parse_args()
    recipe = _load(a.task_recipe)
    flow = _load(a.task_flow_summary) if a.task_flow_summary else {}
    cell = _load(a.cell_definition) if a.cell_definition else {}
    env = _load(a.environment_layout) if a.environment_layout else {}
    payload, warns, missing = generate(recipe, a.task_recipe, flow, cell, env)
    status = 'PASS'
    readiness = 'plan_preview_request_generated'
    errors = []
    if missing:
        if a.allow_incomplete:
            status = 'WARN'; readiness = 'plan_preview_request_incomplete'; warns.append('Missing required fields: ' + ', '.join(missing))
        else:
            status = 'FAIL'; errors.append('Missing required fields: ' + ', '.join(missing))
    if status != 'FAIL':
        _dump(a.output, payload)
    summary = {'status': status, 'output_path': str(a.output), 'task_id': (recipe.get('task') or {}).get('id'), 'pick_source_id': payload['request']['pick']['source_id'], 'place_target_id': payload['request']['place']['target_id'], 'grasp_strategy': payload['request']['tool']['grasp_strategy'], 'waypoint_count': len(payload['request']['waypoints']), 'missing_required_fields': missing, 'warnings': warns, 'errors': errors, 'readiness_classification': readiness, 'safety': payload['safety']}
    if a.validate and status != 'FAIL':
        from validate_offline_plan_preview_request import validate_request
        v = validate_request(payload)
        if v['status'] == 'FAIL':
            summary['status'] = 'FAIL'; summary['errors'].extend(v['errors'])
    if a.json:
        print(json.dumps(summary, indent=2))
    else:
        print(f"{summary['status']}: {a.output}")
    return 1 if summary['status'] == 'FAIL' else 0

if __name__ == '__main__':
    raise SystemExit(main())
