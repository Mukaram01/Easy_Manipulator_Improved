#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from capability_registry import load_structured_data
from validate_builder_task_intent import validate as validate_intent

try:
    import yaml
except Exception:
    yaml = None


def _load(path: Path) -> dict[str, Any]:
    if yaml is not None:
        d = yaml.safe_load(path.read_text(encoding='utf-8'))
        return d if isinstance(d, dict) else {}
    d, _ = load_structured_data(path)
    return d if isinstance(d, dict) else {}


def _dump(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if yaml is None:
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding='utf-8')
    else:
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')


def convert(task_intent_path: Path, scene_package: Path | None = None) -> tuple[dict[str, Any], list[str]]:
    payload = _load(task_intent_path)
    warnings: list[str] = []
    task = payload.get('task') if isinstance(payload.get('task'), dict) else {}
    pick = (payload.get('pick') or {}).get('source') if isinstance((payload.get('pick') or {}).get('source'), dict) else {}
    pick_filter = (payload.get('pick') or {}).get('object_filter') if isinstance((payload.get('pick') or {}).get('object_filter'), dict) else {}
    place = (payload.get('place') or {}).get('target') if isinstance((payload.get('place') or {}).get('target'), dict) else {}
    grasp = payload.get('grasp') if isinstance(payload.get('grasp'), dict) else {}
    place_block = payload.get('place') if isinstance(payload.get('place'), dict) else {}
    routing = payload.get('routing') if isinstance(payload.get('routing'), dict) else {}
    safety = payload.get('safety') if isinstance(payload.get('safety'), dict) else {}

    recipe = {
        'schema_version': 'task_recipe/v1',
        'task': {
            'id': task.get('id') or 'builder_generated_task',
            'type': task.get('type') or 'pick_place',
            'source_object': pick.get('id') or '',
            'object_filter': {
                'class_id': pick_filter.get('class_id'),
                'color': pick_filter.get('color'),
            },
            'destinations': [{'id': place.get('id') or '', 'frame': 'world', 'pose_xyz': place_block.get('offset_xyz') or [0.4, 0.0, 0.2], 'pose_rpy': [0.0, 0.0, 0.0]}],
            'rules': routing.get('rules') if isinstance(routing.get('rules'), list) else [],
        },
        'grasp': {'strategy_ref': grasp.get('strategy_ref')},
        'builder_task_intent': {
            'source_file': str(task_intent_path),
            'pick': payload.get('pick') or {},
            'grasp': payload.get('grasp') or {},
            'place': payload.get('place') or {},
            'routing': routing,
            'release_strategy': place_block.get('release_strategy'),
            'safety': {
                'metadata_only': bool(safety.get('metadata_only', True)),
                'runtime_io_applied': bool(safety.get('runtime_io_applied', False)),
                'motion_started': bool(safety.get('motion_started', False)),
                'ros_launch_started': bool(safety.get('ros_launch_started', False)),
            }
        }
    }
    if not recipe['task']['rules']:
        recipe['task']['rules'] = [{'id': 'default_route', 'when': {'always': True}, 'destination': place.get('id') or ''}]
        warnings.append('Routing rules missing; generated default always-true rule.')
    return recipe, warnings


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--task-intent', required=True, type=Path)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--validate', action='store_true')
    ap.add_argument('--scene-package', type=Path)
    ap.add_argument('--cell-definition', type=Path)
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args()

    if args.validate:
        report = validate_intent(args.task_intent, args.scene_package)
        if report.get('status') == 'FAIL':
            if args.json:
                print(json.dumps({'status': 'FAIL', 'errors': report.get('errors', [])}, indent=2))
            else:
                for e in report.get('errors', []):
                    print(f'FAIL: {e}')
            return 1

    recipe, warns = convert(args.task_intent, args.scene_package)
    _dump(args.output, recipe)
    summary = {
        'status': 'PASS',
        'generated_task_recipe_path': str(args.output),
        'pick_source': ((recipe.get('builder_task_intent') or {}).get('pick') or {}).get('source', {}).get('id'),
        'place_target': ((recipe.get('builder_task_intent') or {}).get('place') or {}).get('target', {}).get('id'),
        'grasp_strategy': (recipe.get('grasp') or {}).get('strategy_ref'),
        'release_strategy': (recipe.get('builder_task_intent') or {}).get('release_strategy'),
        'routing_rule_count': len(((recipe.get('task') or {}).get('rules') or [])),
        'warnings': warns,
    }
    if args.json:
        print(json.dumps(summary, indent=2))
    else:
        print(f"Wrote: {args.output}")
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
