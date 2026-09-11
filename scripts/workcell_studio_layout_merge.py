#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
import yaml
import copy
from authored_yaml import write_preserving

def _load(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    return data if isinstance(data, dict) else {}

def _save_yaml(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')

def _index(items: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    out = {}
    for it in items:
        if isinstance(it, dict) and it.get('id'):
            out[str(it['id'])] = dict(it)
    return out

def merge(scene_dir: Path, deleted_item_ids: list[str] | None = None, *, save_authored: bool = False) -> dict[str, Any]:
    if not scene_dir.exists() or not scene_dir.is_dir():
        return {
            'status': 'BLOCKED',
            'layout_applied': False,
            'generated_from_saved_layout': False,
            'merge_warnings': [],
            'merge_blockers': [f"scene_dir not found: {scene_dir}"],
            'layout_saved_at_utc': None,
            'merged_at_utc': datetime.now(timezone.utc).isoformat(),
            'safety_flags': {'fake_hardware_first': True, 'runtime_execution_enabled': False, 'motion_command_sent': False},
        }
    env = _load(scene_dir/'environment.yaml')
    manifest = _load(scene_dir/'scene_manifest.yaml')
    layout = _load(scene_dir/'layout'/'workcell_studio_layout.yaml')
    intent = _load(scene_dir/'config'/'workcell_builder_task_intent.yaml')
    recipe = _load(scene_dir/'config'/'task_recipe.yaml')
    # Native Save owns structural edits. Consume its tombstones before clearing
    # them: an additive cache merge cannot delete canonical environment records.
    deleted = set(deleted_item_ids or [])
    def without_deleted(value):
        if isinstance(value, list):
            return [without_deleted(item) for item in value
                    if not (isinstance(item, dict) and
                            (str(item.get('id', '')) in deleted or
                             str(item.get('layout_item_ref', '')) in deleted))]
        if isinstance(value, dict):
            return {key: without_deleted(item) for key, item in value.items()}
        return value
    if deleted:
        if any(str(item.get('id', '')) in deleted for item in layout.get('items', []) if isinstance(item, dict)):
            raise ValueError('Deleted IDs still exist in saved layout; refusing environment projection')
        env = without_deleted(env)
        manifest = without_deleted(manifest)
        write_preserving(scene_dir/'environment.yaml', env)
        _save_yaml(scene_dir/'scene_manifest.yaml', manifest)
    generated = scene_dir/'generated'; generated.mkdir(exist_ok=True)

    warnings: list[str] = []
    blockers: list[str] = []
    layout_items = layout.get('items') if isinstance(layout.get('items'), list) else []
    if save_authored:
        physical = env.setdefault('environment', {})
        for item in layout_items:
            if not isinstance(item, dict) or item.get('locked') is True or item.get('editable') is False:
                continue
            iid = item.get('id')
            if not iid:
                raise ValueError('Physical authored item requires a stable ID')
            collection = ('task_zones' if item.get('category') == 'zone' else
                          'support_surfaces' if item.get('role') == 'support_surface' else 'assets')
            target = None
            for key in ('support_surfaces', 'assets', 'sensors', 'task_zones'):
                for record in physical.get(key, []):
                    if record.get('id') == iid:
                        target = record
            if target is None:
                target = {'id': iid}
                physical.setdefault(collection, []).append(target)
            for key in ('type', 'role', 'display_name', 'category', 'frame', 'geometry_type',
                        'mesh', 'collision', 'dimensions', 'asset_class', 'description',
                        'catalog_asset_id', 'support_surface_ref', 'task_zone_ref'):
                if key in item:
                    target[key] = copy.deepcopy(item[key])
            if item.get('mesh') and 'collision' not in target:
                target['collision'] = {'enabled': True, 'mode': 'mesh'}
            target['layout_item_ref'] = iid
            pose = item.get('pose', {})
            for key in ('xyz', 'rpy'):
                if key in pose:
                    target['pose_' + key] = copy.deepcopy(pose[key])
            # Existing top-level compatibility mirrors must not retain stale
            # physical values. Never create additional mirror records.
            for key in ('support_surfaces', 'assets', 'sensors', 'placed_objects', 'objects', 'task_zones'):
                records = env.get(key, [])
                if isinstance(records, dict):
                    records = records.values()
                for mirror in records:
                    if isinstance(mirror, dict) and mirror.get('id') == iid and mirror is not target:
                        mirror.update(copy.deepcopy(target))
        write_preserving(scene_dir/'environment.yaml', env)
    # Generated physical state is a projection of environment.yaml only.
    physical = env.get('environment', {})
    merged = _index(env.get('objects', []) if isinstance(env.get('objects'), list) else [])
    for collection in ('support_surfaces', 'assets', 'sensors', 'task_zones'):
        for item in physical.get(collection, []):
            record = copy.deepcopy(item)
            record['pose'] = {'xyz': record.get('pose_xyz', [0, 0, 0]),
                              'rpy': record.get('pose_rpy', [0, 0, 0])}
            merged[str(record['id'])] = record

    # propagate task bindings
    bindings = layout.get('task_bindings') if isinstance(layout.get('task_bindings'), dict) else {}
    if bindings:
        intent.setdefault('pick', {}).setdefault('source', {})['id'] = bindings.get('pick_source', intent.get('pick', {}).get('source', {}).get('id'))
        intent.setdefault('place', {}).setdefault('target', {})['id'] = bindings.get('place_target', intent.get('place', {}).get('target', {}).get('id'))
        if bindings.get('camera'):
            intent['camera'] = {'id': bindings['camera']}
        recipe.setdefault('builder_task_intent', {})
        recipe['builder_task_intent']['pick'] = intent.get('pick', {})
        recipe['builder_task_intent']['place'] = intent.get('place', {})

    safety = intent.setdefault('safety', {})
    safety['fake_hardware_first'] = True
    safety['runtime_execution_enabled'] = False
    safety['motion_command_sent'] = False

    merged_env = dict(env)
    merged_env['objects'] = list(merged.values())
    merged_manifest = dict(manifest)
    merged_manifest['objects'] = list(merged.values())
    merged_manifest['generated_from_saved_layout'] = bool(layout_items)

    _save_yaml(generated/'workcell_studio_merged_environment.yaml', merged_env)
    _save_yaml(generated/'workcell_studio_merged_scene_manifest.yaml', merged_manifest)
    _save_yaml(generated/'workcell_builder_task_intent.yaml', intent)
    _save_yaml(generated/'task_recipe.yaml', recipe)
    now = datetime.now(timezone.utc).isoformat()
    report = {
        'layout_applied': bool(layout_items),
        'generated_from_saved_layout': bool(layout_items),
        'merge_warnings': warnings,
        'merge_blockers': blockers,
        'layout_saved_at_utc': str(layout.get('saved_at_utc')) if layout.get('saved_at_utc') is not None else None,
        'merged_at_utc': now,
        'safety_flags': {'fake_hardware_first': True, 'runtime_execution_enabled': False, 'motion_command_sent': False},
    }
    (generated/'workcell_studio_layout_merge_report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    (generated/'workcell_studio_layout_merge_summary.txt').write_text(f"layout_applied={report['layout_applied']}\ngenerated_from_saved_layout={report['generated_from_saved_layout']}\n", encoding='utf-8')
    report['status'] = 'READY' if not blockers else 'BLOCKED'
    return report

if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('scene_dir', type=Path); ap.add_argument('--json', action='store_true')
    ap.add_argument('--deleted-item-id', action='append', default=[])
    ap.add_argument('--save-authored', action='store_true')
    a = ap.parse_args(); rep = merge(a.scene_dir, a.deleted_item_id, save_authored=a.save_authored)
    if a.json: print(json.dumps(rep, indent=2))
    raise SystemExit(0 if rep.get('status') == 'READY' else 2)
