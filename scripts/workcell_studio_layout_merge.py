#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
import yaml

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

def merge(scene_dir: Path) -> dict[str, Any]:
    env = _load(scene_dir/'environment.yaml')
    manifest = _load(scene_dir/'scene_manifest.yaml')
    layout = _load(scene_dir/'layout'/'workcell_studio_layout.yaml')
    intent = _load(scene_dir/'config'/'workcell_builder_task_intent.yaml')
    recipe = _load(scene_dir/'config'/'task_recipe.yaml')
    generated = scene_dir/'generated'; generated.mkdir(exist_ok=True)

    warnings: list[str] = []
    blockers: list[str] = []
    layout_items = layout.get('items') if isinstance(layout.get('items'), list) else []
    env_items = env.get('objects') if isinstance(env.get('objects'), list) else []
    merged = _index(env_items)
    for mid, m in _index(manifest.get('objects') if isinstance(manifest.get('objects'), list) else []).items():
        merged[mid] = {**merged.get(mid, {}), **m}

    for item in layout_items:
        if not isinstance(item, dict):
            continue
        iid = str(item.get('id') or f"layout_{len(merged)+1}")
        base = merged.get(iid, {})
        merged[iid] = {**base, **item, 'pose': item.get('pose', base.get('pose')), 'size': item.get('size', base.get('size'))}
        if not (item.get('mesh_path') or item.get('urdf_path') or item.get('source_path')):
            warnings.append(f"{iid}: missing mesh/URDF path (PREVIEW_ONLY)")
        if item.get('metadata_only') is True:
            warnings.append(f"{iid}: metadata-only PREVIEW_ONLY")

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
    return report

if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('scene_dir', type=Path); ap.add_argument('--json', action='store_true')
    a = ap.parse_args(); rep = merge(a.scene_dir)
    if a.json: print(json.dumps(rep, indent=2))
