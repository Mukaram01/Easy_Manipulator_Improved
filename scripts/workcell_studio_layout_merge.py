#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
import yaml
import copy
import hashlib
import shutil
import struct
import math
import os
from authored_yaml import write_preserving
from asset_mounting import validate_physical, validate, RELATION_FIELDS

REPOSITORY = Path(__file__).resolve().parents[1]
IMPORTED_LIBRARY = REPOSITORY / 'workcell_builder/workcell_builder/assets/imported'
TRACKED_LIBRARIES = (REPOSITORY / 'assets', REPOSITORY / 'workcell_builder/workcell_builder/assets')


def _inside(path: Path, root: Path) -> bool:
    # samefile, not string prefixes: the runtime view reaches this repository by another path.
    for parent in path.parents:
        try:
            if os.path.samefile(parent, root):
                return True
        except OSError:
            continue
    return False


def _scene_copy_folder(source: Path) -> Path:
    """Where Save copies a placed catalog mesh in the cell. Only the repository's tracked asset
    library goes to assets/library/, which a reference scene can commit; imported CAD (the local
    imported-asset library, or any file outside the repository) goes to assets/imported/catalog/,
    which Git ignores in every scene."""
    if not _inside(source, IMPORTED_LIBRARY) and any(_inside(source, root) for root in TRACKED_LIBRARIES):
        return Path('assets/library')
    return Path('assets/imported/catalog')


def _load(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    return data if isinstance(data, dict) else {}

def _save_yaml(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')

def _same_pose(a, b, tolerance=1e-9) -> bool:
    try:
        return len(a) == len(b) and all(abs(float(x) - float(y)) <= tolerance for x, y in zip(a, b))
    except (TypeError, ValueError):
        return False


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
        validate_physical(env.get("environment", {}))
        manifest = without_deleted(manifest)
        write_preserving(scene_dir/'environment.yaml', env)
        _save_yaml(scene_dir/'scene_manifest.yaml', manifest)
    generated = scene_dir/'generated'; generated.mkdir(exist_ok=True)

    warnings: list[str] = []
    blockers: list[str] = []
    validate_physical(env.get("environment", {}))
    layout_items = layout.get('items') if isinstance(layout.get('items'), list) else []
    if save_authored:
        validate(layout_items)
        # Catalog placement may hold a workstation path while editing. Runtime
        # resources use the existing scene-owned assets contract, never an
        # absolute path accepted by a relaxed publisher gate.
        for item in layout_items:
            mesh = item.get('mesh') if isinstance(item, dict) else None
            if not item.get('catalog_asset_id') or not isinstance(mesh, dict):
                continue
            source = Path(str(mesh.get('path', '')))
            if not source.is_absolute():
                continue
            if not source.is_file():
                raise ValueError(f"asset {item['id']}: catalog mesh {source} does not exist")
            if source.suffix.lower() != '.stl':
                # Collision geometry is read from STL only (generate_moveit_collision_manifest).
                raise ValueError(f"asset {item['id']}: only STL meshes can be placed in a cell, because collision "
                                 f"geometry is read from STL; convert {source.name} to STL and import it again")
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            relative = _scene_copy_folder(source) / (digest[:16] + '.stl')
            destination = scene_dir / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            if not destination.exists():
                data = source.read_bytes()
                count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
                if count and 84 + 50 * count == len(data):
                    shutil.copy2(source, destination)
                else:
                    # RViz accepts binary STL only. Reuse the bounded collision
                    # reader and retain every catalog triangle and coordinate.
                    from generate_moveit_collision_manifest import read_stl_mesh
                    vertices, triangles = read_stl_mesh(source)
                    binary = bytearray(b'Workcell catalog STL'.ljust(80, b'\0'))
                    binary.extend(struct.pack('<I', len(triangles)))
                    for triangle in triangles:
                        a, b, c = [vertices[i] for i in triangle]
                        u, v = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
                        normal = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
                        length = math.sqrt(sum(x*x for x in normal))
                        normal = [x/length for x in normal] if length else [0., 0., 0.]
                        binary.extend(struct.pack('<12fH', *normal, *a, *b, *c, 0))
                    destination.write_bytes(binary)
            mesh['path'] = relative.as_posix()
            mesh.pop('source_package', None)
            for key in ('mesh_path', 'source_path'):
                if key in item:
                    item[key] = relative.as_posix()
        physical = env.setdefault('environment', {})
        previous_physical = copy.deepcopy(physical)
        editor_to_physical = {}
        for key in ('support_surfaces', 'assets', 'sensors', 'task_zones'):
            for record in physical.get(key, []):
                editor_to_physical.setdefault(record.get('layout_item_ref', record.get('id')), record.get('id'))
        from physical_destination import resolve_destination, is_destination_zone, destination_ids
        for zone_id in destination_ids(previous_physical, env.get('task') or {}):
            resolve_destination(previous_physical, zone_id)
        for item in layout_items:
            if not isinstance(item, dict) or item.get('locked') is True or item.get('editable') is False:
                continue
            iid = item.get('id')
            if not iid:
                raise ValueError('Physical authored item requires a stable ID')
            collection = ('task_zones' if item.get('category') == 'zone' else
                          'support_surfaces' if item.get('role') == 'support_surface' else 'assets')
            targets = []
            for key in ('support_surfaces', 'assets', 'sensors', 'task_zones'):
                for record in physical.get(key, []):
                    # Semantic IDs need not equal their Inspector layout ID.
                    # Update every existing projection of that authored item,
                    # retaining the IDs used by task/runtime bindings.
                    if record.get('id') == iid or record.get('layout_item_ref') == iid:
                        targets.append(record)
            if not targets:
                target = {'id': iid}
                physical.setdefault(collection, []).append(target)
                targets.append(target)
            for target in targets:
                for key in ('type', 'role', 'display_name', 'category', 'frame', 'geometry_type',
                            'mesh', 'collision', 'dimensions', 'asset_class', 'description',
                            'catalog_asset_id', 'support_surface_ref', 'task_zone_ref',
                            'target_ref', 'usable_placement', 'placement_local'):
                    if key in item:
                        target[key] = copy.deepcopy(item[key])
                if item.get('mesh') and 'collision' not in target:
                    target['collision'] = {'enabled': True, 'mode': 'mesh'}
                for key in RELATION_FIELDS:
                    if key in item:
                        target[key] = copy.deepcopy(item[key])
                    else:
                        target.pop(key, None)
                if target.get('parent_object') not in (None, '', 'world'):
                    target['parent_object'] = editor_to_physical.get(target['parent_object'], target['parent_object'])
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
                        if isinstance(mirror, dict) and mirror.get('id') == target['id'] and mirror is not target:
                            mirror.update(copy.deepcopy(target))
        from physical_destination import relative_placement, resolve_destination
        old_zones = _index(previous_physical.get('task_zones', []))
        targets_by_id = _index(physical.get('assets', []))
        for zone in physical.get('task_zones', []):
            if not is_destination_zone(zone):
                continue
            previous = old_zones.get(zone['id'], {})
            edited = any(zone.get(key) != previous.get(key)
                         for key in ('pose_xyz', 'pose_rpy', 'dimensions'))
            if edited:
                zone['placement_local'] = relative_placement(targets_by_id[zone['target_ref']], zone)
            resolved = resolve_destination(physical, zone['id'], check_projection=False)
            for key in ('pose_xyz', 'pose_rpy', 'dimensions'):
                zone[key] = copy.deepcopy(resolved[key])
            for item in layout_items:
                if item.get('id') == zone.get('layout_item_ref', zone['id']):
                    item['pose'] = {'xyz': zone['pose_xyz'], 'rpy': zone['pose_rpy']}
                    item['dimensions'] = zone['dimensions']
                    item['placement_local'] = copy.deepcopy(zone['placement_local'])
            for mirror in env.get('task_zones', []):
                if mirror.get('id') == zone['id']:
                    mirror.update(copy.deepcopy(zone))
        validate_physical(physical)
        camera = env.get('camera') or {}
        camera_id = camera.get('camera_id', camera.get('id'))
        before = _index(previous_physical.get('assets', []) + previous_physical.get('sensors', []))
        for asset in physical.get('assets', []) + physical.get('sensors', []):
            if camera_id and asset.get('id') == camera_id:
                # The camera block mirrors the camera asset's authored extrinsics. It
                # follows only when this Save moved the camera; an unrelated Save never
                # rewrites a (possibly calibrated) camera pose.
                projected = asset['pose_xyz'] + asset['pose_rpy']
                old = before.get(camera_id, {})
                moved = not _same_pose(old.get('pose_xyz', []) + old.get('pose_rpy', []), projected)
                if moved:
                    camera['pose'] = projected
                elif not _same_pose(camera.get('pose') or [], projected):
                    warnings.append(f"camera {camera_id}: camera.pose {camera.get('pose')} differs from its authored asset "
                                    f"pose {projected}; left unchanged (move the camera, or verify its calibration)")
        write_preserving(scene_dir/'environment.yaml', env)
        write_preserving(scene_dir/'layout'/'workcell_studio_layout.yaml', layout)
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
