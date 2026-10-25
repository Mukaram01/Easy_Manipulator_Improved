#!/usr/bin/env python3
"""Physics world for an authored cell, derived from its saved data and the qualified Stage-A world.

The qualified sparse commissioning world (``scenes/ur5_2f_test/worlds/stage_a0_sparse.sdf``, pinned by
git blob in run_stage_a1_finish.SOURCE_WORLDS) is the template: its physics, systems, scene, light, support
model, bin wall/floor construction and test-object material are kept verbatim. Only what the cell author
saved is substituted:

- the physical destination bin: the place target's ``usable_placement`` interior (world-axis-aligned, floor
  top at the target origin, as destination_binding verifies at launch) closed by the template's wall and
  floor thickness and wall height;
- the physical test object(s): the Create New Cell ``manual_observation`` (box) at its authored pose, spawned
  with the template object's drop clearance and the template material (density, surface, visual).

Anything the template cannot represent fails closed (rotated bin, non-box object, object not resting on the
authored support). For the reference cell the derived world equals the qualified world model for model.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = ROOT / 'scenes/ur5_2f_test/worlds/stage_a0_sparse.sdf'
TOLERANCE_M = 1e-6


class SimulatorWorldError(ValueError):
    """The authored cell cannot be represented by the qualified world template (fail closed)."""


def _numbers(text, count):
    values = [float(v) for v in (text or '').split()]
    if len(values) != count or not all(math.isfinite(v) for v in values):
        raise SimulatorWorldError(f'expected {count} finite numbers, got {text!r}')
    return values


def _fmt(values):
    return ' '.join(repr(float(v)) for v in values)


def _vector(value, name, count=3):
    if (not isinstance(value, (list, tuple)) or len(value) != count or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value)):
        raise SimulatorWorldError(f'{name} must be {count} finite numbers')
    return [float(v) for v in value]


def _authored(scene):
    doc = yaml.safe_load((Path(scene) / 'environment.yaml').read_text()) or {}
    return doc, doc.get('environment', doc)


def _intent(scene):
    text = (Path(scene) / 'config/workcell_builder_task_intent.yaml').read_text()
    intent = yaml.safe_load(text)  # the Task Editor saves indented JSON, a YAML subset
    if not isinstance(intent, dict) or intent.get('schema') != 'workcell_builder_task_intent/v2':
        raise SimulatorWorldError('the saved task must be a workcell_builder_task_intent/v2 intent')
    return intent


def _template_parts(world):
    models = {m.get('name'): m for m in world.findall('model')}
    support, bin_model = models.get('pick_support'), models.get('target_bin_physics')
    parts = [m for name, m in models.items() if name not in ('pick_support', 'target_bin_physics')]
    if support is None or bin_model is None or len(parts) != 1:
        raise SimulatorWorldError('qualified template must hold pick_support, target_bin_physics and one part')
    return support, bin_model, parts[0]


def _bin_construction(bin_model):
    link = bin_model.find('link')
    cols = {c.get('name'): c for c in link.findall('collision')}
    floor = cols['floor']; fpose = _numbers(floor.findtext('pose'), 6); fsize = _numbers(floor.findtext('geometry/box/size'), 3)
    wall = cols['wall_x_minus']; wpose = _numbers(wall.findtext('pose'), 6); wsize = _numbers(wall.findtext('geometry/box/size'), 3)
    floor_top = fpose[2] + fsize[2] / 2
    return dict(floor_thickness=fsize[2], wall_thickness=wsize[0], wall_height=wsize[2],
                wall_base_below_floor_top=floor_top - (wpose[2] - wsize[2] / 2), link=link)


def _surface_z(environment, support_ref):
    surfaces = [s for s in environment.get('support_surfaces', []) if s.get('id') == support_ref]
    if len(surfaces) != 1:
        raise SimulatorWorldError(f'support surface {support_ref!r} missing or ambiguous')
    z = surfaces[0].get('surface_z_m')
    if isinstance(z, bool) or not isinstance(z, (int, float)) or not math.isfinite(z):
        raise SimulatorWorldError(f'support surface {support_ref!r} has no finite surface_z_m')
    return float(z)


def _destination_target(environment, intent):
    ref = intent['place']['target'].get('asset_ref')
    targets = [a for a in environment.get('assets', []) if a.get('id') == ref]
    if len(targets) != 1:
        raise SimulatorWorldError(f'place target {ref!r} missing or ambiguous in environment.yaml')
    target = targets[0]
    usable = target.get('usable_placement')
    if not isinstance(usable, dict):
        raise SimulatorWorldError(f'place target {ref!r} has no usable_placement')
    rpy = _vector(target.get('pose_rpy', [0, 0, 0]), 'target pose_rpy') + _vector(usable.get('pose_rpy', [0, 0, 0]), 'usable pose_rpy')
    if any(abs(v) > TOLERANCE_M for v in rpy):
        raise SimulatorWorldError('the qualified bin template is world-axis-aligned; rotated destinations are unsupported')
    return target, _vector(target['pose_xyz'], 'target pose_xyz'), _vector(usable['pose_xyz'], 'usable pose_xyz'), \
        _vector(usable['dimensions'], 'usable dimensions')


def _manual_objects(doc):
    source = ((doc.get('workcell_studio') or {}).get('pick_zone') or {}).get('object_source') or {}
    if source.get('mode') != 'manual_simulated' or not isinstance(source.get('manual_observation'), dict):
        raise SimulatorWorldError('the physical simulator needs an authored manual test object '
                                  '(Create New Cell: object source manual_simulated)')
    return [source['manual_observation']]


def derive_world(scene, template=TEMPLATE):
    """(world SDF text, derivation report) for the authored cell at ``scene``."""
    scene = Path(scene)
    template_bytes = Path(template).read_bytes()
    world_doc = ET.fromstring(template_bytes)
    world = world_doc.find('world')
    support, bin_model, part = _template_parts(world)
    doc, environment = _authored(scene)
    intent = _intent(scene)
    target, origin, usable_xyz, usable_dims = _destination_target(environment, intent)
    surface_z = _surface_z(environment, target.get('support_surface_ref'))

    # Support: the template plane sits at z = 0 (its model carries no pose).
    if support.find('pose') is not None or abs(surface_z) > TOLERANCE_M:
        raise SimulatorWorldError(f'authored support surface z {surface_z} differs from the qualified support plane (0)')

    # Bin: interior = usable_placement footprint, floor top = target origin (destination_binding contract).
    c = _bin_construction(bin_model)
    link = c['link']; t, h, f = c['wall_thickness'], c['wall_height'], c['floor_thickness']
    cx, cy, top = origin[0] + usable_xyz[0], origin[1] + usable_xyz[1], origin[2]
    ix, iy = usable_dims[0], usable_dims[1]
    if ix <= 0 or iy <= 0:
        raise SimulatorWorldError('usable_placement footprint must be positive')
    wall_z = top - c['wall_base_below_floor_top'] + h / 2
    boxes = {'floor': ([cx, cy, top - f / 2], [ix + 2 * t, iy + 2 * t, f]),
             'wall_x_minus': ([cx - ix / 2 - t / 2, cy, wall_z], [t, iy + 2 * t, h]),
             'wall_x_plus': ([cx + ix / 2 + t / 2, cy, wall_z], [t, iy + 2 * t, h]),
             'wall_y_minus': ([cx, cy - iy / 2 - t / 2, wall_z], [ix, t, h]),
             'wall_y_plus': ([cx, cy + iy / 2 + t / 2, wall_z], [ix, t, h])}
    for collision in link.findall('collision'):
        name = collision.get('name')
        if name not in boxes:
            raise SimulatorWorldError(f'unexpected template bin collision {name!r}')
        xyz, size = boxes[name]
        collision.find('pose').text = _fmt(xyz + [0, 0, 0])
        collision.find('geometry/box/size').text = _fmt(size)
    for visual in link.findall('visual'):
        name = visual.get('name', '').replace('_visual', '')
        if name in boxes and visual.find('geometry/box/size') is not None:
            xyz, size = boxes[name]
            if visual.find('pose') is not None:
                visual.find('pose').text = _fmt(xyz + [0, 0, 0])
            visual.find('geometry/box/size').text = _fmt(size)

    # Test object(s): authored box at its authored pose, template material and drop clearance.
    t_pose = _numbers(part.findtext('pose'), 6)
    t_size = _numbers(part.findtext('link/collision/geometry/box/size'), 3)
    t_mass = float(part.findtext('link/inertial/mass'))
    density = t_mass / (t_size[0] * t_size[1] * t_size[2])
    drop = t_pose[2] - t_size[2] / 2           # template spawn clearance above its resting height (support z = 0)
    world.remove(part)
    objects = []
    for row in _manual_objects(doc):
        if row.get('shape') != 'box':
            raise SimulatorWorldError(f"test object {row.get('object_id')!r}: only box objects are supported")
        name = str(row.get('object_id') or '')
        if not name or not name.replace('_', '').isalnum() or name in ('pick_support', 'target_bin_physics'):
            raise SimulatorWorldError(f'test object id {name!r} is not a valid model name')
        size = _vector(row.get('dimensions'), f'{name} dimensions')
        pose = row.get('pose') or {}
        if pose.get('frame_id', 'world') != 'world':
            raise SimulatorWorldError(f'{name}: authored pose must be in the world frame')
        xyz, rpy = _vector(pose.get('xyz'), f'{name} xyz'), _vector(pose.get('rpy', [0, 0, 0]), f'{name} rpy')
        if any(v <= 0 for v in size) or abs(rpy[0]) > TOLERANCE_M or abs(rpy[1]) > TOLERANCE_M:
            raise SimulatorWorldError(f'{name}: positive dimensions resting flat (roll = pitch = 0) required')
        if abs(xyz[2] - (surface_z + size[2] / 2)) > 1e-3:
            raise SimulatorWorldError(f'{name}: authored z {xyz[2]} is not resting on the support ({surface_z + size[2] / 2})')
        model = copy.deepcopy(part)
        model.set('name', name)
        model.find('pose').text = _fmt([xyz[0], xyz[1], surface_z + size[2] / 2 + drop] + rpy)
        mass = density * size[0] * size[1] * size[2]
        inertia = model.find('link/inertial/inertia')
        for axis, (a, b) in zip(('ixx', 'iyy', 'izz'), ((1, 2), (0, 2), (0, 1))):
            inertia.find(axis).text = repr(mass * (size[a] ** 2 + size[b] ** 2) / 12)
        model.find('link/inertial/mass').text = repr(mass)
        for geometry in model.iter('geometry'):
            if geometry.find('box/size') is not None:
                geometry.find('box/size').text = _fmt(size)
        world.append(model)
        objects.append(dict(object_id=name, dimensions_m=size, pose=[xyz[0], xyz[1], surface_z + size[2] / 2 + drop] + rpy,
                            mass_kg=mass))

    text = ET.tostring(world_doc, encoding='unicode')
    report = dict(schema='workcell_simulator_world/v1', template=str(Path(template)),
                  template_sha256=hashlib.sha256(template_bytes).hexdigest(),
                  world_sha256=hashlib.sha256(text.encode()).hexdigest(),
                  destination=dict(asset_ref=target['id'], floor_top_z=top, interior_m=[ix, iy],
                                   wall_thickness_m=t, wall_height_m=h, floor_thickness_m=f),
                  support=dict(surface_ref=target.get('support_surface_ref'), surface_z_m=surface_z),
                  objects=objects, material=dict(density_kg_m3=density, spawn_clearance_m=drop))
    return text, report


def equivalent_models(a_text, b_text, tolerance=1e-9):
    """Model-for-model numeric equality of two worlds (names, poses, sizes, inertials, surfaces)."""
    def canonical(text):
        world = ET.fromstring(text.encode() if isinstance(text, str) else text).find('world')
        out = {}
        for model in world.findall('model'):
            items = []
            for element in model.iter():
                values = (element.text or '').split()
                try:
                    numbers = [float(v) for v in values]
                except ValueError:
                    numbers = None
                items.append((element.tag, element.get('name'), tuple(numbers) if numbers else (element.text or '').strip()))
            out[model.get('name')] = items
        return out
    a, b = canonical(a_text), canonical(b_text)
    if a.keys() != b.keys():
        return False
    for name in a:
        if len(a[name]) != len(b[name]):
            return False
        for (tag_a, n_a, v_a), (tag_b, n_b, v_b) in zip(a[name], b[name]):
            if tag_a != tag_b or n_a != n_b:
                return False
            if isinstance(v_a, tuple) and isinstance(v_b, tuple):
                if len(v_a) != len(v_b) or any(abs(x - y) > tolerance for x, y in zip(v_a, v_b)):
                    return False
            elif v_a != v_b:
                return False
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('scene', type=Path, help='authored cell (scene package directory)')
    parser.add_argument('--output', type=Path, required=True, help='world SDF to write')
    parser.add_argument('--report', type=Path, help='derivation report (JSON)')
    args = parser.parse_args(argv)
    try:
        text, report = derive_world(args.scene)
    except (SimulatorWorldError, OSError, KeyError, ValueError) as exc:
        print(json.dumps({'status': 'BLOCKED', 'reason': str(exc)}))
        return 2
    report['equivalent_to_qualified_template'] = equivalent_models(text, TEMPLATE.read_text())
    args.output.write_text(text)
    if args.report:
        args.report.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'status': 'PASS', **report}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
