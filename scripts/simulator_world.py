#!/usr/bin/env python3
"""Physics world of an authored cell, composed from the cell's own saved and generated geometry.

Sources, all of them the saved cell's except the qualified catalog:

- ``config/moveit_collision_objects.yaml`` (generated planning truth; must be current with
  ``environment.yaml``): the support box and the destination container mesh, i.e. exactly the
  geometry MoveIt certifies plans against;
- ``environment.yaml``: robot model, end-effector capability, the destination's drop region and
  usable placement, and the authored manual test objects (shape, size, pose, class, material);
- the saved TaskIntent: which destination asset and region the task places into;
- ``catalog/simulation/qualified_physics.yaml``: the commissioned robot/tool envelope, qualified
  object materials and surface friction, and the qualified engine configuration (physics, systems,
  scene and light, taken verbatim by element from the pinned Stage-A world; its world name is bound
  by the qualified bridge configuration).

The destination is measured, not assumed: its floor height, interior and per-side wall heights come
from ray casting the planning mesh from the authored drop region (container_geometry.py) and are built
as static box proxies (floor + walls) that destination_binding adds back to the planning scene.
Anything outside the qualified envelope fails closed with the reason (unsupported robot/tool, missing
or unqualified material, non-box object, rotated destination, drop region above the measured floor,
stale generated files); nothing is substituted from the reference scene.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / 'catalog/simulation/qualified_physics.yaml'
TOLERANCE_M = 1e-6
RESTING_TOLERANCE_M = 1e-3
SUPPORT_MODEL, DESTINATION_MODEL = 'pick_support', 'target_bin_physics'   # names bound by the simulator backend


class SimulatorWorldError(ValueError):
    """The authored cell is outside the qualified physics envelope (fail closed)."""


def _sha256(data):
    return hashlib.sha256(data).hexdigest()


def _fmt(values):
    return ' '.join(repr(float(v)) for v in values)


def _vector(value, name, count=3):
    if (not isinstance(value, (list, tuple)) or len(value) != count or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value)):
        raise SimulatorWorldError(f'{name} must be {count} finite numbers')
    return [float(v) for v in value]


def load_catalog(path=CATALOG):
    catalog = yaml.safe_load(Path(path).read_text())
    if not isinstance(catalog, dict) or catalog.get('schema_version') != 'workcell_qualified_physics/v1':
        raise SimulatorWorldError('qualified physics catalog missing or of an unknown schema')
    return catalog


TEMPLATE = ROOT / load_catalog()['engine']['template']


def _cell(scene):
    scene = Path(scene)
    environment_bytes = (scene / 'environment.yaml').read_bytes()
    doc = yaml.safe_load(environment_bytes) or {}
    manifest_path = scene / 'config/moveit_collision_objects.yaml'
    if not manifest_path.is_file():
        raise SimulatorWorldError('the cell has no generated planning scene; Generate the cell first')
    manifest_bytes = manifest_path.read_bytes()
    manifest = yaml.safe_load(manifest_bytes) or {}
    if (manifest.get('source') or {}).get('physical_state_sha256') != _sha256(environment_bytes):
        raise SimulatorWorldError('the generated planning scene is stale against the saved environment; Generate the cell again')
    intent = yaml.safe_load((scene / 'config/workcell_builder_task_intent.yaml').read_text())
    if not isinstance(intent, dict) or intent.get('schema') != 'workcell_builder_task_intent/v2':
        raise SimulatorWorldError('the saved task must be a workcell_builder_task_intent/v2 intent')
    return doc, doc.get('environment', {}), manifest, intent, dict(
        environment_sha256=_sha256(environment_bytes), planning_manifest_sha256=_sha256(manifest_bytes))


def _commissioned(doc, catalog):
    robot = str((doc.get('robot') or {}).get('model') or '').lower()
    capability = (doc.get('end_effector') or {}).get('capability')
    allowed = (catalog.get('commissioned') or {}).get(robot)
    if allowed is None:
        raise SimulatorWorldError(f'robot model {robot or "<none>"!r} is not commissioned for physics simulation '
                                  f'(commissioned: {sorted(catalog.get("commissioned") or {})})')
    if capability not in allowed:
        raise SimulatorWorldError(f'end effector capability {capability!r} is not commissioned with {robot} for physics '
                                  f'simulation (commissioned: {allowed})')
    return dict(robot_model=robot, end_effector_capability=capability)


def _manifest_object(manifest, source_item_id, role):
    rows = [item for item in manifest.get('objects', []) if item.get('source_item_id') == source_item_id]
    if len(rows) != 1:
        raise SimulatorWorldError(f'{role} {source_item_id!r} missing or ambiguous in the generated planning scene')
    item = rows[0]
    pose = item.get('pose') or {}
    quaternion = _vector(pose.get('quaternion_xyzw'), f'{role} quaternion', 4)
    if item.get('frame_id') != 'world' or item.get('operation') != 'ADD':
        raise SimulatorWorldError(f'{role} {source_item_id!r} must be an ADD object in the world frame')
    if any(abs(v) > 1e-9 for v in quaternion[:3]) or abs(abs(quaternion[3]) - 1) > 1e-9:
        raise SimulatorWorldError(f'{role} {source_item_id!r}: only world-axis-aligned geometry is qualified')
    return item, _vector(pose.get('xyz'), f'{role} xyz')


def _engine(catalog):
    template = ET.fromstring(TEMPLATE.read_bytes())
    world = template.find('world')
    if world is None or world.get('name') != catalog['engine']['world_name']:
        raise SimulatorWorldError('qualified engine template world differs from the catalog')
    root = ET.Element('sdf', version=template.get('version'))
    out = ET.SubElement(root, 'world', name=world.get('name'))
    for element in world:
        if element.tag != 'model':                     # physics, systems, gravity, scene, light: verbatim
            out.append(copy.deepcopy(element))
    return root, out


def _surface(parent, mu, contact=None):
    surface = ET.SubElement(parent, 'surface')
    ode = ET.SubElement(ET.SubElement(surface, 'friction'), 'ode')
    ET.SubElement(ode, 'mu').text = repr(float(mu))
    ET.SubElement(ode, 'mu2').text = repr(float(mu))
    if contact:
        cod = ET.SubElement(ET.SubElement(surface, 'contact'), 'ode')
        for key in ('kp', 'kd', 'max_vel', 'min_depth'):
            ET.SubElement(cod, key).text = repr(float(contact[key]))


def _box(link, name, xyz, size, mu=None, diffuse=None, contact=None, pose=True):
    collision = ET.SubElement(link, 'collision', name=name)
    if pose:
        ET.SubElement(collision, 'pose').text = _fmt(list(xyz) + [0, 0, 0])
    ET.SubElement(ET.SubElement(ET.SubElement(collision, 'geometry'), 'box'), 'size').text = _fmt(size)
    if mu is not None:
        _surface(collision, mu, contact)
    visual = ET.SubElement(link, 'visual', name=name + '_visual')
    if pose:
        ET.SubElement(visual, 'pose').text = _fmt(list(xyz) + [0, 0, 0])
    ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = _fmt(size)
    ET.SubElement(ET.SubElement(visual, 'material'), 'diffuse').text = _fmt(diffuse or [0.3, 0.3, 0.32, 1])
    return collision


def _support(world, manifest, support_ref, catalog):
    item, xyz = _manifest_object(manifest, support_ref, 'support surface')
    geometry = item.get('collision_geometry') or {}
    if geometry.get('type') != 'box':
        raise SimulatorWorldError(f'support {support_ref!r}: only a box collision proxy is qualified')
    size = _vector(geometry.get('dimensions_m'), 'support dimensions')
    model = ET.SubElement(world, 'model', name=SUPPORT_MODEL)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = _fmt(xyz + [0, 0, 0])
    link = ET.SubElement(model, 'link', name='support_link')
    collision = ET.SubElement(link, 'collision', name='support_collision')
    ET.SubElement(ET.SubElement(ET.SubElement(collision, 'geometry'), 'box'), 'size').text = _fmt(size)
    _surface(collision, catalog['surfaces']['support']['friction_mu'])
    visual = ET.SubElement(link, 'visual', name='support_visual')
    ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = _fmt(size)
    ET.SubElement(ET.SubElement(visual, 'material'), 'diffuse').text = _fmt([0.3, 0.3, 0.32, 1])
    return dict(source_item_id=support_ref, object_id=item['id'], center_xyz=xyz, dimensions_m=size,
                surface_z_m=xyz[2] + size[2] / 2)


def _destination(world, environment, manifest, intent, catalog):
    from container_geometry import ContainerGeometryError, measure_container
    from physical_destination import resolve_destination
    target_ref, region_ref = intent['place']['target'].get('asset_ref'), intent['place']['target'].get('region_ref')
    try:
        destination = resolve_destination(environment, region_ref)
    except ValueError as exc:
        raise SimulatorWorldError(f'destination {region_ref!r}: {exc}') from exc
    if destination['target_id'] != target_ref:
        raise SimulatorWorldError(f'the task places into {target_ref!r} but region {region_ref!r} belongs to {destination["target_id"]!r}')
    if any(abs(v) > TOLERANCE_M for v in destination['target_pose_rpy'] + destination['placement_local']['pose_rpy']):
        raise SimulatorWorldError('only a world-axis-aligned destination and drop region are qualified')
    item, origin = _manifest_object(manifest, target_ref, 'destination')
    if math.dist(origin, destination['target_pose_xyz']) > TOLERANCE_M:
        raise SimulatorWorldError('the planning destination pose differs from the authored target; Generate the cell again')
    geometry = item.get('collision_geometry') or {}
    if geometry.get('type') != 'mesh' or 'vertices_m' not in geometry or 'triangles' not in geometry:
        raise SimulatorWorldError(f'destination {target_ref!r}: a container collision mesh is required')
    region = destination['placement_local']
    region_xyz, region_size = _vector(region['pose_xyz'], 'drop region xyz'), _vector(region['dimensions'], 'drop region size')
    try:
        measured = measure_container(geometry['vertices_m'], geometry['triangles'], region_xyz[:2], region_xyz[2], region_size[:2])
    except ContainerGeometryError as exc:
        raise SimulatorWorldError(f'destination {target_ref!r}: {exc}') from exc
    floor = measured['floor_top_z']
    for name, block in (('drop region', region), ('usable placement', destination['usable_placement'])):
        bottom = block['pose_xyz'][2] - block['dimensions'][2] / 2
        if bottom > floor + RESTING_TOLERANCE_M:
            raise SimulatorWorldError(
                f'destination {target_ref!r}: the authored {name} starts {bottom - floor:.4f} m above the measured container '
                f'floor (target-local z {floor:.4f}); a part would be released in free fall. Lower it to the floor.')
    t = catalog['construction']['container_wall_thickness_m']
    f = catalog['construction']['container_floor_thickness_m']
    (x0, x1), (y0, y1) = measured['interior_x'], measured['interior_y']
    ox, oy, oz = origin
    top = oz + floor
    model = ET.SubElement(world, 'model', name=DESTINATION_MODEL)
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='bin_link')
    mu = catalog['surfaces']['destination']['friction_mu']
    cx, cy = ox + (x0 + x1) / 2, oy + (y0 + y1) / 2
    boxes = {'floor': ([cx, cy, top - f / 2], [x1 - x0 + 2 * t, y1 - y0 + 2 * t, f])}
    walls = measured['wall_top_z']
    for side, xy, size in (('x_minus', (ox + x0 - t / 2, cy), (t, y1 - y0 + 2 * t)),
                           ('x_plus', (ox + x1 + t / 2, cy), (t, y1 - y0 + 2 * t)),
                           ('y_minus', (cx, oy + y0 - t / 2), (x1 - x0, t)),
                           ('y_plus', (cx, oy + y1 + t / 2), (x1 - x0, t))):
        height = walls[side] - floor + f
        boxes['wall_' + side] = ([xy[0], xy[1], top - f + height / 2], [size[0], size[1], height])
    for name, (xyz, size) in boxes.items():
        _box(link, name, xyz, size, mu=mu, diffuse=[0.2, 0.35, 0.65, 1])
    return dict(asset_ref=target_ref, region_ref=region_ref, object_id=item['id'],
                mesh_sha256=geometry.get('sha256'), mesh_reference=geometry.get('mesh_reference'),
                measured_local=measured, floor_top_z=top,
                interior_world=dict(x=[ox + x0, ox + x1], y=[oy + y0, oy + y1]),
                wall_top_world_z={side: oz + z for side, z in walls.items()},
                proxy_wall_thickness_m=t, proxy_floor_thickness_m=f)


def _manual_objects(doc):
    source = ((doc.get('workcell_studio') or {}).get('pick_zone') or {}).get('object_source') or {}
    if source.get('mode') != 'manual_simulated':
        raise SimulatorWorldError('the physics simulator needs authored manual test objects (Create New Cell: object '
                                  'source manual_simulated); live or recorded perception is not simulated')
    rows = source.get('manual_observations', [source.get('manual_observation')] if source.get('manual_observation') else [])
    if not rows or not all(isinstance(row, dict) for row in rows):
        raise SimulatorWorldError('the cell has no authored manual test object')
    return rows


def _objects(world, doc, support, catalog):
    clearance = catalog['construction']['object_spawn_clearance_m']
    reserved = {SUPPORT_MODEL, DESTINATION_MODEL}
    objects = []
    for row in _manual_objects(doc):
        name = str(row.get('object_id') or '')
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name) or name in reserved:
            raise SimulatorWorldError(f'test object id {name!r} is not a valid, unique model name')
        reserved.add(name)
        if row.get('shape') != 'box':
            raise SimulatorWorldError(f'{name}: only box test objects are qualified (authored {row.get("shape")!r})')
        material_id = row.get('material')
        material = (catalog.get('materials') or {}).get(material_id)
        if material is None:
            raise SimulatorWorldError(f'{name}: physics material {material_id!r} is not qualified '
                                      f'(qualified: {sorted(catalog.get("materials") or {})}); author the object material')
        size = _vector(row.get('dimensions'), f'{name} dimensions')
        pose = row.get('pose') or {}
        if pose.get('frame_id', 'world') != 'world':
            raise SimulatorWorldError(f'{name}: authored pose must be in the world frame')
        xyz, rpy = _vector(pose.get('xyz'), f'{name} xyz'), _vector(pose.get('rpy', [0, 0, 0]), f'{name} rpy')
        if any(v <= 0 for v in size) or abs(rpy[0]) > TOLERANCE_M or abs(rpy[1]) > TOLERANCE_M:
            raise SimulatorWorldError(f'{name}: positive dimensions resting flat (roll = pitch = 0) required')
        rest = support['surface_z_m'] + size[2] / 2
        if abs(xyz[2] - rest) > RESTING_TOLERANCE_M:
            raise SimulatorWorldError(f'{name}: authored z {xyz[2]} does not rest on the support surface (z {rest:.4f})')
        half = [v / 2 for v in support['dimensions_m'][:2]]
        if any(abs(xyz[i] - support['center_xyz'][i]) > half[i] for i in range(2)):
            raise SimulatorWorldError(f'{name}: authored position is outside the support surface')
        mass = material['density_kg_m3'] * size[0] * size[1] * size[2]
        spawn = [xyz[0], xyz[1], rest + clearance] + rpy
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'pose').text = _fmt(spawn)
        link = ET.SubElement(model, 'link', name='link')
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = repr(mass)
        inertia = ET.SubElement(inertial, 'inertia')
        for axis, (a, b) in zip(('ixx', 'iyy', 'izz'), ((1, 2), (0, 2), (0, 1))):
            ET.SubElement(inertia, axis).text = repr(mass * (size[a] ** 2 + size[b] ** 2) / 12)
        for axis in ('ixy', 'ixz', 'iyz'):
            ET.SubElement(inertia, axis).text = '0'
        collision = ET.SubElement(link, 'collision', name='collision')
        ET.SubElement(ET.SubElement(ET.SubElement(collision, 'geometry'), 'box'), 'size').text = _fmt(size)
        _surface(collision, material['friction_mu'], material['contact'])
        visual = ET.SubElement(link, 'visual', name='visual')
        ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = _fmt(size)
        ET.SubElement(ET.SubElement(visual, 'material'), 'diffuse').text = _fmt(material['visual_diffuse'])
        objects.append(dict(object_id=name, class_id=row.get('class_id'), material=material_id, dimensions_m=size,
                            authored_pose=xyz + rpy, spawn_pose=spawn, mass_kg=mass,
                            within_qualified_dimensions=size == [float(v) for v in material.get('qualified_dimensions_m', [])]))
    return objects


def planning_only_objects(manifest, represented):
    """Planning objects with no physics body (out of the physical task: e.g. a camera); reported, not hidden."""
    return sorted(item['id'] for item in manifest.get('objects', []) if item.get('source_item_id') not in represented)


def derive_world(scene, catalog_path=CATALOG):
    """(world SDF text, derivation report) for the authored cell at ``scene``."""
    catalog = load_catalog(catalog_path)
    doc, environment, manifest, intent, provenance = _cell(scene)
    if (intent.get('routing') or {}).get('mode') == 'class':
        # The qualified world binds exactly one physical destination (DESTINATION_MODEL); a routed task
        # may place into another one, which has no physics body or measured walls here.
        raise SimulatorWorldError('TASK_ROUTING_PHYSICS_UNQUALIFIED: class routing is qualified for fake-hardware '
                                  'planning only; the qualified physics world binds one destination. Use Plan Saved '
                                  'Task (fake hardware)')
    envelope = _commissioned(doc, catalog)
    root, world = _engine(catalog)
    target_ref = intent['place']['target'].get('asset_ref')
    targets = [a for a in environment.get('assets', []) if a.get('id') == target_ref]
    if len(targets) != 1 or not targets[0].get('support_surface_ref'):
        raise SimulatorWorldError(f'destination {target_ref!r} missing or not on an authored support surface')
    support = _support(world, manifest, targets[0]['support_surface_ref'], catalog)
    destination = _destination(world, environment, manifest, intent, catalog)
    objects = _objects(world, doc, support, catalog)
    text = ET.tostring(root, encoding='unicode')
    template_bytes = TEMPLATE.read_bytes()
    report = dict(schema='workcell_simulator_world/v2', scene=str(Path(scene).resolve()), **provenance, **envelope,
                  catalog=str(Path(catalog_path)), catalog_sha256=_sha256(Path(catalog_path).read_bytes()),
                  engine_template=str(TEMPLATE), engine_template_sha256=_sha256(template_bytes),
                  world_name=world.get('name'), world_sha256=_sha256(text.encode()),
                  support=support, destination=destination, objects=objects,
                  planning_only_objects=planning_only_objects(manifest, {support['source_item_id'], destination['asset_ref']}))
    return text, report


def engine_equivalent(text, template_text=None):
    """Whether a world's non-model elements equal the qualified engine template's verbatim."""
    def engine(source):
        world = ET.fromstring(source.encode() if isinstance(source, str) else source).find('world')
        return world.get('name'), [ET.tostring(e) for e in world if e.tag != 'model']
    return engine(text) == engine(template_text if template_text is not None else TEMPLATE.read_bytes())


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
    report['engine_equivalent_to_qualified_template'] = engine_equivalent(text)
    args.output.write_text(text)
    if args.report:
        args.report.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'status': 'PASS', **report}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
