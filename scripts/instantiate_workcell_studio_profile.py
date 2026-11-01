#!/usr/bin/env python3
"""Materialize New Cell selections using reviewed physical defaults.

This is the wizard's adapter to the existing authored scene and package generator,
not a scene clone: only equipment/physical sections and the equipment xacro are
reused. Identity, TaskIntent, manifests and caches belong to the new cell.
"""
import argparse
import copy
import json
from pathlib import Path
import re
import sys
import shutil
import tempfile
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
XACRO = '{http://www.ros.org/wiki/xacro}'
PROFILES = ROOT / 'catalog/workcell_studio_profiles'
SUPPORTED_SCENES = ROOT / 'workcell_studio_supported_scenes.yaml'


def load_profile(profile_id):
    if Path(profile_id).name != profile_id:
        raise ValueError('Invalid recommended profile ID')
    profile = yaml.safe_load((PROFILES / (profile_id + '.yaml')).read_text())
    source = yaml.safe_load((ROOT / profile['environment_source']).read_text())
    from physical_destination import resolve_destination
    resolve_destination(source['environment'], profile['place_region'])
    _verify_profile_equipment(profile, source)
    return profile, source


def _capabilities():
    sys.path.insert(0, str(ROOT / 'scripts'))
    from capability_registry import load_capability_registry
    registry = load_capability_registry()
    records = {kind: {} for kind in ('robot', 'end_effector')}
    for capability_id in registry.ids():
        record = registry.get(capability_id)
        kind = record.schema_version.split('_capability/', 1)[0]
        if kind in records:
            records[kind][capability_id] = record
    return records


def _verify_profile_equipment(profile, source):
    """A profile reviews the equipment its source launches; a mislabelled profile never reaches a cell."""
    records = _capabilities()
    robot, tool = records['robot'].get(profile['robot_capability']), records['end_effector'].get(profile['tool_capability'])
    if robot is None or tool is None:
        raise ValueError(f"Profile {profile['id']}: unknown capability {profile['robot_capability']!r} or "
                         f"{profile['tool_capability']!r} (catalog/capabilities)")
    launched = (source['robot'].get('srdf') or {}).get('package')
    if launched != robot.payload.get('moveit_config_package'):
        raise ValueError(f"Profile {profile['id']}: its source launches {launched!r}, not the MoveIt config of "
                         f"{profile['robot_capability']!r}")
    if source['end_effector'].get('capability') != profile['tool_capability']:
        raise ValueError(f"Profile {profile['id']}: its source binds tool {source['end_effector'].get('capability')!r}, "
                         f"not {profile['tool_capability']!r}")


def _package(name):
    """An installed model package (assets/<robots|end_effectors>/<vendor>/<package>), or None."""
    hits = sorted(ROOT.glob(f'assets/*/*/{name}/package.xml')) if name else []
    return hits[0].parent if hits else None


def _unprefixed(name):
    return re.sub(r'\$\{[^}]*\}', '', name or '')


def _srdf_groups(package):
    """Planning groups (and chain base/tip links) of an installed MoveIt config."""
    groups = {}
    for path in sorted((package / 'config').glob('*.srdf*')) if package else []:
        for group in ET.fromstring(path.read_text()).iter('group'):
            chain = group.find('chain')
            groups.setdefault(group.get('name'), {
                'base': _unprefixed(chain.get('base_link')) if chain is not None else None,
                'tip': _unprefixed(chain.get('tip_link')) if chain is not None else None})
    return groups


def _tool_root_link(model):
    """The link a tool macro attaches to its parent (the child of its ``${parent}`` joint)."""
    if model is None or not model.is_file():
        return None
    for joint in ET.fromstring(model.read_text()).iter('joint'):
        parent, child = joint.find('parent'), joint.find('child')
        if parent is not None and child is not None and parent.get('link') == '${parent}':
            return _unprefixed(child.get('link'))
    return None


def _model_links(model):
    """The links an installed model file defines (prefix placeholders removed)."""
    if model is None or not model.is_file():
        return []
    return _unique([_unprefixed(link.get('name')) for link in ET.fromstring(model.read_text()).iter('link')])


def _find_path(filename):
    """An xacro ``$(find package)/relative`` include resolved against the installed model packages."""
    match = re.fullmatch(r'\$\(find ([\w-]+)\)/(.+)', (filename or '').strip())
    package = _package(match.group(1)) if match else None
    return package / match.group(2) if package else None


def _xacro_name(element):
    """The local name of an xacro element. xacro reads the ``xacro:`` prefix, so installed models bind it to
    several URIs (www.ros.org/wiki/xacro, ros.org/wiki/xacro, wiki.ros.org/xacro)."""
    namespace, _, name = element.tag.rpartition('}')
    return name if namespace.rstrip('/').endswith('/xacro') else None


def _macro_names(path):
    return {macro.get('name') for macro in ET.fromstring(path.read_text()).iter() if _xacro_name(macro) == 'macro'}


def _literal_origin(call):
    origins = call.findall('origin')
    if len(origins) != 1:
        return None
    try:
        pose = {axis: [float(v) for v in (origins[0].get(axis) or '0 0 0').split()] for axis in ('xyz', 'rpy')}
    except ValueError:
        return None
    return pose if all(len(v) == 3 for v in pose.values()) else None


def _reference_calls(model_path, robot, tool):
    """(robot call, tool call, include map) of a scene model that mounts the tool's installed macro on a robot tip."""
    model = ET.fromstring(model_path.read_text())
    includes = {}
    for include in (e for e in model if _xacro_name(e) == 'include'):
        path = _find_path(include.get('filename'))
        if path is not None and path.is_file():
            for name in _macro_names(path):
                includes[name] = (include, path)
    calls = [e for e in model if _xacro_name(e) in includes]
    tool_model = (ROOT / tool['model']).resolve()
    tool_calls = [e for e in calls if includes[_xacro_name(e)][1].resolve() == tool_model]
    if len(tool_calls) != 1 or tool_calls[0].get('parent') not in robot['tip_frames']:
        return None
    # The robot macro is the one that defines the link the tool is mounted on.
    defines_parent = re.compile(r'<link\s+name="(?:\$\{[^}]*\})?' + re.escape(tool_calls[0].get('parent')) + '"')
    robot_calls = [e for e in calls if e is not tool_calls[0] and
                   defines_parent.search(includes[_xacro_name(e)][1].read_text())]
    if len(robot_calls) != 1:
        return None
    return model, robot_calls[0], tool_calls[0], includes


def _reference_assembly(robot, tool):
    """How an existing supported scene already mounts this installed tool on this robot, or None.

    The 3D authoring model of a configurable pair reuses that scene's robot and tool macro calls and its literal
    tool mount; it is geometry for layout authoring, never a reviewed runtime binding."""
    moveit = Path(robot['moveit_config']).name if robot.get('moveit_config') else None
    if not moveit or not tool.get('model') or not (ROOT / tool['model']).is_file():
        return None
    registry = yaml.safe_load(SUPPORTED_SCENES.read_text()) if SUPPORTED_SCENES.is_file() else {}
    for entry in (registry or {}).get('scenes') or []:
        urdf = ROOT / (entry.get('scene_path') or '') / 'urdf'
        model_path = urdf / 'scene.urdf.xacro'
        if not entry.get('enabled') or not model_path.is_file():
            continue
        # The scene launches this robot: its SRDF includes the robot's MoveIt config (as reviewed profiles check).
        if not any(f'$(find {moveit})/' in path.read_text() for path in urdf.glob('*.srdf.xacro')):
            continue
        found = _reference_calls(model_path, robot, tool)
        mount = _literal_origin(found[2]) if found else None
        if mount is None:
            continue
        _, robot_call, tool_call, _ = found
        return {'scene': entry.get('scene_name') or model_path.parent.parent.name,
                'source': str(model_path.relative_to(ROOT)), 'robot_macro': _xacro_name(robot_call),
                'tool_macro': _xacro_name(tool_call), 'tool_parent': tool_call.get('parent'),
                'tool_mount': mount}
    return None


def _unique(*values):
    out = []
    for value in values:
        for item in value if isinstance(value, (list, tuple)) else [value]:
            if item and item not in out:
                out.append(item)
    return out


def _status(payload):
    return {key: payload.get(key) or 'unknown' for key in
            ('support_status', 'runtime_status', 'fake_hardware_status', 'real_hardware_status')}


def _robot(capability_id, record):
    p = record.payload
    model = ROOT / p['urdf_or_xacro'] if p.get('urdf_or_xacro') else None
    moveit = _package(p.get('moveit_config_package'))
    groups = _srdf_groups(moveit)
    preview = 'preview_only' in (p.get('support_status'), p.get('runtime_status'))
    missing = ([] if model is not None and model.is_file() else ['robot model (urdf_or_xacro)']) + (
        [] if moveit else ['MoveIt config (moveit_config_package)'])
    group = p.get('default_planning_group') or p.get('planning_group') or next(iter(groups), None)
    return {
        'id': capability_id, 'label': p.get('display_name') or p.get('label') or capability_id,
        'family': p.get('family') or 'unknown', 'brand': p.get('brand') or '', 'source': str(record.path.relative_to(ROOT)),
        'installed': not missing, 'missing': missing, 'preview_only': preview, **_status(p),
        'model': str(model.relative_to(ROOT)) if model is not None and model.is_file() else None,
        'moveit_config': str(moveit.relative_to(ROOT)) if moveit else None,
        # Installed MoveIt groups first; catalog-only names are listed but never invented as defaults.
        'planning_groups': _unique(list(groups), p.get('planning_groups') or []),
        'base_frames': _unique(p.get('base_frame'), [g['base'] for g in groups.values()]),
        'tip_frames': _unique(p.get('flange_frame'), p.get('default_tool_frame'), p.get('tool_frames') or [],
                              (groups.get(group) or {}).get('tip')),
        'default_planning_group': group, 'default_base': p.get('base_frame'),
        'default_tip': p.get('flange_frame') or p.get('default_tool_frame')}


def _tool(capability_id, record, strategies):
    p = record.payload
    interface = p.get('interface') or {}
    model = ROOT / p['urdf_or_xacro'] if p.get('urdf_or_xacro') else None
    package = _package(p.get('asset_package'))
    installed = (model is not None and model.is_file()) or package is not None
    preview = 'preview_only' in (p.get('support_status'), p.get('runtime_status'))
    attach = _tool_root_link(model)
    return {
        'id': capability_id, 'label': p.get('display_name') or p.get('label') or capability_id,
        'family': p.get('family') or 'unknown', 'source': str(record.path.relative_to(ROOT)),
        'installed': installed, 'missing': [] if installed else ['tool model (urdf_or_xacro / asset_package)'],
        'preview_only': preview, **_status(p),
        'model': str(model.relative_to(ROOT)) if model is not None and model.is_file() else
                 (str(package.relative_to(ROOT)) if package else None),
        'compatible_robot_families': p.get('compatible_robot_families') or [],
        'grasp_capabilities': interface.get('grasp_capabilities') or [], 'interface': interface.get('kind'),
        'attach_links': _unique(attach), 'default_attach': attach, 'model_links': _model_links(model),
        'tcp_frames': _unique(interface.get('tcp_link'), p.get('tcp_frame'), p.get('grasp_frames') or []),
        'default_tcp': interface.get('tcp_link') or p.get('tcp_frame'),
        'type': p.get('family') or 'unknown',
        'release': {'joint_position': 'open_gripper', 'io_signal': 'vacuum_off'}.get(interface.get('kind')),
        'grasp_strategies': [s for s, tools in strategies.items() if capability_id in tools]}


def _profile_equipment(profile_path):
    profile_id = profile_path.stem
    profile, source = load_profile(profile_id)
    robot, tool, runtime = source['robot'], source['end_effector'], profile.get('runtime') or {}
    return {'id': profile_id, 'label': profile.get('label') or profile_id, 'robot': profile['robot_capability'],
            'tool': profile['tool_capability'], 'source': str(profile_path.relative_to(ROOT)),
            'scenarios': ['static_table_pick_place', *(profile.get('scenarios') or {})],
            'runtime': bool(runtime), 'arm_controller': runtime.get('arm_controller'),
            'tool_controller': runtime.get('tool_controller'),
            'defaults': {'base': robot.get('base_frame'), 'tip': robot.get('tool_mount_link'),
                         'planning_group': robot.get('planning_group'), 'tcp': tool.get('grasp_frame'),
                         'type': tool.get('type'), 'mount_pose_xyz': tool.get('mount_pose_xyz'),
                         'mount_pose_rpy': tool.get('mount_pose_rpy')}}


def _combination(robot, tool, profiles):
    """How far a robot + tool pair can go in this workspace. Catalog presence never implies runtime readiness:
    only a reviewed New Cell profile with MoveIt/controller bindings makes a pair plan-capable (fake hardware)."""
    pair = f"{robot['label']} + {tool['label']}"
    for item in (robot, tool):
        if item['preview_only']:
            return 'UNSUPPORTED', f"{item['label']} is preview-only in its capability record ({item['source']})."
    if robot['family'] not in tool['compatible_robot_families']:
        declared = ', '.join(tool['compatible_robot_families']) or 'none'
        return 'UNSUPPORTED', (f"{tool['label']} does not declare {robot['family']} robots compatible "
                               f"(compatible_robot_families: {declared}; {tool['source']}).")
    for item in (robot, tool):
        if not item['installed']:
            return 'UNCOMMISSIONED', (f"{item['label']} is in the catalog but not installed in this workspace: "
                                      f"missing {', '.join(item['missing'])}.")
    reviewed = [p for p in profiles if p['robot'] == robot['id'] and p['tool'] == tool['id'] and p['runtime']]
    if reviewed:
        return 'PLAN_CAPABLE', (f"{pair}: reviewed profile {reviewed[0]['id']} binds its MoveIt and controller "
                                f"configuration. Generate, Validate and fake-hardware Plan Only are available.")
    return 'CONFIGURABLE', (f"{pair}: both models are installed and compatible, but no reviewed New Cell profile "
                            f"(catalog/workcell_studio_profiles) binds their MoveIt and controller configuration. "
                            f"The cell records this equipment for authoring only; Generate and planning stay blocked.")


def equipment():
    """The New Cell equipment choices: capability records joined with the installed models and reviewed profiles."""
    records = _capabilities()
    strategies = {}
    for path in sorted((ROOT / 'catalog/grasp_strategies').glob('*.yaml')):
        strategy = (yaml.safe_load(path.read_text()) or {}).get('grasp_strategy') or {}
        strategies[strategy.get('id') or path.stem] = strategy.get('compatible_end_effector_capabilities') or []
    robots = [_robot(cid, record) for cid, record in records['robot'].items()]
    tools = [_tool(cid, record, strategies) for cid, record in records['end_effector'].items()]
    profiles, notes = [], []
    for path in sorted(PROFILES.glob('*.yaml')):
        try:
            profiles.append(_profile_equipment(path))
        except (ValueError, KeyError, OSError, yaml.YAMLError) as exc:
            notes.append(f'{path.name}: not loaded ({exc})')
    # Installed model packages no capability record describes: shown, never selectable as supported.
    described = {r.get('moveit_config') for r in robots} | {t.get('model') for t in tools}
    described |= {str(Path(t['model']).parent) for t in tools if t.get('model')}
    for kind, pattern in (('robot', 'assets/robots/*/*_moveit_config'), ('tool', 'assets/end_effectors/*/*_description')):
        for package in sorted(ROOT.glob(pattern)):
            rel = str(package.relative_to(ROOT))
            if (package / 'package.xml').is_file() and not any(d and (d == rel or d.startswith(rel + '/')) for d in described):
                notes.append(f'{kind} model {rel} has no capability record (catalog/capabilities); it cannot be selected.')
    combinations = []
    for robot in robots:
        for tool in tools:
            status, reason = _combination(robot, tool, profiles)
            profile = next((p for p in profiles if p['robot'] == robot['id'] and p['tool'] == tool['id']
                            and status == 'PLAN_CAPABLE'), None)
            pair = {'robot': robot['id'], 'tool': tool['id'], 'status': status, 'reason': reason,
                    'profile': profile['id'] if profile else None}
            if status == 'CONFIGURABLE':
                # Authoring-only 3D model: the installed geometry as an existing supported scene mounts it.
                pair['assembly'] = _reference_assembly(robot, tool)
                if pair['assembly'] is None:
                    pair['assembly_reason'] = (
                        f"No supported scene ({SUPPORTED_SCENES.name}) mounts {tool['label']} on {robot['label']}, so "
                        f"there is no validated tool mount; the cell opens without equipment geometry.")
            combinations.append(pair)
    return {'schema': 'new_cell_equipment/v1', 'robots': robots, 'tools': tools, 'profiles': profiles,
            'combinations': combinations, 'notes': notes, 'grasp_strategies': list(strategies)}


def describe(profile_id, scenario=None):
    """The wizard rows of the profile; a scenario with a reviewed layout (``scenarios``) moves and adds rows."""
    profile, source = load_profile(profile_id)
    layout = (profile.get('scenarios') or {}).get(scenario) or {}
    physical = source['environment']
    rows = []
    for key in ('support_surfaces', 'assets', 'task_zones'):
        for item in physical.get(key, []):
            if key == 'task_zones' and item['id'] != profile['pick_zone']:
                continue
            role = ('pick_source' if item['id'] == profile['pick_zone'] else
                    'place_target' if item['id'] == profile['place_asset'] else item.get('role', ''))
            pose = layout.get('poses', {}).get(item['id']) or dict(xyz=item['pose_xyz'], rpy=item['pose_rpy'])
            rows.append(dict(id=item['id'], asset_type=item['type'], role=role, pose=dict(pose)))
    for copy_id, spec in layout.get('copies', {}).items():
        original = next(row for row in rows if row['id'] == spec['of'])
        rows.append(dict(original, id=copy_id, pose=dict(spec['pose'])))
    return dict(profile, rows=rows, robot=source['robot'], tool=source['tool'], scenario=layout)


def restore_authored_bin_roles(environment):
    """Older Builder Applies saved the hierarchy display alias 'place_target_bin' as a bin's authored role
    (semantic_role_edit.hpp); a profile source still carrying it keeps its authored meaning, the target bin the
    physics world and destinations bind to."""
    for asset in environment.get('assets', []):
        if asset.get('type') == 'target_bin' and asset.get('role') == 'place_target_bin':
            asset['role'] = 'target_bin'


def apply_scenario_layout(environment, layout):
    """Move the reviewed scenario poses and add the copied destinations, each with its own placement region."""
    from physical_destination import resolve_destination
    assets = {a['id']: a for a in environment['assets']}
    for asset_id, pose in layout.get('poses', {}).items():
        assets[asset_id]['pose_xyz'], assets[asset_id]['pose_rpy'] = list(pose['xyz']), list(pose['rpy'])
    for copy_id, spec in layout.get('copies', {}).items():
        original = assets[spec['of']]
        target = copy.deepcopy(original)
        target.update(id=copy_id, layout_item_ref=copy_id, display_name=spec['display_name'], task_zone_ref=spec['region'],
                      pose_xyz=list(spec['pose']['xyz']), pose_rpy=list(spec['pose']['rpy']))
        zone = copy.deepcopy(next(z for z in environment['task_zones'] if z['id'] == original['task_zone_ref']))
        zone.update(id=spec['region'], layout_item_ref=spec['region'], target_ref=copy_id,
                    display_name=spec['region_display_name'])
        environment['assets'].append(target)
        environment['task_zones'].append(zone)
    # Every destination region follows its (moved or copied) target from its target-local placement.
    for zone in environment['task_zones']:
        if zone.get('target_ref') and zone.get('placement_local'):
            resolved = resolve_destination(environment, zone['id'], check_projection=False)
            zone['pose_xyz'], zone['pose_rpy'] = resolved['pose_xyz'], resolved['pose_rpy']
            resolve_destination(environment, zone['id'])


def scenario_test_objects(scene, layout, rows):
    """The wizard's test objects as qualified physics bodies, each with its authored appearance (render only)."""
    body = layout['test_object']
    size = [float(v) for v in body['dimensions']]
    objects = []
    for row in rows:
        appearance_id = row.get('appearance')
        appearance = layout['appearances'].get(appearance_id)
        if appearance is None:
            raise ValueError(f"test object {row.get('object_id')!r}: unknown appearance {appearance_id!r}")
        pose = row['pose']
        objects.append({'object_id': row['object_id'], 'class_id': 'unclassified', 'appearance': appearance_id,
                        'material': body['material'], 'shape': 'box', 'dimensions': size,
                        'pose': {'frame_id': 'world', 'xyz': [float(pose['xyz'][0]), float(pose['xyz'][1]), body['rest_z']],
                                 'rpy': [0.0, 0.0, float(pose['rpy'][2])]}})
        if appearance.get('mesh'):
            # The visual mesh scaled per axis to fill the body, its base on the body's base.
            mesh = ROOT / appearance['mesh']
            shutil.copytree(mesh.parent, scene / 'assets/visuals' / mesh.parent.name, dirs_exist_ok=True)
            bbox, centre = appearance['bbox'], appearance['centre_xy']
            scale = [s / b for s, b in zip(size, bbox)]
            objects[-1]['visual'] = {
                'mesh': f'assets/visuals/{mesh.parent.name}/{mesh.name}', 'scale': scale,
                'offset_xyz': [-centre[0] * scale[0], -centre[1] * scale[1], -size[2] / 2 - appearance['zmin'] * scale[2]],
                'source': appearance['source'], 'license': appearance['license']}
    if not objects or len({o['object_id'] for o in objects}) != len(objects):
        raise ValueError('Sorting needs at least one test object, each with a unique ID')
    return objects


def apply_destination_regions(environment, regions):
    """The profile's measured physical destination regions (target-local) for a new cell."""
    if not regions:
        return
    from physical_destination import resolve_destination, apply, rotation
    target = next(a for a in environment['assets'] if a['id'] == regions['asset'])
    target['usable_placement'].update(pose_xyz=list(regions['usable_placement']['pose_xyz']),
                                      dimensions=list(regions['usable_placement']['dimensions']))
    target['usable_placement']['review'] = ('Measured container floor (target-local z %s); footprint as reviewed, '
                                            'extended to the floor (profile destination_regions).'
                                            % regions['measured_floor_top_local_z'])
    for zone_id, region in regions['zones'].items():
        zone = next(z for z in environment['task_zones'] if z['id'] == zone_id)
        local = zone['placement_local']
        local.update(pose_xyz=list(region['pose_xyz']), dimensions=list(region['dimensions']))
        offset = apply(rotation(target['pose_rpy']), local['pose_xyz'])
        zone['pose_xyz'] = [a + b for a, b in zip(target['pose_xyz'], offset)]
        zone['dimensions'] = list(region['dimensions'])
        resolve_destination(environment, zone_id)          # projection and containment stay consistent


def materialize(scene, profile_id, destination):
    from authored_yaml import write_preserving
    from task_intent_authoring import load_authoring
    from export_builder_scene_to_cell_definition import export_scene
    import generate_workcell_from_cell_definition as generator
    from ensure_workcell_studio_web_scene_fresh_impl import require_real_xacro_mesh_index
    import subprocess

    if destination.exists():
        raise ValueError('New Cell destination already exists; refusing to replace it')
    profile, defaults = load_profile(profile_id)
    authored = yaml.safe_load((scene / 'environment.yaml').read_text())
    studio = authored['workcell_studio']
    studio['recommended_profile'] = profile_id
    if (studio['robot'].get('capability') != profile['robot_capability'] or
            studio['tool'].get('capability') != profile['tool_capability']):
        raise ValueError('Recommended physical profile does not match selected robot/tool')
    scenario_id = studio.get('scenario', {}).get('id')
    layout = (profile.get('scenarios') or {}).get(scenario_id) or {}
    expected = {row['id']: row for row in describe(profile_id, scenario_id)['rows']}
    selected = {row['id']: row for row in studio['environment_objects']}
    if selected.keys() != expected.keys():
        raise ValueError('Recommended profile assets changed; reselect Use Recommended Layout')
    # Do not silently ignore custom attachment/pose edits or invent their physical meaning.
    for name, row in selected.items():
        supported_fields = {'asset_type': expected[name]['asset_type'],
                            'semantic_role': expected[name]['role'], 'parent_object': 'world',
                            'parent_link': 'world', 'child_link': name, 'joint_type': 'fixed'}
        if any(row.get(key) != value for key, value in supported_fields.items()) or any(
                len(row['pose'][axis]) != 3 for axis in ('xyz', 'rpy')) or any(
                abs(a-b) > 1e-6 for axis in ('xyz', 'rpy')
                for a, b in zip(row['pose'][axis], expected[name]['pose'][axis])):
            raise ValueError(f'Recommended asset {name} differs from its reviewed pose/asset binding; edit it in Studio after creation')
    for key in ('robot', 'tool', 'end_effector', 'environment', 'camera', 'safety'):
        authored[key] = copy.deepcopy(defaults[key])
    restore_authored_bin_roles(authored['environment'])
    authored['schema_version'] = 'workcell_scene/v1'
    # Keep the display name and description the user typed in the wizard.
    wizard_scene = authored.get('scene') if isinstance(authored.get('scene'), dict) else {}
    authored['scene'] = {'id': scene.name, 'name': wizard_scene.get('display_name') or scene.name}
    authored['scene'].update({key: wizard_scene[key] for key in ('display_name', 'description') if wizard_scene.get(key)})
    authored['scaffold_only'] = False
    robot, tool = authored['robot'], authored['tool']
    frames = studio['frames']
    robot['pose_xyz'], robot['pose_rpy'] = frames['robot_mount_pose']['xyz'], frames['robot_mount_pose']['rpy']
    robot['base_frame'] = studio['robot']['base']
    robot['tool_mount_link'] = studio['robot']['tip']
    robot['planning_group'] = studio['robot']['planning_group']
    tool['mount_link'] = studio['robot']['tip']
    tool['grasp_frame'] = studio['tool']['tcp']
    tool['mount_pose_xyz'], tool['mount_pose_rpy'] = frames['tool_mount_pose']['xyz'], frames['tool_mount_pose']['rpy']
    authored['end_effector'] = copy.deepcopy(tool)
    # The reviewed tool's catalog capability (and with it the hold profile the qualified
    # simulator binds) belongs to the selected tool model; keep it rather than drop it.
    if defaults['end_effector'].get('capability'):
        authored['end_effector']['capability'] = defaults['end_effector']['capability']
    apply_destination_regions(authored['environment'], profile.get('destination_regions'))
    apply_scenario_layout(authored['environment'], layout)
    camera = authored['camera']
    physical_camera = next(item for item in authored['environment']['assets'] if item['id'] == camera['camera_id'])
    camera['pose'] = physical_camera['pose_xyz'] + physical_camera['pose_rpy']
    source = studio['pick_zone']['object_source']
    # Camera/adapter binding metadata belongs to the physical equipment profile,
    # even while perception is off. Later task-mode changes reuse these reviewed
    # bindings rather than inventing an EPD topic or copying a reference task.
    perception = copy.deepcopy(defaults['perception'])
    perception['enabled'] = source['mode'] != 'manual_simulated'
    perception['mode'] = {'manual_simulated': 'off', 'recorded_perception': 'replay'}.get(source['mode'], 'live')
    if source['mode'] != 'manual_simulated':
        perception['epd_input']['topic'] = source['perception_binding']
    perception['required_object_classes'] = [source['required_class']] if source.get('required_class') else []
    perception['confidence_threshold'] = source.get('minimum_confidence')
    authored['perception'] = perception
    # TaskIntent is authored from this wizard, never from the profile source task.
    studio['place_zone']['region'] = profile['place_region']
    if studio['place_zone']['target'] != profile['place_asset']:
        raise ValueError('Select the recommended physical destination bin')
    if studio['pick_zone']['object_source']['source_zone_ref'] != profile['pick_zone']:
        raise ValueError('Select the recommended pick region')
    if layout and source['mode'] == 'manual_simulated':
        source['manual_observations'] = scenario_test_objects(scene, layout, source.get('manual_observations') or [])
    write_preserving(scene / 'environment.yaml', authored)
    if layout:
        # A sorting cell runs the reviewed Pick & Place cycle once per object; the Task Editor's class routing table
        # chooses each object's destination. The wizard's mapping of that cycle is reused unchanged.
        from task_intent_authoring import _wizard_intent
        cycle = dict(studio, scenario=dict(studio['scenario'], id='static_table_pick_place'))
        draft = _wizard_intent(scene, {**authored['environment'], 'workcell_studio': cycle,
                                       'end_effector': authored['end_effector']})
        intent = load_authoring(scene, draft=draft)['task_intent']
    else:
        intent = load_authoring(scene)['task_intent']
    intent['scene_package'] = str(destination)
    (scene / 'config/workcell_builder_task_intent.yaml').write_text(json.dumps(intent, indent=2) + '\n')

    # Project the authoritative physical objects into the existing editable layout.
    items = []
    for key in ('support_surfaces', 'assets', 'task_zones'):
        for raw in authored['environment'].get(key, []):
            item = copy.deepcopy(raw)
            item['id'] = item.get('layout_item_ref', item['id'])
            if any(previous['id'] == item['id'] for previous in items):
                continue
            item['pose'] = {'xyz': item.pop('pose_xyz'), 'rpy': item.pop('pose_rpy')}
            item.update(editable=True, locked=False, source='environment.yaml', source_layer='editable_layout')
            items.append(item)
    (scene / 'layout').mkdir(exist_ok=True)
    write_preserving(scene / 'layout/workcell_studio_layout.yaml', {
        'schema_version': 'workcell_studio_layout/v1', 'scene_name': scene.name,
        'scene_path': '.', 'metadata': {'physical_state_source': 'environment.yaml'}, 'items': items})

    # Reuse the reviewed equipment xacro as a profile template. All scene-dependent
    # arguments read this cell's authored values; there is no reference-scene lookup.
    ET.register_namespace('xacro', XACRO[1:-1])
    model = ET.fromstring((ROOT / profile['model_source']).read_text())
    model.set('name', scene.name)
    for arg in model.findall(XACRO + 'arg'):
        if arg.get('name') == 'environment_file':
            arg.set('default', '../environment.yaml')
        elif arg.get('name') == 'tool_mount_link':
            arg.set('default', tool['mount_link'])
        elif arg.get('name') == 'grasp_frame':
            arg.set('default', tool['grasp_frame'])
    origins = profile['model_origins']
    # Profile metadata identifies the existing macro fields, not a second URDF renderer.
    for macro, path in origins.items():
        model_origins = model.findall(XACRO + macro + '/origin')
        if len(model_origins) != 1:
            raise ValueError(f'Reviewed model must define exactly one {macro} origin')
        for origin in model_origins:
            for axis, selection in (('xyz', '[:3]'), ('rpy', '[3:]')):
                expression = path.format(axis=axis, slice=selection)
                origin.set(axis, "${' '.join([str(v) for v in workcell_environment" + expression + "])}")
    (scene / 'urdf').mkdir(exist_ok=True)
    (scene / 'urdf/scene.urdf.xacro').write_bytes(ET.tostring(model, encoding='utf-8', xml_declaration=True))
    # Full initial publication uses the same generator as later in-place refresh.
    # Its initial model is this cell's authored equipment definition, never a
    # reference scene's generated package. In-place refresh cannot create missing
    # curated package files, so it is deliberately not used for initialization.
    export_scene(scene, scene, validate=True)
    with tempfile.TemporaryDirectory(prefix='.initial-package-', dir=scene.parent) as output:
        result = generator.generate_package(scene / 'cell_definition.yaml', Path(output),
                                            scene.name, False, False, authored_model_dir=scene, published_package_dir=destination)
        if result:
            raise ValueError(f'Initial scene package generation failed ({result})')
        shutil.copytree(Path(output) / scene.name, scene, dirs_exist_ok=True)
    result = subprocess.run([sys.executable, str(ROOT / 'scripts/extract_scene_urdf_visual_mesh_index.py'),
                             '--scene', str(scene), '--prefer-xacro', '--require-xacro'], capture_output=True, text=True)
    if result.returncode:
        raise ValueError(result.stderr or result.stdout or 'Required visual extraction failed')
    require_real_xacro_mesh_index(scene / 'generated/scene_visual_mesh_index.json')
    (scene / 'config/workcell_builder_task_intent.yaml').write_text(json.dumps(intent, indent=2) + '\n')


def assemble_authoring_model(scene):
    """Give a CONFIGURABLE New Cell its 3D equipment model, for layout authoring only.

    The installed robot and tool macros are instantiated exactly as the discovered reference assembly does; the
    robot and tool mounts read this cell's environment.yaml. Nothing here binds MoveIt, controllers or a task, and
    the export gate still refuses the cell. Any frame or expansion problem fails closed with its reason.
    """
    from authored_yaml import write_preserving
    from ensure_workcell_studio_web_scene_fresh_impl import require_real_xacro_mesh_index
    import subprocess

    authored = yaml.safe_load((scene / 'environment.yaml').read_text())
    studio = authored['workcell_studio']
    robot_id, tool_id = studio['robot']['capability'], studio['tool']['capability']
    current = equipment()
    pair = next((c for c in current['combinations'] if c['robot'] == robot_id and c['tool'] == tool_id), None)
    if pair is None or pair['status'] != 'CONFIGURABLE':
        raise ValueError(f"{robot_id} + {tool_id} is {pair['status'] if pair else 'unknown'}, not CONFIGURABLE; "
                         f"only configurable equipment receives an authoring-only model.")
    assembly = pair.get('assembly')
    if not assembly:
        raise ValueError(pair['assembly_reason'])
    if studio['robot']['tip'] != assembly['tool_parent']:
        raise ValueError(f"The tool mount of {assembly['scene']} is relative to {assembly['tool_parent']}; "
                         f"select that robot tip (selected: {studio['robot']['tip']}).")
    tool = next(t for t in current['tools'] if t['id'] == tool_id)
    robot = next(r for r in current['robots'] if r['id'] == robot_id)
    found = _reference_calls(ROOT / assembly['source'], robot, tool)
    if found is None:
        raise ValueError(f"Reference assembly {assembly['source']} changed; reopen New Cell.")
    reference, robot_call, tool_call, includes = found

    ns = robot_call.tag.rpartition('}')[0][1:]
    ET.register_namespace('xacro', ns)
    tag = lambda name: f'{{{ns}}}{name}'
    model = ET.Element('robot', {'name': scene.name})
    calls = [copy.deepcopy(robot_call), copy.deepcopy(tool_call)]
    # Only the reference's arguments these calls use (transitively); never its environment or task.
    args = {e.get('name'): e for e in reference if _xacro_name(e) == 'arg'}
    used, pending = [], ET.tostring(robot_call, encoding='unicode') + ET.tostring(tool_call, encoding='unicode')
    while True:
        names = [n for n in re.findall(r'\$\(arg ([\w-]+)\)', pending) if n in args and n not in used]
        if not names:
            break
        used += names
        pending = ' '.join(args[n].get('default') or '' for n in names)
    for name in args:
        if name in used:
            model.append(copy.deepcopy(args[name]))
    ET.SubElement(model, tag('property'), {'name': 'workcell_environment',
                                            'value': "${xacro.load_yaml('../environment.yaml')}"})
    world = robot_call.get('parent')
    if not any(e.tag == 'link' and e.get('name') == world for e in reference):
        raise ValueError(f"Reference assembly {assembly['source']} does not define the robot parent link {world}.")
    ET.SubElement(model, 'link', {'name': world})
    frames = "workcell_environment['workcell_studio']['frames']"
    for call, pose in zip(calls, ('robot_mount_pose', 'tool_mount_pose')):
        model.append(copy.deepcopy(includes[_xacro_name(call)][0]))
        origin = call.find('origin')
        for axis in ('xyz', 'rpy'):
            origin.set(axis, "${' '.join([str(v) for v in " + f"{frames}['{pose}']['{axis}']" + "])}")
        model.append(call)
    (scene / 'urdf').mkdir(exist_ok=True)
    (scene / 'urdf/scene.urdf.xacro').write_bytes(ET.tostring(model, encoding='utf-8', xml_declaration=True))

    result = subprocess.run([sys.executable, str(ROOT / 'scripts/extract_scene_urdf_visual_mesh_index.py'),
                             '--scene', str(scene), '--prefer-xacro', '--require-xacro'], capture_output=True, text=True)
    if result.returncode:
        raise ValueError(result.stderr or result.stdout or 'Required visual extraction failed')
    require_real_xacro_mesh_index(scene / 'generated/scene_visual_mesh_index.json')
    expanded = scene / 'generated/expanded_scene_preview.urdf'
    links = {link.get('name') for link in ET.parse(expanded).getroot().iter('link')} if expanded.is_file() else set()
    for role, frame in (('robot base', studio['robot']['base']), ('robot tip', studio['robot']['tip']),
                        ('tool attach link', studio['tool']['attach'])):
        if frame not in links:
            raise ValueError(f"The assembled model has no {role} {frame}; the cell was not given a 3D model.")
    tcp = studio['tool']['tcp']
    warnings = [] if tcp in links else [
        f"TCP {tcp} is declared for {tool['label']} ({tool['source']}) but no installed model defines it, so the 3D "
        f"view cannot place it. It must be defined and measured before this tool is commissioned."]
    studio['authoring_model'] = {
        'status': 'ASSEMBLED', 'purpose': 'layout_authoring_only', 'reference_scene': assembly['scene'],
        'reference_source': assembly['source'], 'robot_macro': assembly['robot_macro'],
        'tool_macro': assembly['tool_macro'], 'tool_parent': assembly['tool_parent'],
        'tool_mount': assembly['tool_mount'], 'attach_link': studio['tool']['attach'], 'tcp': tcp,
        'tcp_defined': tcp in links, 'warnings': warnings}
    write_preserving(scene / 'environment.yaml', authored)
    return studio['authoring_model']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile')
    parser.add_argument('--equipment', action='store_true',
                        help='the New Cell robot/tool choices, their frames and readiness (JSON)')
    parser.add_argument('--describe', action='store_true')
    parser.add_argument('--authoring-model', action='store_true',
                        help='with --scene: assemble the 3D model of a CONFIGURABLE cell (authoring only)')
    parser.add_argument('--scenario', help='with --describe: the New Cell scenario whose reviewed layout applies')
    parser.add_argument('--scene', type=Path)
    parser.add_argument('--destination', type=Path)
    args = parser.parse_args()
    try:
        if args.equipment:
            print(json.dumps(equipment()))
        elif args.authoring_model:
            print(json.dumps(assemble_authoring_model(args.scene)))
        elif args.describe:
            print(json.dumps(describe(args.profile, args.scenario)))
        else:
            materialize(args.scene, args.profile, args.destination)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
