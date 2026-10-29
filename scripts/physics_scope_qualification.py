#!/usr/bin/env python3
"""``workcell_physics_scope/v1``: the bound scope that admits motion in a multi-destination physics world.

A two-bin world (``simulator_world.derive_world(physical_destinations='all')``) is plan-only by default: the executor
refuses motion (``MULTI_DESTINATION_PHYSICS_UNQUALIFIED``) and the contact guard binds no destination floor. A scope
record names the one destination bin whose floor the placed part may touch and binds everything that trial was
authorised for. Every binding is recomputed from the current files by whoever verifies it; any difference, missing
field or unsupported state refuses.

Four states are kept apart:
- a valid configuration (derive_world and the launch bind every bin): no motion;
- a planned trajectory (plan-only resolve and revalidate in the session): no motion;
- ``AUTHORISED_TRIAL``: written by the CLI qualification runner only, after this session's plan-only revalidation,
  for one guarded full-cycle into one bound bin, gated by the same-build Stage-A prerequisite trials
  (``require_trial_evidence``) and the unchanged contact, retention, placement and release guards. It needs no
  earlier success, and its outcome is labelled ``PHYSICS_SCOPE_TRIAL_NOT_QUALIFIED``;
- a qualified scope (a record binding a strict trial PASS): not produced or accepted here yet.

The record is single-session (it binds the session receipt) and immutable: the executor verifies its digest before
motion and again before reporting a physical result.
"""
import hashlib
import json
import time
from pathlib import Path

import yaml

SCHEMA = 'workcell_physics_scope/v1'
AUTHORISED_TRIAL = 'AUTHORISED_TRIAL'
TRIAL_EVIDENCE_SCOPE = 'PHYSICS_SCOPE_TRIAL_NOT_QUALIFIED'
# What one authorised trial may do. Fixed in code: a record naming anything else is refused.
TRIAL_OPERATION = {'gate': 'full-cycle', 'cycles': 1, 'objects': 1, 'destinations': 1, 'class_routing': False,
                   'continuous_sorting': False, 'real_hardware': False}
# The authored cell and its generated handoff, robot description and controllers (relative to the cell package).
CELL_FILES = ('environment.yaml', 'layout/workcell_studio_layout.yaml', 'config/workcell_builder_task_intent.yaml',
              'cell_definition.yaml', 'config/moveit_collision_objects.yaml', 'config/ros2_controllers.yaml',
              'urdf/scene.urdf.xacro', 'urdf/arm_hand.srdf.xacro')
# The launched robot, gripper hold profile and controllers, as the simulator receipt records them.
ROBOT_RECEIPT_KEYS = ('model', 'description_sha256', 'native_description_sha256', 'native_mimics', 'hardware_class',
                      'control_plugin_name', 'control_plugin_sha256', 'controllers_sha256', 'expected_controllers',
                      'end_effector_hold', 'telemetry_sha256')


class PhysicsScopeRefused(RuntimeError):
    """The requested motion is not covered by a verified physics scope."""


def _sha256(data):
    return hashlib.sha256(data).hexdigest()


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':')).encode()


def _portable(record):
    """A receipt binding without its session-local file paths (their content is bound by sha)."""
    return {k: v for k, v in record.items() if not k.endswith('_path')}


def _required(value, name):
    if value in (None, '', [], {}):
        raise PhysicsScopeRefused(f'PHYSICS_SCOPE_INCOMPLETE: {name} is missing')
    return value


def native_identity(manifest_path):
    """The commissioning build manifest, with every library and bound source proved against the files on disk."""
    manifest = json.loads(Path(manifest_path).read_text())
    for library, key in (('library', 'sha256'), ('support_library', 'support_sha256'),
                         ('telemetry_library', 'telemetry_sha256')):
        path = Path(_required(manifest.get(library), library))
        if not path.is_file() or _sha256(path.read_bytes()) != _required(manifest.get(key), key):
            raise PhysicsScopeRefused(f'PHYSICS_SCOPE_NATIVE_MISMATCH: {library} differs from its build manifest')
    sources = _required(manifest.get('sources'), 'bound sources')
    for path, sha in sources.items():
        if not Path(path).is_file() or _sha256(Path(path).read_bytes()) != sha:
            raise PhysicsScopeRefused(f'PHYSICS_SCOPE_NATIVE_MISMATCH: bound source changed since build: {path}')
    return {key: manifest[key] for key in ('sha256', 'support_sha256', 'telemetry_sha256', 'moveit_version', 'sources')}


def runtime_identity(workspace):
    """The registered qualified runtime (re-validated) and the Stage-A acceptance it was registered from."""
    from qualified_physics_runtime import RECORD_NAME, RuntimeUnavailable, load
    try:
        record = load(workspace)
    except RuntimeUnavailable as exc:
        raise PhysicsScopeRefused(f'PHYSICS_SCOPE_RUNTIME_UNQUALIFIED: {exc}') from exc
    acceptance = record['stage_a_acceptance']
    return dict(record_sha256=_sha256((Path(workspace) / RECORD_NAME).read_bytes()),
                stage_a_evidence=str(Path(acceptance['evidence']).resolve()),
                stage_a_report_sha256=acceptance['report_sha256'], stage_a_repo_head=acceptance['repo_head'])


def cell_world(cell_dir):
    """The cell's derived multi-destination world: SDF text and what the scope binds of it."""
    from simulator_world import SimulatorWorldError, derive_world
    try:
        text, report = derive_world(cell_dir, physical_destinations='all')
    except SimulatorWorldError as exc:
        raise PhysicsScopeRefused(f'PHYSICS_SCOPE_UNSUPPORTED: {exc}') from exc
    if report.get('physical_destinations') != 'all':
        raise PhysicsScopeRefused('PHYSICS_SCOPE_UNSUPPORTED: the cell derives a single-destination world')
    keys = ('object_id', 'class_id', 'material', 'dimensions_m', 'spawn_pose', 'mass_kg', 'within_qualified_dimensions')
    return text, dict(derived_sha256=report['world_sha256'], destination_asset=report['destination']['asset_ref'],
                      destination_region=report['destination']['region_ref'],
                      destination_assets=sorted(d['asset_ref'] for d in report['destinations']),
                      bin_models=sorted(d['model'] for d in report['destinations']),
                      objects=[{k: o[k] for k in keys} for o in report['objects']])


def _models(text, names):
    import xml.etree.ElementTree as ET
    world = ET.fromstring(text).find('world')
    return {m.get('name'): ET.tostring(m) for m in world.findall('model') if m.get('name') in names}


def destination_occupants(observations, binding):
    """Measured objects whose centre lies over the bin's bound floor (the bin must be empty before a trial)."""
    centre, size = binding['floor_xyz'], binding['floor_size_m']
    return sorted(o['object_id'] for o in observations.get('objects', [])
                  if all(abs(o['pose']['xyz'][i] - centre[i]) <= size[i] / 2 for i in (0, 1)))


def scope_binding(*, cell_dir, receipt_path, observations_path, native, runtime, commission_evidence_path,
                  resolution_sha256):
    """Everything an authorised trial is bound to, recomputed from the current files. Refuses unsupported input."""
    from destination_binding import select_destination_binding
    cell_dir = Path(cell_dir)
    receipt_bytes = Path(receipt_path).read_bytes()
    receipt = json.loads(receipt_bytes)
    bindings = receipt.get('destination_geometry_bindings') or []
    if len(bindings) < 2 or receipt.get('destination_geometry_binding') is not None:
        raise PhysicsScopeRefused('PHYSICS_SCOPE_UNSUPPORTED: the receipt does not bind a multi-destination world')
    files = {}
    for name in CELL_FILES:
        path = cell_dir / name
        if not path.is_file():
            raise PhysicsScopeRefused(f'PHYSICS_SCOPE_INCOMPLETE: cell file {name} is missing')
        files[name] = _sha256(path.read_bytes())
    derived, world = cell_world(cell_dir)
    # The launched world adds session plugins (physics overlay, measurement run id); every physical bin and object
    # it simulates must be exactly the cell's derived model.
    launched_path = Path(receipt_path).parent / 'world.sdf'
    launched = launched_path.read_bytes() if launched_path.is_file() else b''
    if _sha256(launched) != receipt.get('world_sha256'):
        raise PhysicsScopeRefused('PHYSICS_SCOPE_WORLD_MISMATCH: the launched world file is not the receipt\'s world')
    names = set(world['bin_models']) | {o['object_id'] for o in world['objects']}
    expected = _models(derived, names)
    if len(expected) != len(names) or _models(launched.decode(), names) != expected:
        raise PhysicsScopeRefused('PHYSICS_SCOPE_WORLD_MISMATCH: the launched bins or objects differ from the cell\'s '
                                  'derived world')
    if not world['objects'] or not all(o['within_qualified_dimensions'] for o in world['objects']):
        raise PhysicsScopeRefused('PHYSICS_SCOPE_UNSUPPORTED: every physics object must have the qualified dimensions')
    if {b.get('source_item_id') for b in bindings} != set(world['destination_assets']):
        raise PhysicsScopeRefused('PHYSICS_SCOPE_WORLD_MISMATCH: bound bins differ from the derived world\'s bins')
    try:
        selected = select_destination_binding(bindings, world['destination_asset'])
    except ValueError as exc:
        raise PhysicsScopeRefused(f'PHYSICS_SCOPE_DESTINATION_UNBOUND: {exc}') from exc
    occupants = destination_occupants(yaml.safe_load(Path(observations_path).read_text()), selected)
    if occupants:
        raise PhysicsScopeRefused(f'PHYSICS_SCOPE_DESTINATION_OCCUPIED: {occupants}')
    support = receipt.get('support_geometry_binding')
    binding = dict(
        cell=dict(files=files),
        world=dict(name=_required(receipt.get('world'), 'world name'), derived_sha256=world['derived_sha256'],
                   bin_models=world['bin_models'], objects=world['objects']),
        destinations=sorted((_portable(b) for b in bindings), key=lambda b: b['model']),
        destination=dict(asset=world['destination_asset'], region=world['destination_region'],
                         object_id=selected['object_id'], model=selected['model'], floor_z=selected['floor_z']),
        support=_portable(_required(support, 'support binding')),
        robot={key: _required(receipt.get(key), key) for key in ROBOT_RECEIPT_KEYS},
        native=native, runtime=runtime,
        evidence=dict(commission_evidence_sha256=_sha256(Path(commission_evidence_path).read_bytes())),
        planning=dict(resolution_sha256=_required(resolution_sha256, 'plan resolution')),
        session=dict(receipt_sha256=_sha256(receipt_bytes), run_id=_required(receipt.get('run_id'), 'run id')),
        occupancy=dict(destination_occupants=occupants))
    for key in ('model', 'link', 'collision', 'object_id', 'floor_z', 'walls'):
        _required(selected.get(key), f'destination binding {key}')
    return binding


def authorise_trial(path, binding):
    """Write the single-session trial record; returns its sha256 (the executor is given both)."""
    record = dict(schema=SCHEMA, state=AUTHORISED_TRIAL, operation=TRIAL_OPERATION, binding=binding,
                  binding_sha256=_sha256(_canonical(binding)), created_wall_ns=time.time_ns())
    data = (json.dumps(record, indent=2, sort_keys=True) + '\n').encode()
    Path(path).write_bytes(data)
    return _sha256(data)


def verify_trial(path, sha256, binding):
    """The record at ``path`` if it is exactly the authorised trial for ``binding`` (recomputed now); else refuse."""
    data = Path(path).read_bytes()
    if not sha256 or _sha256(data) != sha256:
        raise PhysicsScopeRefused('PHYSICS_SCOPE_RECORD_CHANGED: the scope record differs from the authorised one')
    record = json.loads(data)
    if record.get('schema') != SCHEMA:
        raise PhysicsScopeRefused('PHYSICS_SCOPE_UNSUPPORTED: unknown scope schema')
    if record.get('state') != AUTHORISED_TRIAL:
        raise PhysicsScopeRefused(f"PHYSICS_SCOPE_UNSUPPORTED: scope state {record.get('state')!r}")
    if record.get('operation') != TRIAL_OPERATION:
        raise PhysicsScopeRefused('PHYSICS_SCOPE_UNSUPPORTED: the record authorises another operation')
    bound = record.get('binding') or {}
    if record.get('binding_sha256') != _sha256(_canonical(bound)):
        raise PhysicsScopeRefused('PHYSICS_SCOPE_RECORD_CHANGED: binding digest mismatch')
    differing = sorted(key for key in set(bound) | set(binding) if bound.get(key) != binding.get(key))
    if differing:
        raise PhysicsScopeRefused('PHYSICS_SCOPE_MISMATCH: ' + ', '.join(differing))
    return record
