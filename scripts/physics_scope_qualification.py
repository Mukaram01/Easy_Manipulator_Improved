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
G4 (continuous sorting) has its own states, G4_PLAN_ONLY and the CLI-only bounded G4_AUTHORISED_TRIAL (see below).

The record is single-session (it binds the session receipt) and immutable: the executor verifies its digest before
motion and again before reporting a physical result.
"""
import ast
import hashlib
import json
import subprocess
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


REPO = Path(__file__).resolve().parents[1]
# Software identity (G3C): the files that decide how a trial plans and moves. Python: every repo module the executor
# imports, directly or transitively (a test checks this list against the real import closure), except this judge,
# whose changes never reset a campaign. Configuration: the scene template launch (MoveIt pipeline, TOTG tolerance)
# and the capability catalog (tool hold, simulator collision boxes).
EXECUTOR_MODULES = (
    'asset_alignment', 'asset_arrangement', 'asset_mounting', 'build_simulator_physics_overlay', 'capability_registry',
    'closure_clearance', 'container_geometry', 'destination_binding', 'end_effector_hold', 'full_cycle_preplanner',
    'generate_moveit_collision_manifest', 'grasp_capture', 'grasp_strategy_candidates', 'measurement_clock',
    'perceived_object_grasp_execute', 'perceived_object_grasp_plan', 'perception_replay', 'physical_destination',
    'pile_extraction', 'qualified_physics_runtime', 'runtime_pick_inputs', 'simulation_batch_preview',
    'simulation_collision', 'simulator_backend', 'simulator_execution', 'simulator_observations', 'simulator_world',
    'task_intent_authoring', 'task_intent_resolver', 'task_intent_v2', 'tool_interface', 'transport_policy',
    'validate_builder_task_intent', 'workcell_studio_layout_source')
PLANNER_SOURCES = tuple(f'scripts/{m}.py' for m in EXECUTOR_MODULES) + ('scenes/ur5_2f_test/launch/demo.launch.py',
                                                                        'catalog/capabilities')
# G4 control identity: the code outside PLANNER_SOURCES that issues, admits, judges, counts and stops G4 trial cycles
# (this authority, the G4 runner, its ledger and the G5 runner primitives it reuses). A G4 trial binds it, so trials
# of different control code are never one software version; it never enters the G3C identity above.
G4_CONTROL_SOURCES = ('scripts/physics_scope_qualification.py', 'scripts/run_g4_sorting.py', 'scripts/sorting_session.py',
                      'scripts/run_stage_a1_finish.py')
# The stop/cancel paths G5 accepted, by function or class (AST): a G5 run of another software version stays admissible
# only while these are unchanged. The runner's main hosts the stop-at-cycle-end boundary, not the cancel path.
STOP_CANCEL_MECHANISM = {
    'cancellation': {
        'scripts/perceived_object_grasp_execute.py': ('interrupted', 'action', 'request_cancel', 'terminal',
                                                      'wait_stopped', 'stop_proves_hold', 'measured_reconcile',
                                                      'reconcile_retention_loss'),
        'scripts/simulator_execution.py': ('ContactGuard', 'ControllerCancellationAudit', 'CancellationMotion',
                                           'validate_controller_cancellation', 'transfer_cancellation_acceptance'),
        'scripts/run_stage_a1_finish.py': ('run', 'cancel_on_signal', 'OperatorCancel', 'one_session')},
    'stop_after_cycle': {
        'scripts/run_stage_a1_finish.py': ('run', 'cancel_on_signal', 'OperatorCancel', 'StopAfterCycle',
                                           'StageForwarder', 'executor_stage', 'one_session', 'main')},
}


def _git(repo, *args):
    done = subprocess.run(['git', '-C', str(repo), *args], capture_output=True, text=True)
    if done.returncode:
        raise PhysicsScopeRefused(f"PHYSICS_SCOPE_PROVENANCE_UNRECOVERABLE: git {' '.join(args[:2])}: "
                                  f"{done.stderr.strip()[:200]}")
    return done.stdout


def _source_identity(sources, what, repo, revision):
    if revision is None:
        if _git(repo, 'status', '--porcelain', '--', *sources).strip():
            raise PhysicsScopeRefused(f'PHYSICS_SCOPE_SOFTWARE_MODIFIED: {what} sources differ from HEAD')
        revision = 'HEAD'
    blobs = {}
    for line in _git(repo, 'ls-tree', '-r', revision, '--', *sources).splitlines():
        _, kind, blob, path = line.split(None, 3)
        if kind == 'blob':
            blobs[path] = blob
    if not blobs:
        raise PhysicsScopeRefused(f'PHYSICS_SCOPE_PROVENANCE_UNRECOVERABLE: no {what} sources at {revision}')
    return _sha256(_canonical(blobs))


def planner_identity(repo=REPO, revision=None):
    """The software identity of ``revision`` (default: this checkout's HEAD, whose PLANNER_SOURCES must be unmodified):
    sha256 over the git blob id of every file in PLANNER_SOURCES. Historical trials are recovered from their recorded
    clean commit, so no evidence file ever changes."""
    return _source_identity(PLANNER_SOURCES, 'planner or executor', repo, revision)


def g4_control_identity(repo=REPO, revision=None):
    """The G4 control identity of ``revision``, as planner_identity over G4_CONTROL_SOURCES."""
    return _source_identity(G4_CONTROL_SOURCES, 'G4 control', repo, revision)


def mechanism_identity(role, repo=REPO, revision=None):
    """AST hashes of the G5 ``role``'s stop/cancel functions and classes at ``revision`` (default: HEAD, whose files
    must be unmodified)."""
    if revision is None:
        if _git(repo, 'status', '--porcelain', '--', *STOP_CANCEL_MECHANISM[role]).strip():
            raise PhysicsScopeRefused('PHYSICS_SCOPE_SOFTWARE_MODIFIED: stop/cancel sources differ from HEAD')
        revision = 'HEAD'
    out = {}
    for path, names in STOP_CANCEL_MECHANISM[role].items():
        tree = ast.parse(_git(repo, 'show', f'{revision}:{path}'))
        found = {}
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names and node.name not in found:
                found[node.name] = hashlib.sha256(ast.dump(node).encode()).hexdigest()
        out[path] = {name: found.get(name) for name in names}
    return out


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


def _require_launched_world(receipt_path, receipt, derived, world, bindings):
    """The launched world adds session plugins (physics overlay, measurement run id); every physical bin and object it
    simulates must be exactly the cell's derived model, every object the qualified body, every bin bound."""
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


def destination_occupants(observations, binding):
    """Measured objects whose centre lies over the bin's bound floor (the bin must be empty before a trial)."""
    if any((o.get('pose') or {}).get('frame_id') != 'world' for o in observations.get('objects', [])):
        raise PhysicsScopeRefused('PHYSICS_SCOPE_UNSUPPORTED: measured object poses must be in the world frame')
    centre, size = binding['floor_xyz'], binding['floor_size_m']
    return sorted(o['object_id'] for o in observations.get('objects', [])
                  if all(abs(o['pose']['xyz'][i] - centre[i]) <= size[i] / 2 for i in (0, 1)))


def scope_binding(*, cell_dir, receipt_path, observations_path, native, runtime, commission_evidence_path,
                  resolution_sha256, planner=None):
    """Everything an authorised trial is bound to, recomputed from the current files. Refuses unsupported input.
    ``planner`` defaults to this checkout's software identity (planner_identity), recomputed by the executor too."""
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
    # A G3C trial is TRIAL_OPERATION: one object, one destination, no routing. A routed or multi-object cell derives
    # the plan-only multi-destination world (G4) and is never authorised here.
    from task_intent_v2 import class_routing_rules
    intent_path = cell_dir / 'config/workcell_builder_task_intent.yaml'
    intent = yaml.safe_load(intent_path.read_text()) if intent_path.is_file() else {}
    intent = intent.get('task_intent', intent) if isinstance(intent, dict) else {}
    if class_routing_rules(intent) is not None or len(world['objects']) != TRIAL_OPERATION['objects']:
        raise PhysicsScopeRefused('PHYSICS_SCOPE_UNSUPPORTED: a trial is one object into one destination, unrouted '
                                  f"(this cell routes classes or has {len(world['objects'])} objects)")
    _require_launched_world(receipt_path, receipt, derived, world, bindings)
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
        native=native, runtime=runtime, planner=planner or planner_identity(),
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


# --- G4: continuous physical sorting authority. Builds on G3C without widening it ---------------------------------
# A session record binds one exact routed multi-object cell; each cycle needs its own record, chained to the session
# and to the previous cycle's independently re-verified evidence. Three session states:
# - G4_PLAN_ONLY: every session the runner opens without --authorise-g4-trial. No cycle is ever authorised.
# - G4_AUTHORISED_TRIAL: written only by the CLI runner (run_g4_sorting.py --authorise-g4-trial, registered Stage-A
#   prerequisites), for at most G4_TRIAL_MAX_CYCLES guarded full-cycles in this one Gazebo session. It binds the
#   issuing runner process (pid + kernel start time + its command line, which must name the CLI trial): only that
#   live process authorises cycles, and only an executor it started may move. This is process lineage on one host, not
#   authentication. Every use recomputes the binding from the current files (software, G4 control code, native,
#   runtime, cell, receipt, perception, Stage-A evidence). Its evidence is labelled G4_TRIAL_NOT_QUALIFIED: a trial
#   never implies continuous-production qualification.
# - G4_AUTHORISED_SESSION (continuous production): needs a G4 acceptance verifier recomputing accepted evidence. None
#   exists; a record claiming that state is refused like any other claim (_g4_acceptance). See g4_qualification_readiness.
G4_SCHEMA = 'workcell_g4_sorting_scope/v1'
G4_PLAN_ONLY, G4_AUTHORISED_SESSION = 'G4_PLAN_ONLY', 'G4_AUTHORISED_SESSION'
G4_AUTHORISED_TRIAL, G4_TRIAL_CYCLE = 'G4_AUTHORISED_TRIAL', 'G4_AUTHORISED_TRIAL_CYCLE'
G4_TRIAL_MAX_CYCLES = 2
G4_TRIAL_EVIDENCE_SCOPE = 'G4_TRIAL_NOT_QUALIFIED'
G4_TRIAL_ISSUER = ('run_g4_sorting.py', '--authorise-g4-trial')
G4_OPERATION = {'gate': 'full-cycle', 'class_routing': True, 'continuous_sorting': True, 'real_hardware': False,
                'one_object_per_cycle': True}
# What one authorised trial may do. Fixed in code: a record naming anything else is refused.
G4_TRIAL_OPERATION = {'gate': 'full-cycle', 'max_cycles': G4_TRIAL_MAX_CYCLES, 'class_routing': True,
                      'continuous_sorting': False, 'real_hardware': False, 'one_object_per_cycle': True,
                      'qualified': False}
G4_PERCEPTION_KEYS = ('producer', 'source_type', 'epd_runtime_sha256', 'max_age_s')
G4_SAFEGUARDS = {'contact_guard': True, 'retention_policy': 'strict', 'physics_pads': True, 'stop_after_cycle': 'SIGUSR1',
                 'cancel': 'SIGTERM'}
# Grasps with contact evidence on the qualified body: every counted G3C trial (campaign 09c477e0, both bins) grasped with
# top_2f (world quarter-turn top pinch). Other strategies (e.g. the object-aligned finger_pinch_basic a yawed part plans
# with) have no contact or retention evidence, so a cycle planned with one is never authorised.
G4_QUALIFIED_GRASP_STRATEGIES = ('top_2f',)
G4_STOP_CONDITIONS = ('max_cycles reached', 'stop after cycle latched', 'operator cancel', 'unverified cycle',
                      'destination full or unroutable', 'no eligible object', 'stale or mismatched observation',
                      'refused or failed execution (no retry)')


def g4_perception(workspace, intent):
    """The rendered-EPD perception a G4 session is bound to, from the registered EPD runtime and the saved task."""
    from task_intent_v2 import OBSERVATION_PRODUCER_CEILING_S, freshness_source_type
    record = Path(workspace) / 'workcell_perception_runtime.json'
    if not record.is_file():
        raise PhysicsScopeRefused(f'G4_SCOPE_INCOMPLETE: no registered EPD runtime ({record})')
    return dict(producer='simulator_render', source_type=freshness_source_type(intent),
                epd_runtime_sha256=_sha256(record.read_bytes()), max_age_s=OBSERVATION_PRODUCER_CEILING_S['simulator_render'])


def g4_session_binding(*, cell_dir, receipt_path, native, runtime, perception, max_cycles, planner=None,
                       control=None, commission_evidence_path=None):
    """Everything one G4 session is bound to, recomputed from the current files. Refuses unsupported input."""
    from task_intent_v2 import OBSERVATION_PRODUCER_CEILING_S, class_routing_rules, freshness_source_type, routing_job_policy
    cell_dir = Path(cell_dir)
    intent_path = cell_dir / 'config/workcell_builder_task_intent.yaml'
    intent = yaml.safe_load(intent_path.read_text()) if intent_path.is_file() else {}
    rules = class_routing_rules(intent)
    if rules is None:
        raise PhysicsScopeRefused('G4_SCOPE_UNSUPPORTED: the saved task has no class routing')
    receipt_bytes = Path(receipt_path).read_bytes()
    receipt = json.loads(receipt_bytes)
    bindings = receipt.get('destination_geometry_bindings') or []
    if len(bindings) < 2 or receipt.get('destination_geometry_binding') is not None:
        raise PhysicsScopeRefused('G4_SCOPE_UNSUPPORTED: the receipt does not bind a multi-destination world')
    derived, world = cell_world(cell_dir)
    if len(world['objects']) < 2 or not all(o['within_qualified_dimensions'] for o in world['objects']):
        raise PhysicsScopeRefused('G4_SCOPE_UNSUPPORTED: a G4 cell has several objects, every one of the qualified body')
    _require_launched_world(receipt_path, receipt, derived, world, bindings)
    if not isinstance(max_cycles, int) or isinstance(max_cycles, bool) or not 1 <= max_cycles <= len(world['objects']):
        raise PhysicsScopeRefused('G4_SCOPE_UNSUPPORTED: max_cycles must be between 1 and the number of objects')
    perception = dict(perception or {})
    if (set(perception) != set(G4_PERCEPTION_KEYS) or perception['producer'] != 'simulator_render' or
            perception['source_type'] != 'rendered_perception_in_simulation' or
            perception['source_type'] != freshness_source_type(intent) or
            perception['max_age_s'] != OBSERVATION_PRODUCER_CEILING_S['simulator_render'] or
            not isinstance(perception['epd_runtime_sha256'], str) or len(perception['epd_runtime_sha256']) != 64):
        raise PhysicsScopeRefused('G4_SCOPE_UNSUPPORTED: perception must be the simulator-rendered EPD frame source '
                                  'of this task (g4_perception)')
    files = {}
    for name in CELL_FILES:
        if not (cell_dir / name).is_file():
            raise PhysicsScopeRefused(f'G4_SCOPE_INCOMPLETE: cell file {name} is missing')
        files[name] = _sha256((cell_dir / name).read_bytes())
    evidence = None if commission_evidence_path is None else dict(
        commission_evidence_sha256=_sha256(Path(commission_evidence_path).read_bytes()))
    return dict(
        cell=dict(files=files), world=world,
        destinations=sorted((_portable(b) for b in bindings), key=lambda b: b['model']),
        support=_portable(_required(receipt.get('support_geometry_binding'), 'support binding')),
        robot={key: _required(receipt.get(key), key) for key in ROBOT_RECEIPT_KEYS},
        routing=dict(rules=rules, policy=routing_job_policy(intent)),
        perception=perception, safeguards=dict(G4_SAFEGUARDS), stop_conditions=list(G4_STOP_CONDITIONS),
        grasp_strategies=list(G4_QUALIFIED_GRASP_STRATEGIES),
        native=native, runtime=runtime, planner=planner or planner_identity(), control=control or g4_control_identity(),
        max_cycles=max_cycles, evidence=evidence,
        session=dict(receipt_sha256=_sha256(receipt_bytes), run_id=_required(receipt.get('run_id'), 'run id')))


def g4_open_session(path, binding, acceptance=None):
    """Write the session record; returns its sha256. G4_AUTHORISED_SESSION needs accepted G4 evidence for exactly this
    cell, world and software (``acceptance``, from a future G4 qualification); without it the session is plan-only."""
    state = G4_PLAN_ONLY
    if acceptance is not None:
        raise PhysicsScopeRefused('G4_SCOPE_NOT_QUALIFIED: no G4 acceptance verifier exists; motion is not admitted')
    record = dict(schema=G4_SCHEMA, state=state, operation=G4_OPERATION, binding=binding,
                  binding_sha256=_sha256(_canonical(binding)), created_wall_ns=time.time_ns())
    data = (json.dumps(record, indent=2, sort_keys=True) + '\n').encode()
    Path(path).write_bytes(data)
    return _sha256(data)


def process_identity(pid):
    """A live process as {pid, start_ticks, cmdline}; pid plus kernel start time never repeat while the system runs."""
    stat = Path(f'/proc/{pid}/stat').read_text()
    cmdline = [part.decode(errors='replace') for part in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0') if part]
    return dict(pid=int(pid), start_ticks=stat.rsplit(')', 1)[1].split()[19], cmdline=cmdline)


def _g4_issuer_problem(issuer, pid):
    try:
        live = process_identity(pid)
    except OSError:
        return 'the issuing runner process is gone'
    if issuer != live:
        return 'not the issuing runner process'
    if not any(Path(arg).name == G4_TRIAL_ISSUER[0] for arg in live['cmdline']) or G4_TRIAL_ISSUER[1] not in live['cmdline']:
        return f'the issuer is not {G4_TRIAL_ISSUER[0]} {G4_TRIAL_ISSUER[1]}'
    return None


def _g4_issuer(record, role):
    """A trial record acts only inside the live CLI runner that issued it (``runner``) or an executor it started
    (``executor``: its parent is that runner). A record copied, hand-written or replayed elsewhere is refused."""
    import os
    problem = _g4_issuer_problem(record.get('issuer'), os.getpid() if role == 'runner' else os.getppid())
    if problem:
        raise PhysicsScopeRefused(f'G4_SCOPE_ISSUER_MISMATCH: {problem}; a trial record acts only in the live '
                                  f'{G4_TRIAL_ISSUER[0]} {G4_TRIAL_ISSUER[1]} that issued it')


def g4_authorise_trial(path, binding, issuer):
    """Write the CLI trial record (never replaced); returns its sha256. ``issuer`` is process_identity(os.getpid()) of
    the runner itself: a record naming another process is refused."""
    import os
    cycles = binding.get('max_cycles')
    if not isinstance(cycles, int) or isinstance(cycles, bool) or not 1 <= cycles <= G4_TRIAL_MAX_CYCLES:
        raise PhysicsScopeRefused(f'G4_TRIAL_LIMIT: a trial authorises 1..{G4_TRIAL_MAX_CYCLES} cycles, not {cycles!r}')
    if not (binding.get('evidence') or {}).get('commission_evidence_sha256'):
        raise PhysicsScopeRefused('G4_SCOPE_INCOMPLETE: a trial binds the same-build Stage-A prerequisite trials')
    problem = _g4_issuer_problem(issuer, (issuer or {}).get('pid'))
    if problem is None and (issuer or {}).get('pid') != os.getpid():
        problem = 'only the issuing runner itself writes its trial record'
    if problem:
        raise PhysicsScopeRefused(f'G4_SCOPE_ISSUER_MISMATCH: {problem}')
    record = dict(schema=G4_SCHEMA, state=G4_AUTHORISED_TRIAL, operation=G4_TRIAL_OPERATION,
                  evidence_scope=G4_TRIAL_EVIDENCE_SCOPE, binding=binding, binding_sha256=_sha256(_canonical(binding)),
                  issuer=dict(issuer), created_wall_ns=time.time_ns())
    data = (json.dumps(record, indent=2, sort_keys=True) + '\n').encode()
    with Path(path).open('xb') as out:
        out.write(data)
    return _sha256(data)


def _g4_acceptance(record):
    """Recompute the accepted G4 evidence an authorised session claims. No G4 acceptance contract or verifier exists,
    so the claim is refused: a session record never authorises itself."""
    raise PhysicsScopeRefused('G4_SCOPE_NOT_QUALIFIED: no G4 acceptance verifier exists; an authorised session record '
                              'is a claim, not evidence')


def _g4_session_record(path, sha256):
    """A session record's own integrity (file and binding digests, schema, state, fixed operation, trial bounds). No
    live check: used alone only to judge finished evidence offline, never to authorise anything."""
    data = Path(path).read_bytes()
    if not sha256 or _sha256(data) != sha256:
        raise PhysicsScopeRefused('G4_SCOPE_RECORD_CHANGED: the session record differs from the opened one')
    record = json.loads(data)
    state = record.get('state')
    operation = G4_TRIAL_OPERATION if state == G4_AUTHORISED_TRIAL else G4_OPERATION
    if (record.get('schema') != G4_SCHEMA or record.get('operation') != operation or
            state not in (G4_PLAN_ONLY, G4_AUTHORISED_SESSION, G4_AUTHORISED_TRIAL) or
            record.get('binding_sha256') != _sha256(_canonical(record.get('binding')))):
        raise PhysicsScopeRefused('G4_SCOPE_UNSUPPORTED: not a valid G4 session record')
    if state == G4_AUTHORISED_TRIAL:
        bound = record['binding']
        if (record.get('evidence_scope') != G4_TRIAL_EVIDENCE_SCOPE or bound.get('max_cycles') not in
                range(1, G4_TRIAL_MAX_CYCLES + 1) or not (bound.get('evidence') or {}).get('commission_evidence_sha256')):
            raise PhysicsScopeRefused('G4_SCOPE_UNSUPPORTED: not a valid G4 trial record')
    return record


def _g4_session(path, sha256, *, current=None, role=None):
    """The session record if valid now. A claimed production session is refused; a trial record is also verified
    against ``current`` (the binding recomputed from the current files now) and its live issuing process (``role``)."""
    record = _g4_session_record(path, sha256)
    state = record['state']
    if state == G4_AUTHORISED_SESSION:
        _g4_acceptance(record)
    if state == G4_AUTHORISED_TRIAL:
        bound = record['binding']
        if current is None:
            raise PhysicsScopeRefused('G4_SCOPE_UNVERIFIED: a trial record is verified against its binding recomputed now')
        differing = sorted(key for key in set(bound) | set(current) if bound.get(key) != current.get(key))
        if differing:
            raise PhysicsScopeRefused('G4_SCOPE_MISMATCH: ' + ', '.join(differing))
        _g4_issuer(record, role)
    return record


def _g4_cycle_dir(session_path, session_sha256):
    """Where a trial session's issued and consumed cycle numbers are registered (beside the session record)."""
    return Path(session_path).parent / f'g4-cycles-{session_sha256[:16]}'


def _g4_frame(frame_path, max_age_s):
    """(frame record, problems) of a rendered EPD frame file, re-hashed and aged now."""
    raw = Path(frame_path).read_bytes()
    source = (yaml.safe_load(raw) or {}).get('source') or {}
    capture = source.get('capture_wall_ns')
    frame = dict(path=str(Path(frame_path).resolve()), sha256=_sha256(raw), world_revision=source.get('world_revision'),
                 capture_wall_ns=capture)
    problems = []
    if source.get('producer') != 'simulator_render' or not source.get('world_revision') or not isinstance(capture, int):
        problems.append('not a simulator-rendered EPD frame')
    else:
        age = (time.time_ns() - capture) / 1e9
        frame['age_s'] = age
        if not 0 <= age <= max_age_s:
            problems.append(f'frame age {age:.1f} s outside 0..{max_age_s} s')
    return frame, problems


def _g4_route(bound, class_id, destination):
    """The receipt-bound physics bin of an enabled rule routing ``class_id`` into ``destination``, or refuse."""
    pairs = {(r['when']['equals'], r['destination']) for r in bound['routing']['rules'] if r.get('enabled', True)}
    if (class_id, destination) not in pairs:
        raise PhysicsScopeRefused('G4_TARGET_NOT_ROUTED: the target is not an EPD-classified object of a routed class '
                                  'into this destination')
    return pairs


def g4_routed_destination(bound, region, asset):
    """The one bound bin of ``asset`` (the routed region's bin)."""
    rows = [d for d in bound['destinations'] if d.get('source_item_id') == asset]
    if len(rows) != 1:
        raise PhysicsScopeRefused(f'G4_DESTINATION_UNBOUND: routed region {region!r} -> {asset!r} has '
                                  f'{len(rows)} bound physics bins')
    return rows[0]


def g4_authorise_cycle(path, session_path, session_sha256, *, current, cycle, target, destination, frame_path,
                       observations_path, plan_path, occupancy, previous=None, stop_latched=False):
    """One trial cycle's record, written by the issuing runner after the cycle's plan-only resolve and validation.

    Everything is re-read from files: ``frame_path`` the rendered EPD frame the classes came from (re-hashed, aged
    now); ``observations_path`` the validated measured observations (receipt-bound, classes bound to that frame);
    ``plan_path`` the plan-only resolve summary (its resolution is what the executor consumes); ``target`` {object_id,
    class_id} and ``destination`` (routed region) from the ledger; ``occupancy`` the ledger's object ids expected in
    that bin, which must equal what is measured there. ``previous`` ({path, sha256, summary_path}) is re-verified with
    g4_verify_cycle. A cycle number is issued once per session (registered beside the session record), so a cycle that
    failed or never ran can never be re-authorised: nothing moves after it.
    """
    session = _g4_session(session_path, session_sha256, current=current, role='runner')
    if session['state'] != G4_AUTHORISED_TRIAL:
        raise PhysicsScopeRefused('G4_SCOPE_PLAN_ONLY: this session admits planning only; no cycle may move')
    bound = session['binding']
    if stop_latched:
        raise PhysicsScopeRefused('G4_SESSION_STOPPED: a stop after cycle is latched; no further cycle starts')
    limit = min(bound['max_cycles'], G4_TRIAL_MAX_CYCLES)
    if not isinstance(cycle, int) or isinstance(cycle, bool) or not 1 <= cycle <= limit:
        raise PhysicsScopeRefused(f'G4_SESSION_LIMIT: cycle outside 1..{limit}')
    if (cycle == 1) != (previous is None):
        raise PhysicsScopeRefused('G4_CYCLE_CHAIN_BROKEN: cycle 1 has no predecessor; every later cycle needs one')
    link = None
    if previous is not None:
        verified = g4_verify_cycle(previous['path'], previous['sha256'], previous['summary_path'], bound,
                                   session_sha256=session_sha256)
        prior = json.loads(Path(previous['path']).read_bytes()) if verified['verified'] else {}
        if (not verified['verified'] or prior.get('cycle') != cycle - 1 or
                prior.get('session_sha256') != session_sha256):
            raise PhysicsScopeRefused('G4_CYCLE_CHAIN_BROKEN: the previous cycle is not this session\'s verified '
                                      f"cycle {cycle - 1}: {verified['problems']}")
        link = dict(cycle_sha256=previous['sha256'], evidence_sha256=verified['evidence_sha256'])
    frame, problems = _g4_frame(frame_path, bound['perception']['max_age_s'])
    if problems:
        raise PhysicsScopeRefused('G4_OBSERVATION_STALE: ' + '; '.join(problems))
    raw = Path(observations_path).read_bytes()
    observed = yaml.safe_load(raw) or {}
    perceived = observed.get('perception_binding') or {}
    if observed.get('simulator_receipt_sha256') != bound['session']['receipt_sha256'] or \
            perceived.get('perception_sha256') != frame['sha256']:
        raise PhysicsScopeRefused('G4_OBSERVATION_FOREIGN: the observations are not this session\'s measurement bound '
                                  'to this frame')
    oid, cls = target.get('object_id'), target.get('class_id')
    if oid not in {o['object_id'] for o in bound['world']['objects']}:
        raise PhysicsScopeRefused('G4_TARGET_UNKNOWN: the target is not an object of the bound world')
    measured = {o['object_id']: o for o in observed.get('objects', [])}
    if (measured.get(oid) or {}).get('class_id') != cls or ((perceived.get('bindings') or {}).get(oid) or {}).get(
            'class_id') != cls:
        raise PhysicsScopeRefused('G4_TARGET_CLASS_MISMATCH: the measured target is not bound to this EPD class')
    _g4_route(bound, cls, destination)
    plan_raw = Path(plan_path).read_bytes()
    plan = json.loads(plan_raw)
    resolution = plan.get('task_intent_resolution') or {}
    route = (resolution.get('place_resolution') or {}).get('route') or {}
    grasp = (plan.get('selected_extraction_intent') or {}).get('candidate_id')
    walls = (plan.get('physical_destination_walls') or {}).get('selected_destination') or {}
    routed = g4_routed_destination(bound, destination, route.get('asset_ref'))
    if (plan.get('result') != 'PLAN_ONLY' or plan.get('execution_attempted') is not False or
            plan.get('selected_object_id') != 'runtime::' + oid or plan.get('batch_target_object_id') != 'runtime::' + oid or
            not plan.get('resolution_sha256') or resolution.get('resolution_sha256') != plan['resolution_sha256'] or
            route.get('region_ref') != destination or route.get('class_id') != cls or
            walls.get('model') != routed['model'] or not plan.get('physics_pads') or
            (resolution.get('grasp_resolution') or {}).get('selected_candidate_id') != grasp):
        raise PhysicsScopeRefused('G4_PLAN_MISMATCH: the plan is not this target\'s padded plan-only resolve into its '
                                  'routed bin')
    if not isinstance(grasp, str) or grasp.split('::', 1)[0] not in bound['grasp_strategies']:
        raise PhysicsScopeRefused(f'G4_GRASP_UNQUALIFIED: {grasp!r} has no contact evidence on the qualified body '
                                  f"(qualified: {bound['grasp_strategies']})")
    occupants = destination_occupants(observed, routed)
    if oid in occupants or occupants != sorted(occupancy):
        raise PhysicsScopeRefused(f'G4_OCCUPANCY_MISMATCH: measured in {routed["source_item_id"]}: {occupants}; '
                                  f'the session expects {sorted(occupancy)}')
    record = dict(schema=G4_SCHEMA, state=G4_TRIAL_CYCLE, evidence_scope=G4_TRIAL_EVIDENCE_SCOPE,
                  session_sha256=session_sha256, cycle=cycle, target=dict(object_id=oid, class_id=cls),
                  destination=dict(region=destination, asset=routed['source_item_id'], model=routed['model'],
                                   object_id=routed['object_id'], floor_z=routed['floor_z']),
                  grasp=grasp, frame=frame, observations=dict(path=str(Path(observations_path).resolve()), sha256=_sha256(raw)),
                  plan=dict(path=str(Path(plan_path).resolve()), sha256=_sha256(plan_raw),
                            resolution_sha256=plan['resolution_sha256']),
                  occupancy=[dict(object_id=o, pose=measured[o]['pose']) for o in occupants], previous=link,
                  created_wall_ns=time.time_ns())
    data = (json.dumps(record, indent=2, sort_keys=True) + '\n').encode()
    registry = _g4_cycle_dir(session_path, session_sha256)
    registry.mkdir(exist_ok=True)
    try:                               # the issued marker names the one record digest this cycle number was given
        with (registry / f'{cycle}.issued').open('x') as marker:
            marker.write(_sha256(data) + '\n')
    except FileExistsError:
        raise PhysicsScopeRefused(f'G4_CYCLE_ALREADY_ISSUED: cycle {cycle} of this session was authorised before; a '
                                  'cycle number is never re-issued') from None
    with Path(path).open('xb') as out:
        out.write(data)
    return _sha256(data)


def g4_admit_execution(*, session_path, session_sha256, cycle_path, cycle_sha256, plan_path, observations_path,
                       target_object_id, current):
    """The executor's admission of one trial cycle, before any planning or motion. Returns {record, session, binding,
    destination (the routed receipt binding), resolution (the authorised plan's, to consume unchanged)}."""
    session = _g4_session(session_path, session_sha256, current=current, role='executor')
    if session['state'] != G4_AUTHORISED_TRIAL:
        raise PhysicsScopeRefused('G4_SCOPE_PLAN_ONLY: this session admits planning only; no cycle may move')
    bound = session['binding']
    data = Path(cycle_path).read_bytes()
    if not cycle_sha256 or _sha256(data) != cycle_sha256:
        raise PhysicsScopeRefused('G4_CYCLE_RECORD_CHANGED: the cycle record differs from the authorised one')
    record = json.loads(data)
    cycle = record.get('cycle')
    registry = _g4_cycle_dir(session_path, session_sha256)
    issued = registry / f'{cycle}.issued' if isinstance(cycle, int) else None
    if (record.get('schema') != G4_SCHEMA or record.get('state') != G4_TRIAL_CYCLE or
            record.get('session_sha256') != session_sha256 or record.get('evidence_scope') != G4_TRIAL_EVIDENCE_SCOPE or
            not isinstance(cycle, int) or not 1 <= cycle <= min(bound['max_cycles'], G4_TRIAL_MAX_CYCLES) or
            not issued.is_file() or issued.read_text().strip() != cycle_sha256):
        raise PhysicsScopeRefused('G4_CYCLE_UNSUPPORTED: not the record this session issued for that cycle')
    if (cycle == 1) != (record.get('previous') is None) or (
            cycle > 1 and not (registry / f'{cycle - 1}.consumed').is_file()):
        raise PhysicsScopeRefused('G4_CYCLE_CHAIN_BROKEN: a later cycle needs its predecessor issued, run and verified')
    if str(record.get('grasp')).split('::', 1)[0] not in bound['grasp_strategies']:
        raise PhysicsScopeRefused(f"G4_GRASP_UNQUALIFIED: {record.get('grasp')!r} has no contact evidence on the "
                                  'qualified body')
    _g4_route(bound, record['target']['class_id'], record['destination']['region'])
    if (registry / f'{cycle}.consumed').exists():
        raise PhysicsScopeRefused(f'G4_CYCLE_REPLAYED: cycle {cycle} already ran; a cycle record moves once')
    if 'runtime::' + record['target']['object_id'] != target_object_id:
        raise PhysicsScopeRefused('G4_TARGET_MISMATCH: --target-object-id is not the authorised target')
    if _sha256(Path(observations_path).read_bytes()) != record['observations']['sha256']:
        raise PhysicsScopeRefused('G4_OBSERVATION_CHANGED: the observations are not the validated, authorised ones')
    plan_raw = Path(plan_path).read_bytes()
    if _sha256(plan_raw) != record['plan']['sha256']:
        raise PhysicsScopeRefused('G4_PLAN_CHANGED: the plan is not the authorised plan-only resolve')
    resolution = json.loads(plan_raw).get('task_intent_resolution') or {}
    if resolution.get('resolution_sha256') != record['plan']['resolution_sha256']:
        raise PhysicsScopeRefused('G4_PLAN_CHANGED: the plan\'s resolution is not the authorised one')
    frame, problems = _g4_frame(record['frame']['path'], bound['perception']['max_age_s'])
    if problems or frame['sha256'] != record['frame']['sha256']:
        raise PhysicsScopeRefused('G4_OBSERVATION_STALE: ' + ('; '.join(problems) or 'the frame file changed'))
    destination = g4_routed_destination(bound, record['destination']['region'], record['destination']['asset'])
    if destination['model'] != record['destination']['model']:
        raise PhysicsScopeRefused('G4_DESTINATION_MISMATCH: the routed bin is not the authorised bin')
    return dict(record=record, session=session, binding=bound, destination=destination, resolution=resolution,
                frame_age_s=frame['age_s'])


def g4_frame_age_problem(record, bound):
    """At the motion boundary: is the authorised frame still the same file and young enough? (None when it is.)"""
    frame, problems = _g4_frame(record['frame']['path'], bound['perception']['max_age_s'])
    if not problems and frame['sha256'] != record['frame']['sha256']:
        problems.append('the frame file changed')
    return '; '.join(problems) or None


def g4_consume_cycle(session_path, session_sha256, cycle):
    """Mark a cycle as run, at the motion boundary (exclusive create): a cycle record moves the robot at most once."""
    try:
        with (_g4_cycle_dir(session_path, session_sha256) / f'{cycle}.consumed').open('x') as marker:
            marker.write(f'{time.time_ns()}\n')
    except FileExistsError:
        raise PhysicsScopeRefused(f'G4_CYCLE_REPLAYED: cycle {cycle} already ran; a cycle record moves once') from None


def _bottom_corners_z(pose, dims):
    return sorted(pose[2] + _rotate(pose[3:], [sx * dims[0] / 2, sy * dims[1] / 2, sz * dims[2] / 2])[2]
                  for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1))


def g4_verify_cycle(cycle_path, cycle_sha256, summary_path, binding, session_sha256=None):
    """A trial cycle counts only from independently re-verified physical evidence (``summary_path``: the executor's
    summary file): it executed exactly this record, the G3C per-cycle judgement (physical_cycle_problems) holds on the
    record's routed bin, every bottom corner rests on that floor (no part tilted against a neighbour), and the target
    and grasp are the authorised ones, detached and reconciled. With ``session_sha256`` the record and the summary
    must both belong to that session. Returns {verified, problems, cycle, evidence_sha256}."""
    data = Path(cycle_path).read_bytes()
    if not cycle_sha256 or _sha256(data) != cycle_sha256:
        return dict(verified=False, problems=['G4_CYCLE_RECORD_CHANGED'], cycle=None, evidence_sha256=None)
    record = json.loads(data)
    try:
        summary_raw = Path(summary_path).read_bytes()
        summary = json.loads(summary_raw)
    except (OSError, ValueError):
        return dict(verified=False, problems=['executor summary missing or unreadable'], cycle=record.get('cycle'),
                    evidence_sha256=None)
    problems = []
    if record.get('schema') != G4_SCHEMA or record.get('state') != G4_TRIAL_CYCLE:
        problems.append('not a G4 trial cycle record')
    scope = summary.get('physics_scope') or {}
    if scope.get('cycle_sha256') != cycle_sha256 or scope.get('session_sha256') != record.get('session_sha256'):
        problems.append('the summary is not the execution of this cycle record')
    if session_sha256 is not None and record.get('session_sha256') != session_sha256:
        problems.append('the cycle record belongs to another session')
    if summary.get('resolution_sha256') != (record.get('plan') or {}).get('resolution_sha256'):
        problems.append('the executed resolution is not the authorised plan')
    target = 'runtime::' + str((record.get('target') or {}).get('object_id'))
    destination = next((d for d in binding.get('destinations', [])
                        if d.get('model') == (record.get('destination') or {}).get('model')), None)
    dims = next((o['dimensions_m'] for o in (binding.get('world') or {}).get('objects', [])
                 if 'runtime::' + o['object_id'] == target), None)
    problems += physical_cycle_problems(summary, destination, {'dimensions_m': dims} if dims else None)
    try:
        corners = _bottom_corners_z(summary['full_cycle_physical_acceptance']['final_pose'], dims)
        if any(abs(z - destination['floor_z']) > QUALIFICATION_CONTRACT['seat_tolerance_m'] for z in corners[:4]):
            problems.append('measured part not flat on the selected bin floor')
    except (KeyError, TypeError, IndexError):
        pass                                             # already reported by physical_cycle_problems
    if summary.get('selected_object_id') != target:
        problems.append('the executed object is not the authorised target')
    if not record.get('grasp') or (summary.get('selected_extraction_intent') or {}).get('candidate_id') != record['grasp']:
        problems.append('the executed grasp is not the authorised grasp')
    reconciled = summary.get('measured_reconciliation') or {}
    if reconciled.get('held') is not False or reconciled.get('target') != target or not reconciled.get('acm_restored'):
        problems.append('target not measured, detached and reconciled')
    return dict(verified=not problems, problems=problems, cycle=record.get('cycle'),
                evidence_sha256=_sha256(_canonical(dict(cycle=cycle_sha256, summary=_sha256(summary_raw)))))


# --- G4 qualification (offline; issues nothing) -------------------------------------------------------------------
# Promotion of accepted trials, later: readiness is recomputed from raw trial evidence only. Fixed in code. No issuer
# exists yet and G4 stop/cancel acceptance (G5-style, inside the G4 runner) is not defined, so readiness is never
# QUALIFIED_READY today; _g4_acceptance keeps refusing G4_AUTHORISED_SESSION.
G4_QUALIFICATION_CONTRACT = {
    'trial_sessions': 3,              # independent accepted trial sessions of the exact cell and software
    'cycles_per_session': G4_TRIAL_MAX_CYCLES,
    'seat_tolerance_m': 0.001, 'placement_tolerance_m': 0.003,
    'stop_cancel': 'G4 runner stop after cycle and cancel during a G4 cycle (not defined yet)',
}
G4_RUN_REPORT = 'g4-sorting-report.json'


def _g4_prerequisite_problem(summary, bound):
    prerequisites = summary.get('commissioning_prerequisites') or {}
    if (sorted(prerequisites.get('results') or []) != PREREQUISITE_RESULTS or
            prerequisites.get('capability_sha256') != (bound.get('native') or {}).get('sha256') or
            prerequisites.get('sources') != (bound.get('native') or {}).get('sources')):
        return 'Stage-A prerequisite trials not bound to the trial build'
    return None


def g4_trial_evidence(run_dir, repo=REPO):
    """One G4 trial run judged from its own files. Kinds:
    STRICT_PASS         both bounded cycles moved and were completed by the runner (verified and reconciled), every
                        one re-verified here, chained, bound to this session, clean run;
    INCOMPLETE          an operator stop or batch end: every moved cycle completed and verified, fewer than the bound;
    CANCELLED           the operator cancelled (G5 stop evidence, never a cycle outcome); every earlier cycle verified;
    PRE_MOTION_FAILURE  nothing moved (refused, or authorised and never started): a reliability finding;
    FAILED              a moved cycle not completed or not verified (a FAULT after motion blocks), or unclean provenance;
    NOT_A_TRIAL         a plan-only session (a run whose session record is an authorised trial is never one).
    Malformed evidence is FAILED, never an exception."""
    run = Path(run_dir)
    out = dict(run=str(run.resolve()), kind='FAILED', problems=[], cycles=[], planner=None, control=None, binding=None,
               run_id=None)
    try:
        return _g4_trial_evidence(run, repo, out)
    except Exception as exc:                       # noqa: BLE001 (any malformed input is a judged failure)
        out.update(kind='FAILED', problems=out['problems'] + [f'malformed trial evidence: {exc!r}'])
        return out


def _g4_trial_evidence(run, repo, out):
    try:
        report = json.loads((run / G4_RUN_REPORT).read_text())
    except (OSError, ValueError):
        out['problems'].append('run report missing or unreadable')
        return out
    if report.get('scope') != G4_TRIAL_EVIDENCE_SCOPE:
        try:
            state = json.loads((run / 'session/g4-session.json').read_text()).get('state')
        except (OSError, ValueError, AttributeError):
            state = None
        if state == G4_AUTHORISED_TRIAL:           # a trial's report relabelled or damaged: never "not a trial"
            out['problems'].append('the run report of an authorised trial is not labelled as one')
            return out
        out['kind'] = 'NOT_A_TRIAL'
        return out
    session_sha = (report.get('g4_session') or {}).get('sha256')
    try:
        record = _g4_session_record(run / 'session/g4-session.json', session_sha)
    except (OSError, ValueError, PhysicsScopeRefused) as exc:
        out['problems'].append(f'session record: {exc}')
        return out
    if record['state'] != G4_AUTHORISED_TRIAL:
        out['problems'].append('the session record is not a trial')
        return out
    bound = record['binding']
    out.update(binding=bound, run_id=bound['session']['run_id'])
    out['planner'], provenance = _software(report, bound, repo)
    out['control'], control_problems = _g4_control(report, bound, repo)
    provenance += control_problems
    if (report.get('preflight') or {}).get('repo_dirty') is not False:
        provenance.append('the run did not record a clean checkout')
    if (report.get('shutdown') or {}).get('clean') is not True:
        provenance.append('the run did not shut down cleanly')
    status = report.get('status')
    granted = [c for c in report.get('cycles', []) if ((c.get('steps') or {}).get('authorise') or {}).get('granted')]
    moved, failures, previous = 0, [], None
    for n, cycle in enumerate(granted, 1):
        steps = cycle['steps']
        sha = steps['authorise'].get('sha256')
        path = run / 'session/cycles' / f"{int(cycle['cycle']):03d}"
        summary_path = path / 'execute/summary.json'
        if 'execute' not in steps and not summary_path.is_file():   # authorised, stopped before the executor ran
            out['cycles'].append(dict(cycle=n, moved=False, verified=False, problems=['authorised, never executed']))
            continue
        attempted = json.loads(summary_path.read_text()).get('execution_attempted') if summary_path.is_file() else None
        if attempted is False:                     # refused before motion: nothing to verify, and nothing moved
            out['cycles'].append(dict(cycle=n, moved=False, verified=False, problems=['refused before motion']))
            continue
        moved += 1
        cancelled = cycle.get('outcome') == 'CANCELLED' and status == 'CANCELLED'
        verified = g4_verify_cycle(path / 'g4-cycle.json', sha, summary_path, bound, session_sha256=session_sha)
        problems = list(verified['problems'])
        cycle_record = json.loads((path / 'g4-cycle.json').read_text())
        if cycle_record.get('cycle') != n or cycle_record.get('previous') != previous:
            problems.append('cycle chain broken')
        if summary_path.is_file():
            problem = _g4_prerequisite_problem(json.loads(summary_path.read_text()), bound)
            if problem:
                problems.append(problem)
        # The runner's own post-motion checks (gate, ledger, reconcile measurement) must have completed this cycle.
        if (cycle.get('outcome') != 'COMPLETED' or (steps.get('verify') or {}).get('verified') is not True or
                not steps.get('reconcile')):
            problems.append(f"the runner did not complete this cycle ({cycle.get('outcome')})")
        out['cycles'].append(dict(cycle=n, moved=True, verified=not problems, cancelled=cancelled, problems=problems,
                                  evidence_sha256=verified.get('evidence_sha256')))
        if not cancelled:                          # the cancelled cycle is stop evidence, not a cycle outcome
            failures += [f'cycle {n}: {p}' for p in problems]
        previous = dict(cycle_sha256=sha, evidence_sha256=verified.get('evidence_sha256'))
    out['problems'] = provenance + failures
    if failures or provenance:
        out['kind'] = 'FAILED' if moved or provenance else 'PRE_MOTION_FAILURE'
    elif status == 'CANCELLED':
        out['kind'] = 'CANCELLED'
    elif not moved:
        out['kind'] = 'PRE_MOTION_FAILURE'
        out['problems'].append(f"nothing moved ({status}: {report.get('stop') or report.get('failure')})")
    elif moved == bound['max_cycles'] == G4_TRIAL_MAX_CYCLES and status == 'PASS' and report.get('result') == 'MAX_CYCLES':
        out['kind'] = 'STRICT_PASS'
    elif status == 'STOPPED' or (status == 'PASS' and report.get('result') in ('BATCH_COMPLETE', 'MAX_CYCLES')):
        out['kind'] = 'INCOMPLETE'
        out['problems'].append(f"{moved} of {G4_TRIAL_MAX_CYCLES} bounded cycles ran ({status} / {report.get('result')})")
    else:                                          # e.g. FAULT/BLOCKED after verified cycles: a failure after motion
        out['kind'] = 'FAILED'
        out['problems'].append(f"the run ended {status} after {moved} moved cycle(s): {report.get('stop') or report.get('failure')}")
    return out


def _g4_control(report, bound, repo):
    """(G4 control identity, problems) of one trial: recovered from its recorded clean commit and equal to the bound one."""
    head = (report.get('preflight') or {}).get('repo_head')
    recorded = (bound or {}).get('control')
    if not head:
        return recorded, ['G4 control identity unrecoverable: the trial recorded no commit']
    try:
        recovered = g4_control_identity(repo, head)
    except PhysicsScopeRefused as exc:
        return recorded, [str(exc)]
    if recorded is None:                           # a trial from before the control binding: its own version, never counted
        return recovered, ['G4 control identity not bound by the trial']
    if recorded != recovered:
        return recovered, ['G4 control identity differs from the commit the trial ran from']
    return recovered, []


def _g4_shared(bound):
    """What every counted trial of one G4 scope must share (session-specific fields left out)."""
    strip = lambda record: {k: v for k, v in record.items() if k != 'manifest_sha256'}
    return dict(world=bound['world'], destinations=[strip(d) for d in bound['destinations']],
                support=strip(bound['support']), robot={k: bound['robot'][k] for k in STABLE_ROBOT_KEYS},
                routing=bound['routing'], perception=bound['perception'], grasp_strategies=bound['grasp_strategies'],
                safeguards=bound['safeguards'], stop_conditions=bound['stop_conditions'],
                cell={k: bound['cell']['files'].get(k) for k in QUALIFIED_CELL_FILES})


def g4_qualification_readiness(cell_dir, runs, native, runtime, planner=None, *, evidence_roots, control=None,
                               repo=REPO):
    """Readiness of one exact G4 cell from listed trial runs, recomputed from raw evidence (issues nothing).
    Categories as G3C: reliability (pre-motion), incomplete (stopped/cancelled, all moved cycles verified: listed, not
    counted, not blocking), other_versions (another planner or G4 control identity: listed, never counted), unmet
    (blocking). Every authorised trial session of this planner identity and cell under ``evidence_roots`` must be
    listed; unreadable trial evidence blocks."""
    contract = G4_QUALIFICATION_CONTRACT
    current = planner or planner_identity(repo)
    current_control = control or g4_control_identity(repo)
    _, world = cell_world(cell_dir)
    files = {name: _sha256((Path(cell_dir) / name).read_bytes()) for name in QUALIFIED_CELL_FILES}
    counted, unmet, reliability, incomplete, other_versions, seen, shared = [], [], [], [], [], set(), None
    for run in runs:
        ev = g4_trial_evidence(run, repo)
        if ev['kind'] == 'PRE_MOTION_FAILURE':
            reliability.append(dict(run=ev['run'], problems=ev['problems']))
            continue
        if (ev['planner'] is not None and ev['planner'] != current) or (
                ev['control'] is not None and ev['control'] != current_control):
            other_versions.append(dict(run=ev['run'], planner=ev['planner'], control=ev['control'], kind=ev['kind']))
            continue
        if ev['kind'] in ('INCOMPLETE', 'CANCELLED'):
            incomplete.append(dict(run=ev['run'], kind=ev['kind'], problems=ev['problems']))
            continue
        if ev['kind'] != 'STRICT_PASS':
            unmet.append(f"{ev['run']}: {ev['kind']}: {ev['problems']}")
            continue
        bound = ev['binding']
        problems = [label for label, ok in (
            ('native identity changed', bound['native'] == native), ('runtime identity changed', bound['runtime'] == runtime),
            ('another cell', {k: bound['cell']['files'].get(k) for k in QUALIFIED_CELL_FILES} == files),
            ('another world', bound['world']['derived_sha256'] == world['derived_sha256']),
            ('geometry, robot, routing, perception or safeguards differ from the other trials',
             shared is None or _g4_shared(bound) == shared),
            ('the same trial session counted twice', ev['run_id'] not in seen)) if not ok]
        if problems:
            unmet.append(f"{ev['run']}: {problems}")
            continue
        shared = shared or _g4_shared(bound)
        seen.add(ev['run_id'])
        counted.append(ev['run'])
    if len(counted) < contract['trial_sessions']:
        unmet.append(f"{len(counted)} of {contract['trial_sessions']} accepted trial sessions")
    listed = {str(Path(r).resolve()) for r in runs}
    unlisted = []
    for root in evidence_roots:
        for session_path in sorted(Path(root).rglob('g4-session.json')):
            run = session_path.parent.parent
            try:
                record = json.loads(session_path.read_text())
                if record.get('state') != G4_AUTHORISED_TRIAL:
                    continue                                       # plan-only sessions never moved
                bound = record['binding']
                same = (bound['world']['derived_sha256'] == world['derived_sha256'] and
                        {k: bound['cell']['files'].get(k) for k in QUALIFIED_CELL_FILES} == files)   # this exact cell
                report = json.loads((run / G4_RUN_REPORT).read_text())
                software = _software(report, bound, repo)[0]
            except (OSError, ValueError, KeyError, TypeError):
                if str(run.resolve()) not in listed:
                    unlisted.append(f'{run.resolve()} (unreadable trial evidence)')
                continue
            if same and software == current and str(run.resolve()) not in listed:
                unlisted.append(str(run.resolve()))
    if unlisted:
        unmet.append(f'unlisted trials of this software version: {unlisted}')
    unmet.append(f"stop/cancel: {contract['stop_cancel']}")
    return dict(status='NOT_QUALIFIED' if unmet else 'QUALIFIED_READY', contract=contract, planner=current,
                control=current_control,
                world=world['derived_sha256'], counted=counted, unmet=unmet, reliability_findings=reliability,
                incomplete=incomplete, other_versions=other_versions, unlisted=unlisted)


# --- Qualification (offline; admits no motion) -------------------------------------------------------------------
# A QUALIFIED scope would admit one exact cell (its task places into one bin of a multi-destination world) beyond the
# CLI trial path. Fixed in code: no editable file can lower it. Nothing here authorises motion, and a trial never
# qualifies itself: readiness is recomputed from the bound evidence of several independent strict trials, and the
# contract's stop/cancel item has no accepted evidence yet, so every readiness is NOT_QUALIFIED today.
QUALIFIED = 'QUALIFIED'
QUALIFICATION_CONTRACT = {
    'strict_trials': 3,               # independent strict PASS sessions of the exact cell into its bin
    'seat_tolerance_m': 0.001,        # measured part bottom on the bin's bound floor
    'placement_tolerance_m': 0.003,   # measured final pose to the planned seat (the executor's own bound)
    'stop_cancel': 'R2.5B G5: stop at cycle end and SIGTERM cancellation in the multi-destination world',
}
# The exact cell across sessions: cell_definition.yaml embeds the session's plan resolution (bound per trial).
QUALIFIED_CELL_FILES = tuple(name for name in CELL_FILES if name != 'cell_definition.yaml')
# The launched URDF embeds the session's controller path; its source is bound by the cell's xacro files.
STABLE_ROBOT_KEYS = tuple(k for k in ROBOT_RECEIPT_KEYS if k not in ('description_sha256', 'native_description_sha256'))
FULL_CYCLE_STAGES = ['EXECUTE_APPROACH', 'EXECUTE_GRASP', 'EXECUTE_CLOSE_GRIPPER', 'ATTACH', 'EXECUTE_LIFT',
                     'EXECUTE_TRANSFER', 'EXECUTE_PLACE', 'EXECUTE_OPEN_GRIPPER', 'DETACH', 'EXECUTE_RETREAT',
                     'EXECUTE_HOME']
PREREQUISITE_RESULTS = ['CANCELLATION_TRIAL_PASS', 'CONTACT_RELEASE_PASS', 'MOTION_TELEMETRY_PASS',
                        'STATIONARY_RETENTION_PASS']


def _rotate(q, v):
    x, y, z, w = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return [v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx]


def _run_files(run):
    session = run / '06-full-cycle'
    return dict(report=run / 'stage-a1-targeted-report.json', session_report=session / 'session-report.json',
                scope=session / 'physics-scope.json', summary=session / 'full-cycle/summary.json')


def physical_cycle_problems(summary, destination, part):
    """Q5 for one executed full-cycle, from the executor's own measured evidence: a strict PASS, every stage, release
    confirmed and settled, nothing attached, baseline ACM and a valid final collision state, the planned seat within
    3 mm, and the measured part resting inside ``destination`` (a bound bin floor) with its lowest corner on that
    floor. ``part`` gives the part's dimensions. Used by trial_evidence and by the sorting ledger (G4)."""
    contract, problems = QUALIFICATION_CONTRACT, []
    accepted = summary.get('full_cycle_physical_acceptance') or {}
    release = summary.get('release_evidence') or {}
    if (summary.get('result') != 'PASS' or summary.get('strict_retention_compliant') is not True or
            summary.get('retention_policy') != 'strict' or (summary.get('retention_findings') or {}).get('findings')):
        problems.append('not a strict full-cycle PASS')
    if summary.get('stages') != FULL_CYCLE_STAGES:
        problems.append('incomplete stage sequence')
    if release.get('state') != 'RELEASE_CONFIRMED' or release.get('settled') is not True:
        problems.append('release not confirmed and settled')
    if (accepted.get('attached_ids') != [] or accepted.get('baseline_acm_restored') is not True or
            accepted.get('final_collision_valid') is not True):
        problems.append('attachment, collision permissions or final collision state not restored')
    error = (accepted.get('placement') or {}).get('error_to_planned_m')
    if not isinstance(error, (int, float)) or isinstance(error, bool) or not error <= contract['placement_tolerance_m']:
        problems.append('placement error to the planned seat exceeds the bound')
    try:
        pose = accepted['final_pose']
        bottom = min(pose[2] + _rotate(pose[3:], [sx * part['dimensions_m'][0] / 2, sy * part['dimensions_m'][1] / 2,
                                                  sz * part['dimensions_m'][2] / 2])[2]
                     for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1))
        inside = all(abs(pose[i] - destination['floor_xyz'][i]) <= destination['floor_size_m'][i] / 2 for i in (0, 1))
        if not inside or abs(bottom - destination['floor_z']) > contract['seat_tolerance_m']:
            problems.append('measured part not seated on the selected bin floor')
    except (KeyError, TypeError, IndexError):
        problems.append('measured final pose or selected bin not recorded')
    return problems


def _provenance(out, paths, status, label):
    """Q1 for one authorised run with every evidence file present: its digests, binding and session, and every
    provenance problem (official report with ``status``, clean checkout, the verified record, clean shutdown)."""
    out['digests'] = {k: _sha256(path.read_bytes()) for k, path in paths.items()}
    report, session_report, record, summary = (json.loads(paths[k].read_text())
                                               for k in ('report', 'session_report', 'scope', 'summary'))
    bound = record.get('binding') or {}
    out.update(binding=bound, run_id=(bound.get('session') or {}).get('run_id'))
    problems = out['problems']
    if (report.get('status') != status or report.get('scope') != TRIAL_EVIDENCE_SCOPE or report.get('mode') != 'targeted'
            or report.get('gate') != 'full-cycle' or report.get('stage_a_acceptance') is not False):
        problems.append(f'not a {label} authorised trial report')
    if (report.get('preflight') or {}).get('repo_dirty') is not False:
        problems.append('trial ran from a modified checkout')
    if ((session_report.get('physics_scope') or {}).get('sha256') != out['digests']['scope'] or
            (summary.get('physics_scope') or {}).get('sha256') != out['digests']['scope']):
        problems.append('scope record differs from the one the trial verified')
    if (record.get('schema') != SCHEMA or record.get('state') != AUTHORISED_TRIAL or
            record.get('operation') != TRIAL_OPERATION or record.get('binding_sha256') != _sha256(_canonical(bound))):
        problems.append('scope record invalid')
    if not (session_report.get('shutdown') or {}).get('clean'):
        problems.append('owned shutdown not clean')
    return report, session_report, summary


def _software(report, bound, repo):
    """(identity, problems) of one run: recovered from its recorded clean commit; a newer run's scope record also binds
    it, and the two must agree (a forged or substituted identity never counts)."""
    head = (report.get('preflight') or {}).get('repo_head')
    recorded = (bound or {}).get('planner')
    if not head:
        return recorded, ['software provenance unrecoverable: the trial recorded no commit']
    try:
        recovered = planner_identity(repo, head)
    except PhysicsScopeRefused as exc:
        return recorded, [str(exc)]
    if recorded is not None and recorded != recovered:
        # Judged as the version it really ran (never relabelled into another campaign), and refused.
        return recovered, ['software identity differs from the commit the trial ran from']
    return recovered, []


def trial_evidence(run_dir, repo=REPO):
    """One authorised trial, judged from its own evidence files: kind STRICT_PASS, PRE_MOTION_FAILURE or FAILED."""
    run, contract = Path(run_dir), QUALIFICATION_CONTRACT
    session = run / '06-full-cycle'
    paths = _run_files(run)
    out = dict(run=str(run.resolve()), kind='FAILED', problems=[], digests={})
    if not paths['report'].is_file():
        out['problems'].append('no trial report')
        return out
    report = json.loads(paths['report'].read_text())
    if not paths['scope'].is_file() and report.get('status') != 'PASS':
        # Stopped before any scope was authorised (so before motion): a reliability finding, not a safety one.
        planned = [json.loads(p.read_text()) for p in (session / 'revalidate/summary.json', session / 'resolve/summary.json')
                   if p.is_file()]
        why = next((f"{s.get('failed_stage')}: {s.get('failure')}" for s in planned if s.get('failure')),
                   str(report.get('failure', 'not authorised'))[:300])
        return dict(out, kind='PRE_MOTION_FAILURE', problems=[why], planner=_software(report, None, repo)[0])
    missing = sorted(k for k, path in paths.items() if not path.is_file())
    if missing:
        out['problems'].append('evidence missing: ' + ', '.join(missing))
        return out
    report, session_report, summary = _provenance(out, paths, 'PASS', 'passing')
    bound, problems = out['binding'], out['problems']
    out['planner'], why = _software(report, bound, repo)
    problems.extend(why)
    try:
        bin_ = next(d for d in bound['destinations'] if d['model'] == bound['destination']['model'])
        part = next(o for o in bound['world']['objects']
                    if 'runtime::' + o['object_id'] == summary.get('selected_object_id'))
    except (KeyError, StopIteration, TypeError):
        bin_ = part = None
    problems.extend(physical_cycle_problems(summary, bin_, part))
    prerequisites = summary.get('commissioning_prerequisites') or {}
    if (sorted(prerequisites.get('results') or []) != PREREQUISITE_RESULTS or
            prerequisites.get('capability_sha256') != (bound.get('native') or {}).get('sha256') or
            prerequisites.get('sources') != (bound.get('native') or {}).get('sources')):
        problems.append('Stage-A prerequisite trials not bound to the trial build')
    if not problems:
        out['kind'] = 'STRICT_PASS'
    return out


def stop_after_cycle_acceptance(run_dir, repo=REPO):
    """G5B: an operator Stop at Cycle End (SIGUSR1 to the runner) latched during the authorised trial's transfer, the
    cycle completed as a strict trial and nothing else started. Judged only from the trial's own evidence files."""
    run = Path(run_dir)
    trial = trial_evidence(run, repo)
    report, session_report, summary = (json.loads(p.read_text()) if p.is_file() else {} for p in (
        run / 'stage-a1-targeted-report.json', run / '06-full-cycle/session-report.json',
        run / '06-full-cycle/full-cycle/summary.json'))
    stop = report.get('stop_after_cycle') or {}
    requested = stop.get('requested_wall_ns')
    began = {e.get('stage'): e.get('timestamp') for e in summary.get('stage_events') or [] if isinstance(e, dict)}
    transfer, place = began.get('EXECUTE_TRANSFER'), began.get('EXECUTE_PLACE')
    checks = dict(
        latched=stop.get('signal') == 'SIGUSR1' and isinstance(requested, int) and stop.get('forwarded') is None
        and 'acknowledgement_error' not in stop,
        during_transfer=(stop.get('gate') == 'full-cycle' and stop.get('phase') == 'execute'
                         and stop.get('executor_stage') == 'EXECUTE_TRANSFER' and isinstance(requested, int)
                         and isinstance(transfer, (int, float)) and isinstance(place, (int, float))
                         and transfer * 1e9 < requested < place * 1e9),
        not_cancelled=('operator_cancel' not in session_report and 'superseded_by' not in stop
                       and summary.get('failed_stage') is None and not summary.get('recovery_required')),
        cycle_completed_strictly=trial['kind'] == 'STRICT_PASS',
        no_further_cycle=(stop.get('state') == 'STOPPED' and stop.get('gates_not_started') == []
                          and list(report.get('gates') or {}) == ['full-cycle']),
    )
    failed = [name for name, ok in checks.items() if not ok]
    return dict(accepted=not failed, state='STOPPED_AFTER_CYCLE' if not failed else 'NOT_ACCEPTED', failed=failed,
                checks=checks, trial_problems=trial['problems'], run=trial['run'], run_id=trial.get('run_id'),
                digests=trial['digests'], binding=trial.get('binding'), planner=trial.get('planner'),
                repo_head=(report.get('preflight') or {}).get('repo_head'))


def cancellation_evidence(run_dir, repo=REPO):
    """G5A: an operator cancel (SIGTERM to the runner) of the authorised trial's held transfer, judged from the trial's
    own evidence files: Q1 provenance of a CANCELLED run, the stop forwarded to (not killing) the executor, and
    simulator_execution.transfer_cancellation_acceptance recomputed from the summary; the recorded verdict must agree."""
    from simulator_execution import TRANSFER_CANCELLATION_STATE, transfer_cancellation_acceptance
    run = Path(run_dir)
    paths = _run_files(run)
    out = dict(run=str(run.resolve()), accepted=False, state=None, problems=[], digests={})
    missing = sorted(k for k, path in paths.items() if not path.is_file())
    if missing:
        out['problems'].append('evidence missing: ' + ', '.join(missing))
        return out
    report, session_report, summary = _provenance(out, paths, 'CANCELLED', 'cancelled')
    out['planner'], why = _software(report, out['binding'], repo)
    out['problems'].extend(why)
    out['repo_head'] = (report.get('preflight') or {}).get('repo_head')
    cancel = session_report.get('operator_cancel') or {}
    child = cancel.get('child') or {}
    if (cancel.get('signal') != 'SIGTERM' or child.get('forwarded') != 'SIGTERM' or child.get('killed') is not False
            or report.get('gates', {}).get('full-cycle', {}).get('operator_cancel') != cancel):
        out['problems'].append('operator cancel not forwarded to the running executor')
    accepted = transfer_cancellation_acceptance(summary)
    if not accepted['accepted']:
        out['problems'].append('transfer cancellation not accepted: ' + ', '.join(accepted['failed']))
    if summary.get('transfer_cancellation') != accepted:
        out['problems'].append("the executor's recorded verdict differs from its evidence")
    out.update(accepted=not out['problems'], state=TRANSFER_CANCELLATION_STATE if not out['problems'] else None)
    return out


def _qualified_cell(cell_dir):
    cell_dir = Path(cell_dir)
    files = {name: _sha256((cell_dir / name).read_bytes()) for name in QUALIFIED_CELL_FILES if (cell_dir / name).is_file()}
    if len(files) != len(QUALIFIED_CELL_FILES):
        raise PhysicsScopeRefused('PHYSICS_SCOPE_INCOMPLETE: the cell lacks a bound file')
    return files, cell_world(cell_dir)[1]


def _shared(binding):
    """What every trial of one qualified scope must share exactly (session-specific fields left out)."""
    strip = lambda record: {k: v for k, v in record.items() if k != 'manifest_sha256'}
    return dict(destination=binding['destination'], world=binding['world'],
                destinations=[strip(d) for d in binding['destinations']], support=strip(binding['support']),
                robot={k: binding['robot'][k] for k in STABLE_ROBOT_KEYS},
                cell={k: binding['cell']['files'].get(k) for k in QUALIFIED_CELL_FILES})


def _world_level(shared):
    """What the G5 stop/cancel trials must share with the exact cell's trials: the multi-destination world, its geometry
    and the robot. G5 is once per world in R2.5B (G3 is per bin), so not the task's bin or cell files."""
    return {k: v for k, v in shared.items() if k not in ('destination', 'cell')}


def stop_cancel_evidence(world, native, runtime, cancellation_trial, stop_trial, trial_shared=None, planner=None,
                         repo=REPO):
    """Q7: the accepted G5A cancel and G5B stop-at-cycle-end runs as bound identities, and why either is not. A run of
    another software version is admissible only while its stop/cancel mechanism is unchanged (mechanism_identity)."""
    accepted, why = {}, []
    shared = None if trial_shared is None else _world_level(trial_shared)
    for role, run, judge in (('cancellation', cancellation_trial, cancellation_evidence),
                             ('stop_after_cycle', stop_trial, stop_after_cycle_acceptance)):
        if run is None:
            why.append(f'{role}: no evidence')
            continue
        found = judge(run, repo)
        if not found['accepted']:
            reasons = found.get('problems', []) + found.get('failed', []) + found.get('trial_problems', [])
            why.append(f"{role} {found['run']}: {'; '.join(reasons)}")
            continue
        bound = found['binding']
        wrong = [name for name, ok in (
            ('native identity changed', bound.get('native') == native),
            ('runtime identity changed', bound.get('runtime') == runtime),
            ('another world', bound['world']['derived_sha256'] == world['derived_sha256']),
            ('geometry or robot differs from the other trials', shared is None or _world_level(_shared(bound)) == shared),
        ) if not ok]
        if wrong:
            why.append(f"{role} {found['run']}: {', '.join(wrong)}")
            continue
        compatibility = 'same software'
        if planner is not None and found.get('planner') != planner:
            try:
                unchanged = mechanism_identity(role, repo, found['repo_head']) == mechanism_identity(role, repo)
            except PhysicsScopeRefused as exc:
                why.append(f"{role} {found['run']}: {exc}")
                continue
            if not unchanged:
                why.append(f"{role} {found['run']}: stop/cancel code changed since this run (another software version)")
                continue
            compatibility = 'stop/cancel mechanism unchanged'
        shared = _world_level(_shared(bound))
        accepted[role] = dict(run=found['run'], run_id=found['run_id'], state=found['state'], digests=found['digests'],
                              planner=found.get('planner'), compatibility=compatibility)
    if len(accepted) == 2 and (accepted['cancellation']['run_id'] == accepted['stop_after_cycle']['run_id'] or
                               accepted['cancellation']['digests']['scope'] == accepted['stop_after_cycle']['digests']['scope']):
        why.append('the same trial session given as both stop and cancellation evidence')
    return accepted, why


def _campaign_runs(roots, files, world, planner, repo):
    """Every authorised trial run of the exact cell and software version found under ``roots`` (motion was authorised:
    it has a scope record). Operator-cancelled runs are stop evidence (G5A), not cycle outcomes."""
    found = {}
    for root in roots:
        for report_path in sorted(Path(root).rglob('stage-a1-targeted-report.json')):
            run = report_path.parent
            scope = run / '06-full-cycle/physics-scope.json'
            if not scope.is_file():
                continue
            try:
                report, bound = json.loads(report_path.read_text()), json.loads(scope.read_text())['binding']
                same_cell = ({k: bound['cell']['files'].get(k) for k in QUALIFIED_CELL_FILES} == files and
                             bound['world']['derived_sha256'] == world['derived_sha256'] and
                             bound['destination']['asset'] == world['destination_asset'])
            except (OSError, ValueError, KeyError, TypeError):
                found[str(run.resolve())] = 'unreadable trial evidence'
                continue
            if same_cell and report.get('status') != 'CANCELLED' and _software(report, bound, repo)[0] == planner:
                found[str(run.resolve())] = report.get('status')
    return found


def qualification_readiness(cell_dir, trial_dirs, native, runtime, cancellation_trial=None, stop_trial=None,
                            planner=None, evidence_roots=(), repo=REPO):
    """Whether the listed trials qualify the exact cell's scope under QUALIFICATION_CONTRACT. Never writes anything.
    ``cancellation_trial`` (G5A) and ``stop_trial`` (G5B) are the Q7 evidence; the G5B run counts toward Q8 only when it
    is also listed in ``trial_dirs`` and meets every per-trial criterion there.

    Repeatability counts one software version (``planner``, default this checkout's): a trial of another version is
    listed under ``other_versions`` with its outcome, never counted and never blocking; a failure after motion of this
    version blocks. Every run of this version and cell under ``evidence_roots`` must be listed (no cherry-picking)."""
    planner = planner or planner_identity(repo)
    files, world = _qualified_cell(cell_dir)
    trials = [trial_evidence(run, repo) for run in trial_dirs]
    unmet, reliability, counted, shared, other_versions = [], [], [], None, []
    for trial in trials:
        if trial['kind'] == 'PRE_MOTION_FAILURE':
            reliability.append(dict(run=trial['run'], finding=trial['problems'][0], planner=trial.get('planner')))
            continue
        if trial.get('planner') is not None and trial['planner'] != planner:
            other_versions.append(dict(run=trial['run'], kind=trial['kind'], planner=trial['planner'],
                                       problems=trial['problems']))
            continue
        if trial['kind'] != 'STRICT_PASS':
            unmet.append(f"{trial['run']}: {'; '.join(trial['problems'])}")
            continue
        bound = trial['binding']
        why = [name for name, ok in (
            ('native identity changed', bound.get('native') == native),
            ('runtime identity changed', bound.get('runtime') == runtime),
            ('another cell', {k: bound['cell']['files'].get(k) for k in QUALIFIED_CELL_FILES} == files),
            ('another world', bound['world']['derived_sha256'] == world['derived_sha256']),
            ('another bin', bound['destination']['asset'] == world['destination_asset']),
        ) if not ok]
        if why:
            unmet.append(f"{trial['run']}: {', '.join(why)}")
            continue
        if shared is None:
            shared = _shared(bound)
        elif _shared(bound) != shared:
            unmet.append(f"{trial['run']}: geometry, robot or cell differs from the other trials")
            continue
        if any(trial['run_id'] == c['run_id'] or trial['digests']['scope'] == c['digests']['scope'] for c in counted):
            unmet.append(f"{trial['run']}: the same trial session counted twice")
            continue
        counted.append(dict(run=trial['run'], run_id=trial['run_id'], digests=trial['digests']))
    required = QUALIFICATION_CONTRACT['strict_trials']
    if len(counted) < required:
        unmet.append(f'{len(counted)} of {required} independent strict trial PASS sessions')
    stop_cancel, why = stop_cancel_evidence(world, native, runtime, cancellation_trial, stop_trial, shared, planner,
                                            repo)
    if why:
        unmet.append('stop/cancel acceptance not established: ' + QUALIFICATION_CONTRACT['stop_cancel'] +
                     ('' if cancellation_trial is None and stop_trial is None else ' (' + '; '.join(why) + ')'))
    listed = {str(Path(run).resolve()) for run in [*trial_dirs, cancellation_trial, stop_trial] if run is not None}
    unlisted = {run: status for run, status in _campaign_runs(evidence_roots, files, world, planner, repo).items()
                if run not in listed}
    if unlisted:
        unmet.append('unlisted trials of this software version: ' +
                     ', '.join(f'{run} ({status})' for run, status in sorted(unlisted.items())))
    return dict(status='NOT_QUALIFIED' if unmet else 'QUALIFIED_READY', contract=QUALIFICATION_CONTRACT,
                cell=files, world=world['derived_sha256'], destination=world['destination_asset'],
                native=native, runtime=runtime, planner=planner, trials=counted,
                stop_cancel=stop_cancel if not why else {}, unmet=unmet, reliability_findings=reliability,
                other_versions=other_versions, unlisted=sorted(unlisted))


def issue_qualified_scope(path, cell_dir, trial_dirs, native, runtime, cancellation_trial, stop_trial, planner=None,
                          evidence_roots=(), repo=REPO):
    """Write the QUALIFIED record of the exact cell only if every Q1-Q8 requirement holds on the evidence now; return its
    sha256. It permits exactly TRIAL_OPERATION into the cell's bin, for one software version. Nothing consumes it: the
    executor, Builder and HMI admit no qualified record (runtime admission is G6). An existing record is never replaced."""
    readiness = qualification_readiness(cell_dir, trial_dirs, native, runtime, cancellation_trial, stop_trial, planner,
                                        evidence_roots, repo)
    if readiness['status'] != 'QUALIFIED_READY':
        raise PhysicsScopeRefused('PHYSICS_SCOPE_NOT_QUALIFIED: ' + '; '.join(readiness['unmet']))
    body = dict(schema=SCHEMA, state=QUALIFIED, contract=QUALIFICATION_CONTRACT, operation=TRIAL_OPERATION,
                trials=readiness['trials'], **{k: readiness[k] for k in QUALIFIED_RECORD_KEYS})
    data = (json.dumps(dict(body, binding_sha256=_sha256(_canonical(body))), indent=2, sort_keys=True) + '\n').encode()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('xb') as stream:                     # refuses an existing record
        stream.write(data)
    return _sha256(data)


QUALIFIED_RECORD_KEYS = ('cell', 'world', 'destination', 'native', 'runtime', 'planner', 'stop_cancel')


def verify_qualified_scope(path, sha256, cell_dir, native, runtime, planner=None, evidence_roots=(), repo=REPO):
    """The QUALIFIED record at ``path`` if every bound trial still qualifies the exact cell now; else refuse."""
    data = Path(path).read_bytes()
    if not sha256 or _sha256(data) != sha256:
        raise PhysicsScopeRefused('PHYSICS_SCOPE_RECORD_CHANGED: the qualified record differs from the issued one')
    record = json.loads(data)
    if record.get('schema') != SCHEMA or record.get('state') != QUALIFIED:
        raise PhysicsScopeRefused(f"PHYSICS_SCOPE_UNSUPPORTED: not a qualified scope ({record.get('state')!r})")
    if record.get('contract') != QUALIFICATION_CONTRACT:
        raise PhysicsScopeRefused('PHYSICS_SCOPE_UNSUPPORTED: issued under another qualification contract')
    if record.get('operation') != TRIAL_OPERATION:
        raise PhysicsScopeRefused('PHYSICS_SCOPE_UNSUPPORTED: the record permits another operation than one full-cycle')
    body = {k: v for k, v in record.items() if k != 'binding_sha256'}
    if record.get('binding_sha256') != _sha256(_canonical(body)):
        raise PhysicsScopeRefused('PHYSICS_SCOPE_RECORD_CHANGED: binding digest mismatch')
    bound = record.get('trials') or []
    if len({t.get('run_id') for t in bound}) != len(bound) or len({t.get('run') for t in bound}) != len(bound):
        raise PhysicsScopeRefused('PHYSICS_SCOPE_REPLAYED: a trial session is bound twice')
    for trial in bound:
        current = trial_evidence(trial['run'], repo)
        if current.get('digests') != trial.get('digests') or current.get('run_id') != trial.get('run_id'):
            raise PhysicsScopeRefused(f"PHYSICS_SCOPE_EVIDENCE_CHANGED: {trial.get('run')}")
    # The G5 runs are re-judged now; their digests and session ids must equal the bound ones.
    stop_cancel = record.get('stop_cancel') or {}
    readiness = qualification_readiness(cell_dir, [t['run'] for t in bound], native, runtime,
                                        cancellation_trial=(stop_cancel.get('cancellation') or {}).get('run'),
                                        stop_trial=(stop_cancel.get('stop_after_cycle') or {}).get('run'),
                                        planner=planner, evidence_roots=evidence_roots, repo=repo)
    for key in QUALIFIED_RECORD_KEYS:
        if record.get(key) != readiness[key]:
            raise PhysicsScopeRefused(f'PHYSICS_SCOPE_MISMATCH: {key}')
    if readiness['status'] != 'QUALIFIED_READY':
        raise PhysicsScopeRefused('PHYSICS_SCOPE_NOT_QUALIFIED: ' + '; '.join(readiness['unmet']))
    return record


def main(argv=None):
    """Offline: readiness (display), issue (write the QUALIFIED record only when ready) or verify (an issued record).
    python3 physics_scope_qualification.py readiness|issue --cell DIR --trial RUN... --cancellation-trial RUN
    --stop-trial RUN [--evidence-root DIR...] [--output RECORD]; verify --cell DIR --record RECORD --sha256 SHA"""
    import argparse
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument('command', choices=['readiness', 'issue', 'verify'])
    parser.add_argument('--cell', type=Path, required=True)
    parser.add_argument('--trial', type=Path, action='append', default=[])
    parser.add_argument('--cancellation-trial', type=Path, help='Q7: the G5A run (operator cancel during the transfer)')
    parser.add_argument('--stop-trial', type=Path, help='Q7: the G5B run (stop at cycle end); list it as --trial too for Q8')
    parser.add_argument('--evidence-root', type=Path, action='append',
                        help='scanned for unlisted runs of this cell and software version (default: the workspace '
                             'qualification evidence directories)')
    parser.add_argument('--output', type=Path, help='issue: the new QUALIFIED record (never replaced)')
    parser.add_argument('--record', type=Path, help='verify: the issued record')
    parser.add_argument('--sha256', help='verify: the sha256 printed when the record was issued')
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[3])
    args = parser.parse_args(argv)
    if (args.command == 'issue') != (args.output is not None) or (args.command == 'verify') != (args.record is not None):
        parser.error('issue needs --output; verify needs --record and --sha256')
    roots = args.evidence_root if args.evidence_root is not None else [
        root for root in (args.workspace.parent / 'log/qualification', args.workspace.parent / 'r25b_multibin_20261029')
        if root.is_dir()]
    try:
        native = native_identity(args.workspace / 'install/workcell_builder/share/workcell_builder/'
                                 'commission_execute_build.json')
        runtime = runtime_identity(args.workspace)
        if args.command == 'issue':
            sha = issue_qualified_scope(args.output, args.cell, args.trial, native, runtime, args.cancellation_trial,
                                        args.stop_trial, evidence_roots=roots)
            result = dict(status='ISSUED', record=str(args.output), sha256=sha, admission='none (runtime admission is G6)')
        elif args.command == 'verify':
            record = verify_qualified_scope(args.record, args.sha256, args.cell, native, runtime, evidence_roots=roots)
            result = dict(status='VERIFIED', record=str(args.record), destination=record['destination'],
                          operation=record['operation'], planner=record['planner'],
                          trials=[t['run'] for t in record['trials']],
                          stop_cancel={k: v['run'] for k, v in record['stop_cancel'].items()},
                          admission='none (runtime admission is G6)')
        else:
            result = qualification_readiness(args.cell, args.trial, native, runtime, args.cancellation_trial,
                                             args.stop_trial, evidence_roots=roots)
    except PhysicsScopeRefused as exc:
        result = dict(status='REFUSED', reason=str(exc))
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['status'] in ('QUALIFIED_READY', 'ISSUED', 'VERIFIED') else 2


if __name__ == '__main__':
    import sys
    sys.exit(main())
