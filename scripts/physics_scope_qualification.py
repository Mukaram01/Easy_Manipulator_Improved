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


def planner_identity(repo=REPO, revision=None):
    """The software identity of ``revision`` (default: this checkout's HEAD, whose PLANNER_SOURCES must be unmodified):
    sha256 over the git blob id of every file in PLANNER_SOURCES. Historical trials are recovered from their recorded
    clean commit, so no evidence file ever changes."""
    if revision is None:
        if _git(repo, 'status', '--porcelain', '--', *PLANNER_SOURCES).strip():
            raise PhysicsScopeRefused('PHYSICS_SCOPE_SOFTWARE_MODIFIED: planner or executor sources differ from HEAD')
        revision = 'HEAD'
    blobs = {}
    for line in _git(repo, 'ls-tree', '-r', revision, '--', *PLANNER_SOURCES).splitlines():
        _, kind, blob, path = line.split(None, 3)
        if kind == 'blob':
            blobs[path] = blob
    if not blobs:
        raise PhysicsScopeRefused(f'PHYSICS_SCOPE_PROVENANCE_UNRECOVERABLE: no planner sources at {revision}')
    return _sha256(_canonical(blobs))


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
    if not (accepted.get('placement') or {}).get('error_to_planned_m', 1.0) <= contract['placement_tolerance_m']:
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
