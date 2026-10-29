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
    if any((o.get('pose') or {}).get('frame_id') != 'world' for o in observations.get('objects', [])):
        raise PhysicsScopeRefused('PHYSICS_SCOPE_UNSUPPORTED: measured object poses must be in the world frame')
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


def trial_evidence(run_dir):
    """One authorised trial, judged from its own evidence files: kind STRICT_PASS, PRE_MOTION_FAILURE or FAILED."""
    run, contract = Path(run_dir), QUALIFICATION_CONTRACT
    session = run / '06-full-cycle'
    paths = dict(report=run / 'stage-a1-targeted-report.json', session_report=session / 'session-report.json',
                 scope=session / 'physics-scope.json', summary=session / 'full-cycle/summary.json')
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
        return dict(out, kind='PRE_MOTION_FAILURE', problems=[why])
    missing = sorted(k for k, path in paths.items() if not path.is_file())
    if missing:
        out['problems'].append('evidence missing: ' + ', '.join(missing))
        return out
    out['digests'] = {k: _sha256(path.read_bytes()) for k, path in paths.items()}
    session_report, record, summary = (json.loads(paths[k].read_text()) for k in ('session_report', 'scope', 'summary'))
    bound = record.get('binding') or {}
    out.update(binding=bound, run_id=(bound.get('session') or {}).get('run_id'))
    problems = out['problems']
    if (report.get('status') != 'PASS' or report.get('scope') != TRIAL_EVIDENCE_SCOPE or report.get('mode') != 'targeted'
            or report.get('gate') != 'full-cycle' or report.get('stage_a_acceptance') is not False):
        problems.append('not a passing authorised trial report')
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
    accepted = summary.get('full_cycle_physical_acceptance') or {}
    release = summary.get('release_evidence') or {}
    prerequisites = summary.get('commissioning_prerequisites') or {}
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
    if (sorted(prerequisites.get('results') or []) != PREREQUISITE_RESULTS or
            prerequisites.get('capability_sha256') != (bound.get('native') or {}).get('sha256') or
            prerequisites.get('sources') != (bound.get('native') or {}).get('sources')):
        problems.append('Stage-A prerequisite trials not bound to the trial build')
    # The measured part rests inside the selected bin, its lowest corner on the bound floor.
    try:
        bin_ = next(d for d in bound['destinations'] if d['model'] == bound['destination']['model'])
        part = next(o for o in bound['world']['objects']
                    if 'runtime::' + o['object_id'] == summary.get('selected_object_id'))
        pose = accepted['final_pose']
        bottom = min(pose[2] + _rotate(pose[3:], [sx * part['dimensions_m'][0] / 2, sy * part['dimensions_m'][1] / 2,
                                                  sz * part['dimensions_m'][2] / 2])[2]
                     for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1))
        inside = all(abs(pose[i] - bin_['floor_xyz'][i]) <= bin_['floor_size_m'][i] / 2 for i in (0, 1))
        if not inside or abs(bottom - bin_['floor_z']) > contract['seat_tolerance_m']:
            problems.append('measured part not seated on the selected bin floor')
    except (KeyError, StopIteration, TypeError, IndexError):
        problems.append('measured final pose or selected bin not recorded')
    if not problems:
        out['kind'] = 'STRICT_PASS'
    return out


def stop_after_cycle_acceptance(run_dir):
    """G5B: an operator Stop at Cycle End (SIGUSR1 to the runner) latched during the authorised trial's transfer, the
    cycle completed as a strict trial and nothing else started. Judged only from the trial's own evidence files."""
    run = Path(run_dir)
    trial = trial_evidence(run)
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
                checks=checks, trial_problems=trial['problems'])


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


def qualification_readiness(cell_dir, trial_dirs, native, runtime):
    """Whether the listed trials qualify the exact cell's scope under QUALIFICATION_CONTRACT. Never writes anything."""
    files, world = _qualified_cell(cell_dir)
    trials = [trial_evidence(run) for run in trial_dirs]
    unmet, reliability, counted, shared = [], [], [], None
    for trial in trials:
        if trial['kind'] == 'PRE_MOTION_FAILURE':
            reliability.append(dict(run=trial['run'], finding=trial['problems'][0]))
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
    unmet.append('stop/cancel acceptance not established: ' + QUALIFICATION_CONTRACT['stop_cancel'])
    return dict(status='NOT_QUALIFIED' if unmet else 'QUALIFIED_READY', contract=QUALIFICATION_CONTRACT,
                cell=files, world=world['derived_sha256'], destination=world['destination_asset'],
                native=native, runtime=runtime, trials=counted, unmet=unmet, reliability_findings=reliability)


def verify_qualified_scope(path, sha256, cell_dir, native, runtime):
    """The QUALIFIED record at ``path`` if every bound trial still qualifies the exact cell now; else refuse."""
    data = Path(path).read_bytes()
    if not sha256 or _sha256(data) != sha256:
        raise PhysicsScopeRefused('PHYSICS_SCOPE_RECORD_CHANGED: the qualified record differs from the issued one')
    record = json.loads(data)
    if record.get('schema') != SCHEMA or record.get('state') != QUALIFIED:
        raise PhysicsScopeRefused(f"PHYSICS_SCOPE_UNSUPPORTED: not a qualified scope ({record.get('state')!r})")
    if record.get('contract') != QUALIFICATION_CONTRACT:
        raise PhysicsScopeRefused('PHYSICS_SCOPE_UNSUPPORTED: issued under another qualification contract')
    body = {k: v for k, v in record.items() if k != 'binding_sha256'}
    if record.get('binding_sha256') != _sha256(_canonical(body)):
        raise PhysicsScopeRefused('PHYSICS_SCOPE_RECORD_CHANGED: binding digest mismatch')
    bound = record.get('trials') or []
    if len({t.get('run_id') for t in bound}) != len(bound) or len({t.get('run') for t in bound}) != len(bound):
        raise PhysicsScopeRefused('PHYSICS_SCOPE_REPLAYED: a trial session is bound twice')
    for trial in bound:
        current = trial_evidence(trial['run'])
        if current.get('digests') != trial.get('digests') or current.get('run_id') != trial.get('run_id'):
            raise PhysicsScopeRefused(f"PHYSICS_SCOPE_EVIDENCE_CHANGED: {trial.get('run')}")
    readiness = qualification_readiness(cell_dir, [t['run'] for t in bound], native, runtime)
    for key in ('cell', 'world', 'destination', 'native', 'runtime'):
        if record.get(key) != readiness[key]:
            raise PhysicsScopeRefused(f'PHYSICS_SCOPE_MISMATCH: {key}')
    if readiness['status'] != 'QUALIFIED_READY':
        raise PhysicsScopeRefused('PHYSICS_SCOPE_NOT_QUALIFIED: ' + '; '.join(readiness['unmet']))
    return record


def main(argv=None):
    """Offline readiness display: python3 physics_scope_qualification.py readiness --cell DIR --trial RUN..."""
    import argparse
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument('command', choices=['readiness'])
    parser.add_argument('--cell', type=Path, required=True)
    parser.add_argument('--trial', type=Path, action='append', default=[])
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[3])
    args = parser.parse_args(argv)
    try:
        native = native_identity(args.workspace / 'install/workcell_builder/share/workcell_builder/'
                                 'commission_execute_build.json')
        readiness = qualification_readiness(args.cell, args.trial, native, runtime_identity(args.workspace))
    except PhysicsScopeRefused as exc:
        readiness = dict(status='REFUSED', reason=str(exc))
    print(json.dumps(readiness, indent=2, sort_keys=True))
    return 0 if readiness['status'] == 'QUALIFIED_READY' else 2


if __name__ == '__main__':
    import sys
    sys.exit(main())
