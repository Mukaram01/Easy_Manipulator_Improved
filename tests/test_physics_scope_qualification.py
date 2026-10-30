"""G3A: a workcell_physics_scope/v1 trial record admits one full-cycle into exactly the bound bin of the exact cell,
world, robot, runtime and evidence it was authorised for; every changed, missing or substituted binding refuses."""
import copy
import hashlib
import json
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import destination_binding as db  # noqa: E402
import physics_scope_qualification as psq  # noqa: E402
import simulator_world as sw  # noqa: E402

pytestmark = pytest.mark.skipif(not Path('/opt/ros/humble/bin/xacro').exists(), reason='Real Humble xacro required')

NATIVE = {'sha256': 'commission', 'support_sha256': 'support', 'telemetry_sha256': 'telemetry', 'moveit_version': '2.5.10',
          'sources': {'/ws/src_support_contact_adapter.cpp': 'adapter'}}
RUNTIME = {'record_sha256': 'record', 'stage_a_evidence': '/evidence/stage_a', 'stage_a_report_sha256': 'report',
           'stage_a_repo_head': 'head'}
# Software identity: the committed HEAD stands for "this checkout" (the working tree may be mid-edit); the clean-tree
# check itself is tested on a scratch repository below.
HEAD = psq._git(psq.REPO, 'rev-parse', 'HEAD').strip()
PLANNER = psq.planner_identity(psq.REPO, HEAD)
CONTROL = psq.g4_control_identity(psq.REPO, HEAD)


@pytest.fixture(autouse=True)
def committed_software(monkeypatch):
    planner, mechanism, control = psq.planner_identity, psq.mechanism_identity, psq.g4_control_identity
    monkeypatch.setattr(psq, 'planner_identity', lambda repo=psq.REPO, revision=None: planner(repo, revision or HEAD))
    monkeypatch.setattr(psq, 'g4_control_identity', lambda repo=psq.REPO, revision=None: control(repo, revision or HEAD))
    monkeypatch.setattr(psq, 'mechanism_identity',
                        lambda role, repo=psq.REPO, revision=None: mechanism(role, repo, revision or HEAD))


@pytest.fixture(scope='module')
def cells(tmp_path_factory):
    from r25b_cells import author_two_bin_cell
    root = tmp_path_factory.mktemp('g3a')
    a, rc_a = author_two_bin_cell(root, 'two_bin_a', destination='a')
    b, rc_b = author_two_bin_cell(root, 'two_bin_b', destination='b')
    assert rc_a == rc_b == 0
    return {'a': a, 'b': b}


def launch(tmp_path, cell, edit_world=None, occupant=None):
    """A session as the qualified launch leaves it: launched world, receipt with every bin bound, measured parts."""
    text, report = sw.derive_world(cell, physical_destinations='all')
    runtime = tmp_path / 'runtime'
    runtime.mkdir(parents=True)
    (runtime / 'world.sdf').write_text(edit_world(text) if edit_world else text)
    (runtime / 'spec.json').write_text('{}')
    db.augment_spec(runtime, cell / 'config/moveit_collision_objects.yaml', True)
    receipt = json.loads((runtime / 'spec.json').read_text())
    receipt.update(world='a0', run_id='run-1', world_sha256=hashlib.sha256((runtime / 'world.sdf').read_bytes()).hexdigest(),
                   support_geometry_binding=dict(model='pick_support', link='support_link', collision='support_collision',
                                                 object_id='workcell::support_surface_table', manifest_path='/session/x'),
                   expected_controllers=['ur5_arm_controller', 'ur5_gripper_controller'],
                   end_effector_hold={'capability_id': 'robotiq_2f_85', 'sha256': 'hold'},
                   native_mimics=[{'follower': 'f2', 'driver': 'f1', 'multiplier': 1.0}],
                   **{k: 'id-' + k for k in psq.ROBOT_RECEIPT_KEYS
                      if k not in ('expected_controllers', 'end_effector_hold', 'native_mimics')})
    (runtime / 'receipt.json').write_text(json.dumps(receipt))
    parts = [dict(object_id=o['object_id'], dimensions=o['dimensions_m'],
                  pose=dict(frame_id='world', xyz=o['spawn_pose'][:3], rpy=o['spawn_pose'][3:])) for o in report['objects']]
    if occupant:
        parts.append(dict(object_id='occupant', dimensions=[0.025] * 3, pose=dict(frame_id='world', xyz=occupant, rpy=[0, 0, 0])))
    (tmp_path / 'parts.yaml').write_text(yaml.safe_dump({'objects': parts}))
    (tmp_path / 'commission-evidence.json').write_text('[{"result": "CANCELLATION_TRIAL_PASS"}]')
    return tmp_path


def binding(session, cell, native=NATIVE, runtime=RUNTIME, resolution='resolution', planner=PLANNER):
    return psq.scope_binding(cell_dir=cell, receipt_path=session / 'runtime/receipt.json',
                             observations_path=session / 'parts.yaml', native=native, runtime=runtime,
                             commission_evidence_path=session / 'commission-evidence.json', resolution_sha256=resolution,
                             planner=planner)


def rewrite(path, edit):
    """An edited record with both of its digests recomputed (what a forger would do)."""
    record = json.loads(path.read_text())
    edit(record)
    record['binding_sha256'] = hashlib.sha256(psq._canonical(record['binding'])).hexdigest()
    data = (json.dumps(record) + '\n').encode()
    path.write_bytes(data)
    return hashlib.sha256(data).hexdigest()


BINS = [('a', 'target_bin_default'), ('b', 'target_bin_b')]      # the same world; each cell's task selects its bin


@pytest.mark.parametrize('cell,asset', BINS)
def test_authorised_trial_verifies_against_the_same_recomputed_binding(cells, tmp_path, cell, asset):
    session = launch(tmp_path, cells[cell])
    bound = binding(session, cells[cell])
    sha = psq.authorise_trial(tmp_path / 'scope.json', bound)
    record = psq.verify_trial(tmp_path / 'scope.json', sha, binding(session, cells[cell]))
    assert record['state'] == 'AUTHORISED_TRIAL' and record['operation'] == psq.TRIAL_OPERATION
    assert record['binding']['destination']['asset'] == asset
    assert record['binding']['destination']['model'] == db.select_destination_binding(
        json.loads((session / 'runtime/receipt.json').read_text())['destination_geometry_bindings'], asset)['model']
    assert len(record['binding']['destinations']) == 2                                   # the other bin is bound too
    assert all('manifest_path' not in d for d in record['binding']['destinations'])     # content bound, not paths


@pytest.mark.parametrize('cell,asset', BINS)
def test_a_trial_for_one_bin_never_admits_the_other(cells, tmp_path, cell, asset):
    other_cell = 'a' if cell == 'b' else 'b'
    session = launch(tmp_path, cells[cell])
    sha = psq.authorise_trial(tmp_path / 'scope.json', binding(session, cells[cell]))
    # The same world with the task pointed at the other bin: the destination (and the cell) no longer match.
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: .*destination'):
        psq.verify_trial(tmp_path / 'scope.json', sha,
                         binding(launch(tmp_path / other_cell, cells[other_cell]), cells[other_cell]))
    other = next(d for d in binding(session, cells[cell])['destinations'] if d['source_item_id'] != asset)
    forged = rewrite(tmp_path / 'scope.json', lambda r: r['binding']['destination'].update(
        model=other['model'], object_id=other['object_id'], floor_z=other['floor_z']))
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: destination'):
        psq.verify_trial(tmp_path / 'scope.json', forged, binding(session, cells[cell]))


def test_changed_bin_geometry_refuses(cells, tmp_path):
    session = launch(tmp_path / 'one', cells['b'])
    sha = psq.authorise_trial(tmp_path / 'scope.json', binding(session, cells['b']))
    receipt = json.loads((session / 'runtime/receipt.json').read_text())
    receipt['destination_geometry_bindings'][0]['floor_z'] += 0.001
    (session / 'runtime/receipt.json').write_text(json.dumps(receipt))
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH'):
        psq.verify_trial(tmp_path / 'scope.json', sha, binding(session, cells['b']))
    # A launched bin that is not the cell's derived bin is refused before any record is written.
    moved = launch(tmp_path / 'two', cells['b'], edit_world=lambda text: text.replace(
        '<size>0.3200500719225441', '<size>0.3300500719225441', 1))
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_WORLD_MISMATCH'):
        binding(moved, cells['b'])


def test_source_build_or_runtime_change_refuses(cells, tmp_path):
    session = launch(tmp_path, cells['b'])
    sha = psq.authorise_trial(tmp_path / 'scope.json', binding(session, cells['b']))
    rebuilt = copy.deepcopy(NATIVE)
    rebuilt['sources']['/ws/src_support_contact_adapter.cpp'] = 'repaired-adapter'
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: native'):
        psq.verify_trial(tmp_path / 'scope.json', sha, binding(session, cells['b'], native=rebuilt))
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: runtime'):
        psq.verify_trial(tmp_path / 'scope.json', sha,
                         binding(session, cells['b'], runtime=dict(RUNTIME, stage_a_report_sha256='other')))
    # The native identity itself is proved against the bytes on disk.
    libs = {name: tmp_path / f'{name}.so' for name in ('library', 'support_library', 'telemetry_library')}
    for path in libs.values():
        path.write_bytes(path.name.encode())
    source = tmp_path / 'src_support_contact_adapter.cpp'
    source.write_text('qualified')
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = tmp_path / 'manifest.json'
    manifest.write_text(json.dumps(dict({k: str(v) for k, v in libs.items()}, sha256=digest(libs['library']),
        support_sha256=digest(libs['support_library']), telemetry_sha256=digest(libs['telemetry_library']),
        moveit_version='2.5.10', sources={str(source): digest(source)})))
    assert psq.native_identity(manifest)['support_sha256'] == digest(libs['support_library'])
    libs['support_library'].write_bytes(b'rebuilt support library')          # commission library unchanged
    with pytest.raises(psq.PhysicsScopeRefused, match='support_library differs'):
        psq.native_identity(manifest)
    libs['support_library'].write_bytes(b'support_library.so')
    source.write_text('edited')
    with pytest.raises(psq.PhysicsScopeRefused, match='bound source changed'):
        psq.native_identity(manifest)


def test_missing_or_changed_evidence_refuses(cells, tmp_path):
    session = launch(tmp_path, cells['b'])
    sha = psq.authorise_trial(tmp_path / 'scope.json', binding(session, cells['b']))
    (session / 'commission-evidence.json').write_text('[]')
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: evidence'):
        psq.verify_trial(tmp_path / 'scope.json', sha, binding(session, cells['b']))
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_INCOMPLETE: plan resolution'):
        binding(session, cells['b'], resolution=None)
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_RUNTIME_UNQUALIFIED'):
        psq.runtime_identity(tmp_path / 'no_registered_runtime')


def test_record_integrity_state_and_operation(cells, tmp_path):
    session = launch(tmp_path, cells['b'])
    current = binding(session, cells['b'])
    path = tmp_path / 'scope.json'
    sha = psq.authorise_trial(path, current)
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_RECORD_CHANGED'):
        psq.verify_trial(path, '0' * 64, current)
    record = json.loads(path.read_text())
    record['binding']['destination']['floor_z'] = 0.5                     # edited without the binding digest
    path.write_text(json.dumps(record))
    with pytest.raises(psq.PhysicsScopeRefused, match='binding digest mismatch'):
        psq.verify_trial(path, hashlib.sha256(path.read_bytes()).hexdigest(), current)
    for edit, match in ((lambda r: r.update(state='QUALIFIED'), "scope state 'QUALIFIED'"),
                        (lambda r: r['operation'].update(cycles=2), 'another operation'),
                        (lambda r: r['operation'].update(real_hardware=True), 'another operation'),
                        (lambda r: r.update(schema='workcell_physics_scope/v0'), 'unknown scope schema')):
        psq.authorise_trial(path, current)
        with pytest.raises(psq.PhysicsScopeRefused, match=match):
            psq.verify_trial(path, rewrite(path, edit), current)
    assert psq.verify_trial(path, psq.authorise_trial(path, current), current)


def test_an_occupied_destination_or_a_single_bin_world_is_unsupported(cells, tmp_path):
    receipt = launch(tmp_path / 'clear', cells['b']) / 'runtime/receipt.json'
    floor = db.select_destination_binding(json.loads(receipt.read_text())['destination_geometry_bindings'],
                                          'target_bin_b')['floor_xyz']
    occupied = launch(tmp_path / 'occupied', cells['b'], occupant=[floor[0], floor[1], 0.02])
    with pytest.raises(psq.PhysicsScopeRefused, match="PHYSICS_SCOPE_DESTINATION_OCCUPIED: \\['occupant'\\]"):
        binding(occupied, cells['b'])
    single = launch(tmp_path / 'single', cells['b'])
    data = json.loads((single / 'runtime/receipt.json').read_text())
    data.update(destination_geometry_binding=data['destination_geometry_bindings'][0], destination_geometry_bindings=None)
    (single / 'runtime/receipt.json').write_text(json.dumps(data))
    with pytest.raises(psq.PhysicsScopeRefused, match='does not bind a multi-destination world'):
        binding(single, cells['b'])


# --- Qualification (offline): trials are judged from their evidence and never qualify themselves ------------------
def strict_trial(root, cell, run_id, *, cell_dir=None, native=NATIVE, edit=None, head=HEAD):
    """A run directory laid out like the qualification runner's, for one strict trial PASS session."""
    session = launch(root / 'launch', cell)
    receipt = json.loads((session / 'runtime/receipt.json').read_text())
    receipt['run_id'] = run_id
    (session / 'runtime/receipt.json').write_text(json.dumps(receipt))
    bound = binding(session, cell, native=native, planner=psq.planner_identity(psq.REPO, head))
    run = root / 'run'
    scope = run / '06-full-cycle/physics-scope.json'
    scope.parent.mkdir(parents=True)
    sha = psq.authorise_trial(scope, bound)
    selected = next(d for d in bound['destinations'] if d['model'] == bound['destination']['model'])
    pose = [selected['floor_xyz'][0], selected['floor_xyz'][1], selected['floor_z'] + 0.0125, 0.0, 0.0, 0.0, 1.0]
    summary = dict(result='PASS', strict_retention_compliant=True, retention_policy='strict',
                   retention_findings={'findings': []}, stages=psq.FULL_CYCLE_STAGES, selected_object_id='runtime::part_04',
                   release_evidence={'state': 'RELEASE_CONFIRMED', 'settled': True}, physics_scope={'sha256': sha},
                   full_cycle_physical_acceptance=dict(attached_ids=[], baseline_acm_restored=True, final_collision_valid=True,
                                                       final_pose=pose, placement={'error_to_planned_m': 0.0001}),
                   commissioning_prerequisites=dict(results=psq.PREREQUISITE_RESULTS, capability_sha256=native['sha256'],
                                                    sources=native['sources']))
    report = dict(status='PASS', scope=psq.TRIAL_EVIDENCE_SCOPE, mode='targeted', gate='full-cycle',
                  stage_a_acceptance=False, preflight={'repo_dirty': False, 'repo_head': head})
    if edit:
        edit(summary, report)
    (run / '06-full-cycle/full-cycle').mkdir()
    (run / '06-full-cycle/full-cycle/summary.json').write_text(json.dumps(summary))
    (run / '06-full-cycle/session-report.json').write_text(json.dumps({'physics_scope': {'sha256': sha},
                                                                       'shutdown': {'clean': True}}))
    (run / 'stage-a1-targeted-report.json').write_text(json.dumps(report))
    return run


@pytest.fixture(scope='module')
def trials(cells, tmp_path_factory):
    root = tmp_path_factory.mktemp('trials')
    return [strict_trial(root / str(i), cells['b'], f'run-{i}') for i in range(3)]


def qualified_record(path, readiness, **change):
    """A QUALIFIED record as an issuer would write it (no issuer exists: qualification is not reachable yet)."""
    body = dict(schema=psq.SCHEMA, state=psq.QUALIFIED, contract=psq.QUALIFICATION_CONTRACT, operation=psq.TRIAL_OPERATION,
                trials=readiness['trials'], **{k: readiness[k] for k in psq.QUALIFIED_RECORD_KEYS})
    body.update(change)
    data = (json.dumps(dict(body, binding_sha256=hashlib.sha256(psq._canonical(body)).hexdigest())) + '\n').encode()
    path.write_bytes(data)
    return hashlib.sha256(data).hexdigest()


def test_strict_trials_never_qualify_themselves(cells, trials):
    one = psq.qualification_readiness(cells['b'], trials[:1], NATIVE, RUNTIME)
    assert one['status'] == 'NOT_QUALIFIED' and '1 of 3 independent strict trial PASS sessions' in one['unmet']
    three = psq.qualification_readiness(cells['b'], trials, NATIVE, RUNTIME)
    assert len(three['trials']) == 3                                       # every per-trial criterion holds
    assert three['status'] == 'NOT_QUALIFIED'                              # stop/cancel acceptance is still missing
    assert three['unmet'] == ['stop/cancel acceptance not established: ' + psq.QUALIFICATION_CONTRACT['stop_cancel']]


@pytest.mark.parametrize('edit,match', [
    (lambda s, r: s.update(result='FUNCTIONAL_PASS'), 'not a strict full-cycle PASS'),
    (lambda s, r: s['retention_findings'].update(findings=[{'phase': 'transfer'}]), 'not a strict full-cycle PASS'),
    (lambda s, r: s.update(stages=psq.FULL_CYCLE_STAGES[:-1]), 'incomplete stage sequence'),
    (lambda s, r: s['release_evidence'].update(settled=False), 'release not confirmed and settled'),
    (lambda s, r: s['full_cycle_physical_acceptance'].update(attached_ids=['runtime::part_04']), 'not restored'),
    (lambda s, r: s['full_cycle_physical_acceptance']['final_pose'].__setitem__(2, 0.0145), 'not seated'),
    (lambda s, r: s['full_cycle_physical_acceptance']['placement'].update(error_to_planned_m=0.004), 'placement error'),
    (lambda s, r: s['commissioning_prerequisites'].update(results=psq.PREREQUISITE_RESULTS[1:]), 'prerequisite'),
    (lambda s, r: s['physics_scope'].update(sha256='other'), 'differs from the one the trial verified'),
    (lambda s, r: r.update(preflight={'repo_dirty': True}), 'modified checkout'),
    (lambda s, r: r.update(scope='DIAGNOSTIC_NOT_STAGE_A_ACCEPTANCE'), 'not a passing authorised trial report'),
])
def test_a_trial_missing_any_physical_or_provenance_criterion_does_not_count(cells, tmp_path, edit, match):
    run = strict_trial(tmp_path, cells['b'], 'run-x', edit=edit)
    readiness = psq.qualification_readiness(cells['b'], [run], NATIVE, RUNTIME)
    assert readiness['trials'] == [] and any(match in u for u in readiness['unmet'])


def test_stale_cross_bin_and_replayed_trials_do_not_count(cells, trials, tmp_path):
    stale = psq.qualification_readiness(cells['b'], trials, dict(NATIVE, support_sha256='rebuilt'), RUNTIME)
    assert stale['trials'] == [] and all('native identity changed' in u for u in stale['unmet'][:3])
    other = psq.qualification_readiness(cells['a'], trials, NATIVE, RUNTIME)            # Bin B trials for the Bin A cell
    assert other['trials'] == [] and all('another cell' in u and 'another bin' in u for u in other['unmet'][:3])
    twice = psq.qualification_readiness(cells['b'], [trials[0], trials[0], trials[1]], NATIVE, RUNTIME)
    assert len(twice['trials']) == 2 and any('counted twice' in u for u in twice['unmet'])
    blocked = tmp_path / 'blocked'
    (blocked / '06-full-cycle/resolve').mkdir(parents=True)
    (blocked / 'stage-a1-targeted-report.json').write_text(json.dumps({'status': 'BLOCKED', 'failure': 'command failed'}))
    (blocked / '06-full-cycle/resolve/summary.json').write_text(json.dumps(
        {'failed_stage': 'PREPLAN_TRANSFER', 'failure': 'SEARCH_BUDGET_EXHAUSTED: candidate search budget exhausted'}))
    found = psq.qualification_readiness(cells['b'], [blocked, *trials], NATIVE, RUNTIME)
    assert len(found['trials']) == 3 and found['reliability_findings'] == [dict(
        run=str(blocked.resolve()), finding='PREPLAN_TRANSFER: SEARCH_BUDGET_EXHAUSTED: candidate search budget exhausted',
        planner=None)]


def test_a_qualified_record_is_refused_while_any_requirement_is_unmet_or_any_binding_is_forged(cells, trials, tmp_path):
    readiness = psq.qualification_readiness(cells['b'], trials, NATIVE, RUNTIME)
    path = tmp_path / 'qualified.json'
    verify = lambda sha, cell='b', native=NATIVE: psq.verify_qualified_scope(path, sha, cells[cell], native, RUNTIME)
    sha = qualified_record(path, readiness)
    # Everything bound correctly, yet not qualified: the stop/cancel acceptance does not exist.
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_NOT_QUALIFIED: stop/cancel'):
        verify(sha)
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_RECORD_CHANGED'):
        verify('0' * 64)
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: native'):          # stale
        verify(sha, native=dict(NATIVE, support_sha256='rebuilt'))
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: cell'):            # cross-bin cell
        verify(sha, cell='a')
    for change, match in ((dict(contract=dict(psq.QUALIFICATION_CONTRACT, strict_trials=1)), 'another qualification contract'),
                          (dict(state=psq.AUTHORISED_TRIAL), 'not a qualified scope'),
                          (dict(destination='target_bin_default'), 'PHYSICS_SCOPE_MISMATCH: destination'),
                          (dict(trials=[readiness['trials'][0]] * 3), 'PHYSICS_SCOPE_REPLAYED'),
                          (dict(trials=[dict(readiness['trials'][0], run_id='run-9'), *readiness['trials'][1:]]),
                           'PHYSICS_SCOPE_EVIDENCE_CHANGED')):                                  # cross-session
        with pytest.raises(psq.PhysicsScopeRefused, match=match):
            verify(qualified_record(path, readiness, **change))
    sha = qualified_record(path, readiness)
    summary = Path(readiness['trials'][1]['run']) / '06-full-cycle/full-cycle/summary.json'
    original = summary.read_bytes()
    summary.write_bytes(original.replace(b'"settled": true', b'"settled": true, "edited": 1'))
    try:
        with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_EVIDENCE_CHANGED'):     # modified evidence
            verify(sha)
    finally:
        summary.write_bytes(original)


def test_trial_scope_needs_world_frame_measurements(cells, tmp_path):
    session = launch(tmp_path, cells['b'])
    parts = yaml.safe_load((session / 'parts.yaml').read_text())
    parts['objects'][0]['pose']['frame_id'] = 'camera_link'
    (session / 'parts.yaml').write_text(yaml.safe_dump(parts))
    with pytest.raises(psq.PhysicsScopeRefused, match='world frame'):
        binding(session, cells['b'])


# --- G5B: Stop at Cycle End during the transfer of an authorised trial ---------------------------------------------
def stopped_trial(root, cell, edit=None, session_edit=None, head=HEAD):
    """A strict trial whose runner latched SIGUSR1 while the executor was transferring, as the runner records it."""
    def stop(summary, report):
        summary['stage_events'] = [{'stage': 'EXECUTE_LIFT', 'timestamp': 90.0}, {'stage': 'EXECUTE_TRANSFER', 'timestamp': 100.0},
                                   {'stage': 'EXECUTE_PLACE', 'timestamp': 110.0}, {'stage': 'EXECUTE_HOME', 'timestamp': 130.0}]
        report.update(gates={'full-cycle': {'status': 'PASS'}}, stop_after_cycle=dict(
            signal='SIGUSR1', requested_wall_ns=105_000_000_000, state='STOPPED', gate='full-cycle', phase='execute',
            executor_stage='EXECUTE_TRANSFER', forwarded=None, repeated=0, stopped_wall_ns=140_000_000_000,
            completed_gates=['full-cycle'], gates_not_started=[]))
        if edit:
            edit(summary, report, report['stop_after_cycle'])
    run = strict_trial(root, cell, 'run-stop', edit=stop, head=head)
    if session_edit:
        path = run / '06-full-cycle/session-report.json'
        record = json.loads(path.read_text())
        session_edit(record)
        path.write_text(json.dumps(record))
    return run


def test_a_stop_after_cycle_latched_during_transfer_and_completed_strictly_is_accepted(cells, tmp_path):
    run = stopped_trial(tmp_path, cells['b'])
    accepted = psq.stop_after_cycle_acceptance(run)
    assert accepted['accepted'] is True and accepted['state'] == 'STOPPED_AFTER_CYCLE' and accepted['failed'] == []
    # The contract is unchanged: G5B evidence alone never satisfies the qualification's stop/cancel item.
    readiness = psq.qualification_readiness(cells['b'], [run], NATIVE, RUNTIME)
    assert readiness['status'] == 'NOT_QUALIFIED' and any('stop/cancel acceptance' in u for u in readiness['unmet'])


@pytest.mark.parametrize('edit,session_edit,failed', [
    (lambda s, r, x: r.pop('stop_after_cycle'), None, 'latched'),
    (lambda s, r, x: x.update(signal='SIGTERM'), None, 'latched'),
    (lambda s, r, x: x.update(forwarded='SIGUSR1'), None, 'latched'),
    (lambda s, r, x: x.update(acknowledgement_error='OSError()'), None, 'latched'),
    (lambda s, r, x: x.update(requested_wall_ns=99_000_000_000), None, 'during_transfer'),     # before the transfer
    (lambda s, r, x: x.update(requested_wall_ns=111_000_000_000), None, 'during_transfer'),    # after the place began
    (lambda s, r, x: x.update(executor_stage='EXECUTE_LIFT'), None, 'during_transfer'),
    (lambda s, r, x: x.update(phase='revalidate'), None, 'during_transfer'),
    (lambda s, r, x: s.update(stage_events=[]), None, 'during_transfer'),
    (lambda s, r, x: x.update(superseded_by='SIGTERM'), None, 'not_cancelled'),
    (None, lambda d: d.update(operator_cancel={'signal': 'SIGTERM'}), 'not_cancelled'),
    (lambda s, r, x: s.update(failed_stage='EXECUTE_PLACE', result='FAIL'), None, 'not_cancelled'),
    (lambda s, r, x: s.update(recovery_required=True), None, 'not_cancelled'),
    (lambda s, r, x: s['release_evidence'].update(state='RELEASE_UNCONFIRMED'), None, 'cycle_completed_strictly'),
    (lambda s, r, x: s.update(stages=psq.FULL_CYCLE_STAGES[:-1]), None, 'cycle_completed_strictly'),
    (lambda s, r, x: s['full_cycle_physical_acceptance'].update(attached_ids=['runtime::part_04']), None,
     'cycle_completed_strictly'),
    (None, lambda d: d.update(shutdown={'clean': False}), 'cycle_completed_strictly'),
    (lambda s, r, x: x.update(state='STOPPING'), None, 'no_further_cycle'),
    (lambda s, r, x: x.update(gates_not_started=['full-cycle']), None, 'no_further_cycle'),
    (lambda s, r, x: r['gates'].update({'resolve': {'status': 'PASS'}}), None, 'no_further_cycle'),
])
def test_a_stop_after_cycle_missing_any_item_is_not_accepted(cells, tmp_path, edit, session_edit, failed):
    run = stopped_trial(tmp_path, cells['b'], edit=edit, session_edit=session_edit)
    accepted = psq.stop_after_cycle_acceptance(run)
    assert accepted['accepted'] is False and accepted['state'] == 'NOT_ACCEPTED' and failed in accepted['failed']


# --- Q7: the accepted G5A cancel and G5B stop-at-cycle-end runs, bound by identity --------------------------------
CANCEL = dict(signal='SIGTERM', child=dict(forwarded='SIGTERM', killed=False, grace_s=90, returncode=1))


def cancelled_trial(root, cell, run_id='run-cancel', edit=None, session_edit=None, head=HEAD):
    """A G5A run as the runner records it: SIGTERM forwarded to the executor, its held transfer cancelled."""
    from simulator_execution import transfer_cancellation_acceptance
    from test_simulator_execution import g5a_summary
    def cancelled(summary, report):
        scope = summary['physics_scope']
        summary.clear()
        summary.update(g5a_summary(), physics_scope=scope)
        summary['transfer_cancellation'] = transfer_cancellation_acceptance(summary)
        report.update(status='CANCELLED', gates={'full-cycle': {'status': 'INTERRUPTED', 'operator_cancel': CANCEL}})
        if edit:
            edit(summary, report)
    run = strict_trial(root, cell, run_id, edit=cancelled, head=head)
    path = run / '06-full-cycle/session-report.json'
    record = dict(json.loads(path.read_text()), operator_cancel=copy.deepcopy(CANCEL))
    if session_edit:
        session_edit(record)
    path.write_text(json.dumps(record))
    return run


def rebound(run, edit):
    """The run's scope record re-bound after ``edit`` and every copy of its digest updated (a consistent forgery)."""
    session = run / '06-full-cycle'
    sha = rewrite(session / 'physics-scope.json', edit)
    for path in (session / 'session-report.json', session / 'full-cycle/summary.json'):
        rewrite_json = json.loads(path.read_text())
        rewrite_json['physics_scope']['sha256'] = sha
        path.write_text(json.dumps(rewrite_json))
    return run


@pytest.fixture(scope='module')
def g5(cells, tmp_path_factory):
    root = tmp_path_factory.mktemp('g5')
    return dict(cancel=cancelled_trial(root / 'cancel', cells['b']), stop=stopped_trial(root / 'stop', cells['b']))


def test_q7_recognises_both_accepted_g5_runs_and_never_replaces_the_strict_trials(cells, trials, g5):
    q7 = 'stop/cancel acceptance not established'
    ready = psq.qualification_readiness(cells['b'], [*trials[:2], g5['stop']], NATIVE, RUNTIME,
                                        cancellation_trial=g5['cancel'], stop_trial=g5['stop'])
    assert ready['status'] == 'QUALIFIED_READY' and ready['unmet'] == [] and len(ready['trials']) == 3
    assert {k: v['state'] for k, v in ready['stop_cancel'].items()} == {
        'cancellation': 'RECOVERY_REQUIRED_HELD_STOPPED', 'stop_after_cycle': 'STOPPED_AFTER_CYCLE'}
    assert ready['stop_cancel']['cancellation']['run_id'] == 'run-cancel' and ready['stop_cancel']['stop_after_cycle']['run_id'] == 'run-stop'
    # G5B counts toward Q8 only when listed as a trial; G5A never does (it is not a completed placement).
    unlisted = psq.qualification_readiness(cells['b'], trials[:2], NATIVE, RUNTIME, cancellation_trial=g5['cancel'], stop_trial=g5['stop'])
    assert unlisted['unmet'] == ['2 of 3 independent strict trial PASS sessions']
    listed = psq.qualification_readiness(cells['b'], [*trials[:2], g5['cancel']], NATIVE, RUNTIME,
                                         cancellation_trial=g5['cancel'], stop_trial=g5['stop'])
    assert len(listed['trials']) == 2 and any('not a passing authorised trial report' in u for u in listed['unmet'])
    # Either G5 run alone, or none, leaves Q7 unmet.
    for kwargs in (dict(), dict(cancellation_trial=g5['cancel']), dict(stop_trial=g5['stop'])):
        alone = psq.qualification_readiness(cells['b'], trials, NATIVE, RUNTIME, **kwargs)
        assert alone['status'] == 'NOT_QUALIFIED' and alone['stop_cancel'] == {} and any(q7 in u for u in alone['unmet'])
    # G5 is once per multi-destination world (R2.5B): the same runs serve the other bin's cell, whose Q8 still fails.
    other = psq.qualification_readiness(cells['a'], [], NATIVE, RUNTIME, cancellation_trial=g5['cancel'], stop_trial=g5['stop'])
    assert other['unmet'] == ['0 of 3 independent strict trial PASS sessions'] and len(other['stop_cancel']) == 2


@pytest.mark.parametrize('edit,session_edit,reason', [
    (lambda s, r: r.update(status='PASS'), None, 'not a cancelled authorised trial report'),
    (lambda s, r: r.update(preflight={'repo_dirty': True}), None, 'modified checkout'),
    (lambda s, r: s['physics_scope'].update(sha256='other'), None, 'differs from the one the trial verified'),
    (None, lambda d: d.update(shutdown={'clean': False}), 'owned shutdown not clean'),
    (None, lambda d: d['operator_cancel']['child'].update(killed=True), 'not forwarded'),
    (None, lambda d: d['operator_cancel']['child'].update(forwarded=None), 'not forwarded'),
    (None, lambda d: d.pop('operator_cancel'), 'not forwarded'),
    (lambda s, r: r['gates']['full-cycle'].pop('operator_cancel'), None, 'not forwarded'),
    (lambda s, r: s['measured_reconciliation'].update(held=False), None, 'part_still_held'),
    (lambda s, r: s['controller_cancellation']['ur5_arm_controller'].update(status=4), None, 'controller_goal_canceled'),
    (lambda s, r: s['execution_results'].append({'stage': 'EXECUTE_PLACE'}), None, 'no_motion_after_cancel'),
    (lambda s, r: s.update(result='PASS'), None, 'recovery_required'),
    (lambda s, r: s['transfer_cancellation'].update(state='OTHER'), None, 'recorded verdict differs'),
    (lambda s, r: s.pop('transfer_cancellation'), None, 'recorded verdict differs'),
])
def test_a_g5a_run_missing_any_item_fails_closed(cells, trials, g5, tmp_path, edit, session_edit, reason):
    run = cancelled_trial(tmp_path, cells['b'], edit=edit, session_edit=session_edit)
    found = psq.qualification_readiness(cells['b'], trials, NATIVE, RUNTIME, cancellation_trial=run, stop_trial=g5['stop'])
    assert found['status'] == 'NOT_QUALIFIED' and found['stop_cancel'] == {}
    assert any('stop/cancel' in u and reason in u for u in found['unmet']), found['unmet']


def test_missing_wrong_type_mismatched_or_replayed_g5_evidence_fails_closed(cells, trials, g5, tmp_path):
    def unmet(cancel, stop, native=NATIVE, runtime=RUNTIME, cell='b'):
        found = psq.qualification_readiness(cells[cell], trials, native, runtime, cancellation_trial=cancel, stop_trial=stop)
        assert found['status'] == 'NOT_QUALIFIED' and found['stop_cancel'] == {}
        return ' '.join(u for u in found['unmet'] if 'stop/cancel' in u)
    assert 'evidence missing' in unmet(tmp_path / 'absent', g5['stop'])
    assert 'latched' in unmet(g5['cancel'], g5['cancel'])                          # a cancel is not a stop at cycle end
    assert 'not a cancelled authorised trial report' in unmet(g5['stop'], g5['stop'])   # nor the reverse
    assert 'not a cancelled authorised trial report' in unmet(trials[0], g5['stop'])    # a plain strict trial
    assert 'native identity changed' in unmet(g5['cancel'], g5['stop'], native=dict(NATIVE, support_sha256='rebuilt'))
    assert 'runtime identity changed' in unmet(g5['cancel'], g5['stop'], runtime=dict(RUNTIME, record_sha256='other'))
    world = rebound(cancelled_trial(tmp_path / 'world', cells['b'], run_id='run-w'),
                    lambda r: r['binding']['world'].update(derived_sha256='other'))
    assert 'another world' in unmet(world, g5['stop'])
    robot = rebound(cancelled_trial(tmp_path / 'robot', cells['b'], run_id='run-r'),
                    lambda r: r['binding']['robot'].update(model='another_robot'))
    assert 'geometry or robot differs' in unmet(robot, g5['stop'])
    same = stopped_trial(tmp_path / 'same', cells['b'])                               # both runs from one session
    assert 'same trial session' in unmet(cancelled_trial(tmp_path / 'c2', cells['b'], run_id='run-stop'), same)
    late = stopped_trial(tmp_path / 'late', cells['b'], edit=lambda s, r, x: x.update(executor_stage='EXECUTE_LIFT'))
    assert 'during_transfer' in unmet(g5['cancel'], late)
    truncated = cancelled_trial(tmp_path / 'truncated', cells['b'], run_id='run-t')
    summary = truncated / '06-full-cycle/full-cycle/summary.json'
    summary.write_bytes(summary.read_bytes()[:-40])                                # unreadable: refused, never ready
    with pytest.raises(ValueError):
        psq.qualification_readiness(cells['b'], trials, NATIVE, RUNTIME, cancellation_trial=truncated, stop_trial=g5['stop'])


def test_a_qualified_record_binds_the_g5_runs_and_refuses_any_change_to_them(cells, trials, g5, tmp_path):
    """No issuer exists: this record is written as one would be, to prove the verifier re-judges the G5 evidence."""
    ready = psq.qualification_readiness(cells['b'], [*trials[:2], g5['stop']], NATIVE, RUNTIME,
                                        cancellation_trial=g5['cancel'], stop_trial=g5['stop'])
    path = tmp_path / 'qualified.json'
    verify = lambda sha: psq.verify_qualified_scope(path, sha, cells['b'], NATIVE, RUNTIME)
    assert verify(qualified_record(path, ready))['stop_cancel'] == ready['stop_cancel']
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_NOT_QUALIFIED: stop/cancel'):     # G5 left out
        verify(qualified_record(path, ready, stop_cancel={}))
    other = cancelled_trial(tmp_path / 'other', cells['b'], run_id='run-cancel-2')                  # swapped G5A run
    swapped = dict(ready['stop_cancel'], cancellation=dict(ready['stop_cancel']['cancellation'], run=str(other.resolve())))
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: stop_cancel'):
        verify(qualified_record(path, ready, stop_cancel=swapped))
    sha = qualified_record(path, ready)
    summary = g5['cancel'] / '06-full-cycle/full-cycle/summary.json'
    original = summary.read_bytes()
    summary.write_bytes(original.replace(b'"cancellation_confirmed": true', b'"cancellation_confirmed": true, "edited": 1'))
    try:
        with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: stop_cancel'):    # modified evidence
            verify(sha)
    finally:
        summary.write_bytes(original)


# --- Software identity: repeatability counts one version; nothing is hidden or cherry-picked -----------------------
def commit(rev):
    """A real historical commit of this repository (skipped where the history is not available)."""
    try:
        return psq._git(psq.REPO, 'rev-parse', '--verify', rev + '^{commit}').strip()
    except psq.PhysicsScopeRefused:
        pytest.skip(f'commit {rev} not in this clone')


def test_the_planner_sources_cover_the_executors_whole_import_closure():
    import ast
    scripts, local = ROOT / 'scripts', {p.stem for p in (ROOT / 'scripts').glob('*.py')}
    seen, todo = set(), ['perceived_object_grasp_execute']
    while todo:
        name = todo.pop()
        if name in seen:
            continue
        seen.add(name)
        for node in ast.walk(ast.parse((scripts / f'{name}.py').read_text())):
            names = ([a.name for a in node.names] if isinstance(node, ast.Import) else
                     [node.module] if isinstance(node, ast.ImportFrom) and node.module and not node.level else [])
            todo += [n.split('.')[0] for n in names if n.split('.')[0] in local]
    assert set(psq.EXECUTOR_MODULES) == seen - {'physics_scope_qualification'}
    assert 'scripts/physics_scope_qualification.py' not in psq.PLANNER_SOURCES          # the judge never resets a campaign
    assert {'scenes/ur5_2f_test/launch/demo.launch.py', 'catalog/capabilities'} <= set(psq.PLANNER_SOURCES)


def test_identity_is_reproducible_from_git_and_refuses_a_modified_or_unknown_checkout(tmp_path):
    assert psq.planner_identity(psq.REPO, HEAD) == PLANNER                             # deterministic
    old = commit('cd383090')                                                           # Bin A trial 3's commit
    assert psq.planner_identity(psq.REPO, old) != PLANNER                              # the clearance fix is a new version
    with pytest.raises(psq.PhysicsScopeRefused, match='PROVENANCE_UNRECOVERABLE'):
        psq.planner_identity(psq.REPO, '0' * 40)
    import subprocess
    repo = tmp_path / 'repo'
    (repo / 'scripts').mkdir(parents=True)
    source = repo / 'scripts/perceived_object_grasp_execute.py'
    source.write_text('x = 1\n')
    for args in (['init', '-q'], ['add', '.'], ['-c', 'user.email=t@t', '-c', 'user.name=t', 'commit', '-qm', 'one']):
        subprocess.run(['git', '-C', str(repo), *args], check=True)
    real = psq.planner_identity.__wrapped__ if hasattr(psq.planner_identity, '__wrapped__') else None
    from importlib import reload
    fresh = reload(__import__('physics_scope_qualification'))                          # unpatched module functions
    try:
        clean = fresh.planner_identity(repo)
        source.write_text('x = 2\n')
        with pytest.raises(fresh.PhysicsScopeRefused, match='SOFTWARE_MODIFIED'):     # never fingerprint an edited tree
            fresh.planner_identity(repo)
        assert clean == fresh.planner_identity(repo, 'HEAD')
    finally:
        reload(fresh)


def test_trials_of_another_software_version_are_listed_never_counted_and_never_blocking(cells, trials, g5, tmp_path):
    old = commit('cd383090')
    passed = strict_trial(tmp_path / 'old_pass', cells['b'], 'run-old-pass', head=old)
    failed = strict_trial(tmp_path / 'old_fail', cells['b'], 'run-old-fail', head=old,
                          edit=lambda s, r: s.update(result='FAIL', failed_stage='EXECUTE_HOME'))
    found = psq.qualification_readiness(cells['b'], [passed, failed, *trials], NATIVE, RUNTIME, g5['cancel'], g5['stop'])
    assert found['status'] == 'QUALIFIED_READY' and len(found['trials']) == 3                # only this version counts
    assert [(v['run'], v['kind']) for v in found['other_versions']] == [
        (str(passed.resolve()), 'STRICT_PASS'), (str(failed.resolve()), 'FAILED')]        # both remain visible
    assert all(v['planner'] != PLANNER for v in found['other_versions'])
    same = strict_trial(tmp_path / 'new_fail', cells['b'], 'run-new-fail',
                        edit=lambda s, r: s.update(result='FAIL', failed_stage='EXECUTE_HOME'))
    blocked = psq.qualification_readiness(cells['b'], [same, *trials], NATIVE, RUNTIME, g5['cancel'], g5['stop'])
    assert blocked['status'] == 'NOT_QUALIFIED' and any('not a strict full-cycle PASS' in u for u in blocked['unmet'])


@pytest.mark.parametrize('edit,match', [
    (lambda s, r: r['preflight'].pop('repo_head'), 'recorded no commit'),
    (lambda s, r: r['preflight'].update(repo_head='0' * 40), 'PROVENANCE_UNRECOVERABLE'),
])
def test_unrecoverable_software_provenance_never_counts(cells, trials, g5, tmp_path, edit, match):
    run = strict_trial(tmp_path, cells['b'], 'run-x', edit=edit)
    found = psq.qualification_readiness(cells['b'], [run, *trials[:2]], NATIVE, RUNTIME, g5['cancel'], g5['stop'])
    assert found['status'] == 'NOT_QUALIFIED' and any(match in u for u in found['unmet'])


def test_a_bound_identity_that_differs_from_the_commit_is_refused(cells, trials, g5, tmp_path):
    run = strict_trial(tmp_path, cells['b'], 'run-forged')
    rebound(run, lambda r: r['binding'].update(planner='f' * 64))                           # claims another version
    found = psq.qualification_readiness(cells['b'], [run, *trials[:2]], NATIVE, RUNTIME, g5['cancel'], g5['stop'])
    assert found['status'] == 'NOT_QUALIFIED' and any('software identity differs' in u for u in found['unmet'])


def test_every_unlisted_run_of_this_version_and_cell_is_found(cells, trials, g5, tmp_path):
    root = tmp_path / 'evidence'
    hidden = strict_trial(root / 'hidden', cells['b'], 'run-hidden',
                          edit=lambda s, r: s.update(result='FAIL', failed_stage='EXECUTE_HOME'))
    strict_trial(root / 'old', cells['b'], 'run-old', head=commit('cd383090'))                   # another version
    cancelled_trial(root / 'cancel', cells['b'], run_id='run-cancel-2')                          # operator stop evidence
    strict_trial(root / 'other_cell', cells['a'], 'run-a')                                      # another cell
    found = psq.qualification_readiness(cells['b'], trials, NATIVE, RUNTIME, g5['cancel'], g5['stop'],
                                        evidence_roots=[root])
    assert found['status'] == 'NOT_QUALIFIED' and found['unlisted'] == [str(hidden.resolve())]  # the failure is found
    listed = psq.qualification_readiness(cells['b'], [*trials, hidden], NATIVE, RUNTIME, g5['cancel'], g5['stop'],
                                         evidence_roots=[root])
    assert listed['unlisted'] == [] and any('not a strict full-cycle PASS' in u for u in listed['unmet'])


def test_g5_runs_of_another_version_need_their_stop_cancel_code_unchanged(cells, trials, tmp_path):
    for role, head in (('cancellation', commit('b5092535')), ('stop_after_cycle', commit('b22b8337'))):
        assert psq.mechanism_identity(role, psq.REPO, head) == psq.mechanism_identity(role, psq.REPO, HEAD), role
    cancel = cancelled_trial(tmp_path / 'g5a', cells['b'], run_id='run-g5a', head=commit('b5092535'))
    stop = stopped_trial(tmp_path / 'g5b', cells['b'], head=commit('b22b8337'))
    found = psq.qualification_readiness(cells['b'], trials, NATIVE, RUNTIME, cancel, stop)
    assert found['status'] == 'QUALIFIED_READY'
    assert {k: v['compatibility'] for k, v in found['stop_cancel'].items()} == {
        'cancellation': 'stop/cancel mechanism unchanged', 'stop_after_cycle': 'stop/cancel mechanism unchanged'}
    before = cancelled_trial(tmp_path / 'g5a_old', cells['b'], run_id='run-g5a-old', head=commit('a8b287b2'))
    assert psq.mechanism_identity('cancellation', psq.REPO, commit('a8b287b2')) != psq.mechanism_identity(
        'cancellation', psq.REPO, HEAD)                                                     # before the G5A fixes
    refused = psq.qualification_readiness(cells['b'], trials, NATIVE, RUNTIME, before, stop)
    assert refused['status'] == 'NOT_QUALIFIED' and any('stop/cancel code changed' in u for u in refused['unmet'])


def test_the_executor_recomputes_the_identity_bound_in_its_trial_record(cells, tmp_path):
    session = launch(tmp_path, cells['b'])
    path = tmp_path / 'physics-scope.json'
    sha = psq.authorise_trial(path, binding(session, cells['b']))
    assert psq.verify_trial(path, sha, binding(session, cells['b']))['binding']['planner'] == PLANNER
    with pytest.raises(psq.PhysicsScopeRefused):                                            # the software changed
        psq.verify_trial(path, sha, binding(session, cells['b'], planner='e' * 64))


# --- Issuance (G3C): a QUALIFIED record is written only when ready, and nothing consumes it -----------------------
def test_the_issuer_writes_a_record_only_when_every_requirement_holds(cells, trials, g5, tmp_path):
    issue = lambda path, runs, cancel=g5['cancel'], stop=g5['stop'], cell='b': psq.issue_qualified_scope(
        path, cells[cell], runs, NATIVE, RUNTIME, cancel, stop)
    for path, runs, kwargs, match in ((tmp_path / 'short.json', trials[:2], {}, '2 of 3'),
                                      (tmp_path / 'no_g5.json', trials, dict(cancel=None), 'stop/cancel'),
                                      (tmp_path / 'other_cell.json', trials, dict(cell='a'), 'another cell')):
        with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_NOT_QUALIFIED: .*' + match):
            issue(path, runs, **kwargs)
        assert not path.exists()                                                     # nothing written
    path = tmp_path / 'qualified.json'
    sha = issue(path, trials)
    record = psq.verify_qualified_scope(path, sha, cells['b'], NATIVE, RUNTIME)
    assert record['state'] == psq.QUALIFIED and record['operation'] == psq.TRIAL_OPERATION and record['planner'] == PLANNER
    assert record['destination'] == 'target_bin_b' and len(record['trials']) == 3
    with pytest.raises(FileExistsError):                                             # an issued record is never replaced
        issue(path, trials)
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: cell'):           # only its exact cell
        psq.verify_qualified_scope(path, sha, cells['a'], NATIVE, RUNTIME)
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: native'):         # stale build
        psq.verify_qualified_scope(path, sha, cells['b'], dict(NATIVE, sha256='rebuilt'), RUNTIME)
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_MISMATCH: planner'):        # another version
        psq.verify_qualified_scope(path, sha, cells['b'], NATIVE, RUNTIME, planner='d' * 64)
    hidden = strict_trial(tmp_path / 'roots/hidden', cells['b'], 'run-hidden-2',
                          edit=lambda s, r: s.update(result='FAIL', failed_stage='EXECUTE_HOME'))
    with pytest.raises(psq.PhysicsScopeRefused, match='unlisted trials'):                         # a failure surfaces
        psq.verify_qualified_scope(path, sha, cells['b'], NATIVE, RUNTIME, evidence_roots=[tmp_path / 'roots'])
    widened = json.loads(path.read_text())
    for operation in (dict(psq.TRIAL_OPERATION, cycles=2), dict(psq.TRIAL_OPERATION, class_routing=True),
                      dict(psq.TRIAL_OPERATION, destinations=2)):
        body = {k: v for k, v in dict(widened, operation=operation).items() if k != 'binding_sha256'}
        forged = tmp_path / f"forged_{len(list(tmp_path.glob('forged_*')))}.json"
        data = (json.dumps(dict(body, binding_sha256=hashlib.sha256(psq._canonical(body)).hexdigest())) + '\n').encode()
        forged.write_bytes(data)
        with pytest.raises(psq.PhysicsScopeRefused, match='permits another operation'):     # no wider permission
            psq.verify_qualified_scope(forged, hashlib.sha256(data).hexdigest(), cells['b'], NATIVE, RUNTIME)


def test_issue_and_verify_cli_report_without_admitting(cells, trials, g5, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(psq, 'native_identity', lambda path: NATIVE)
    monkeypatch.setattr(psq, 'runtime_identity', lambda workspace: RUNTIME)
    runs = [a for run in trials for a in ('--trial', str(run))]
    g5_args = ['--cancellation-trial', str(g5['cancel']), '--stop-trial', str(g5['stop']), '--evidence-root', str(tmp_path)]
    record = tmp_path / 'cli.json'
    assert psq.main(['issue', '--cell', str(cells['b']), *runs[:2], *g5_args, '--output', str(record)]) == 2
    assert json.loads(capsys.readouterr().out)['status'] == 'REFUSED' and not record.exists()
    assert psq.main(['issue', '--cell', str(cells['b']), *runs, *g5_args, '--output', str(record)]) == 0
    issued = json.loads(capsys.readouterr().out)
    assert issued['status'] == 'ISSUED' and issued['admission'].startswith('none')
    assert psq.main(['verify', '--cell', str(cells['b']), '--record', str(record), '--sha256', issued['sha256'],
                     '--evidence-root', str(tmp_path)]) == 0
    assert json.loads(capsys.readouterr().out)['status'] == 'VERIFIED'
    assert psq.main(['verify', '--cell', str(cells['b']), '--record', str(record), '--sha256', '0' * 64]) == 2
    assert 'RECORD_CHANGED' in json.loads(capsys.readouterr().out)['reason']


def test_no_runtime_path_consumes_a_qualified_record():
    """Runtime admission is G6: the executor, runner, Builder and HMI never read a QUALIFIED record or its verifier."""
    for name in ('scripts/perceived_object_grasp_execute.py', 'scripts/run_stage_a1_finish.py', 'scripts/simulator_world.py',
                 'scripts/simulator_execution.py', 'scripts/run_r14_plan_only_acceptance.py'):
        source = (ROOT / name).read_text()
        assert 'verify_qualified_scope' not in source and 'issue_qualified_scope' not in source, name
    for source in (ROOT / 'workcell_builder').rglob('*'):
        if source.suffix in ('.py', '.cpp', '.hpp', '.h'):
            text = source.read_text(errors='replace')
            assert 'verify_qualified_scope' not in text and 'issue_qualified_scope' not in text, source


# --- G4: continuous sorting authority: plan-only sessions and the bounded CLI trial -----------------------------------
import os  # noqa: E402
import time  # noqa: E402

Q = [0.025, 0.025, 0.025]
PERCEPTION = dict(producer='simulator_render', source_type='rendered_perception_in_simulation',
                  epd_runtime_sha256='e' * 64, max_age_s=300.0)
RUNNER = ['python3', 'scripts/run_g4_sorting.py', '--authorise-g4-trial', '--max-cycles', '2']


@pytest.fixture(scope='module')
def g4_cell(cells, tmp_path_factory):
    """The two-bin cell with three qualified-body objects and one plain cube, observed through the simulated RGB-D
    camera, with a saved cup -> Bin A routing rule (the Bin A cell's own placement region)."""
    import shutil
    cell = tmp_path_factory.mktemp('g4') / 'g4_two_bin'
    shutil.copytree(cells['a'], cell)
    path = cell / 'environment.yaml'
    doc = yaml.safe_load(path.read_text())
    source = doc['workcell_studio']['pick_zone']['object_source']
    base = source.pop('manual_observation')
    source['manual_observations'] = [dict(copy.deepcopy(base), object_id=oid, class_id='unclassified',
                                          pose=dict(base['pose'], xyz=[x, y, 0.0125]))
                                     for oid, x, y in (('cup_1', 0.36, -0.20), ('cup_2', 0.42, -0.20),
                                                       ('cup_3', 0.48, -0.20), ('cube_1', 0.42, -0.14))]
    path.write_text(yaml.safe_dump(doc, sort_keys=False))
    intent_path = cell / 'config/workcell_builder_task_intent.yaml'
    intent = yaml.safe_load(intent_path.read_text())
    selection = intent['pick']['selection']
    selection['object_filter']['class_id'] = ''
    selection.update(source_type='perception', source_ref='detected_objects/v1')
    selection['object_filter']['max_age_seconds'] = 180.0
    intent['perception'] = dict(intent.get('perception') or {}, mode='live_epd', camera_source='simulated_rgbd',
                                replay_path='')
    intent['routing'] = {'mode': 'class', 'unrouted_policy': 'leave_in_place', 'rules': [
        {'id': 'cup_to_bin_a', 'when': {'attribute': 'class', 'equals': 'cup'}, 'destination': 'default_drop_zone'}]}
    intent_path.write_text(json.dumps(intent, indent=4))
    from test_r4_1_authored_cell_flow import generate_and_validate
    rc, _ = generate_and_validate(cell)                                   # the Builder's Generate + Validate
    assert rc == 0
    return cell


def g4_binding(cell, session, **kw):
    args = dict(cell_dir=cell, receipt_path=session / 'runtime/receipt.json', native=NATIVE, runtime=RUNTIME,
                perception=PERCEPTION, max_cycles=2, planner=PLANNER, control=CONTROL,
                commission_evidence_path=session / 'commission-evidence.json')
    args.update(kw)
    return psq.g4_session_binding(**args)


def fake_processes(monkeypatch, runner_cmdline=RUNNER):
    """Every pid looks like the CLI trial runner; the executor's parent is this (runner) process."""
    monkeypatch.setattr(psq, 'process_identity', lambda pid: dict(pid=int(pid), start_ticks='42', cmdline=list(runner_cmdline)))
    monkeypatch.setattr(os, 'getppid', os.getpid)


@pytest.fixture
def trial(g4_cell, tmp_path, monkeypatch):
    fake_processes(monkeypatch)
    session = launch(tmp_path / 's', g4_cell)
    bound = g4_binding(g4_cell, session)
    path = session / 'g4-session.json'
    sha = psq.g4_authorise_trial(path, bound, psq.process_identity(os.getpid()))
    return dict(cell=g4_cell, session=session, bound=bound, path=path, sha=sha)


def routed(bound, asset='target_bin_default'):
    return next(d for d in bound['destinations'] if d['source_item_id'] == asset)


def cycle_inputs(t, name, target='cup_1', cls='cup', occupants=(), frame_age_s=1.0, grasp='top_2f::000',
                 edit_observations=None, edit_plan=None):
    """The frame, validated observations and plan-only summary a runner cycle produces, consistent with the binding."""
    bound, root = t['bound'], t['session'] / name
    root.mkdir()
    frame = dict(schema_version='detected_objects/v1', objects=[], source=dict(
        producer='simulator_render', world_revision='rev-' + name, capture_wall_ns=time.time_ns() - int(frame_age_s * 1e9)))
    (root / 'frame.yaml').write_text(yaml.safe_dump(frame))
    frame_sha = hashlib.sha256((root / 'frame.yaml').read_bytes()).hexdigest()
    floor = routed(bound)
    objects = []
    for o in bound['world']['objects']:
        xyz = list(o['spawn_pose'][:3])
        if o['object_id'] in occupants:
            xyz = [floor['floor_xyz'][0], floor['floor_xyz'][1], floor['floor_z'] + 0.0125]
        objects.append(dict(object_id=o['object_id'], class_id='cup' if o['object_id'].startswith('cup') else 'unperceived',
                            dimensions=o['dimensions_m'], pose=dict(frame_id='world', xyz=xyz, rpy=[0.0, 0.0, 0.0])))
    observed = dict(schema_version='detected_objects/v1', simulator_receipt_sha256=bound['session']['receipt_sha256'],
                    perception_binding=dict(perception_sha256=frame_sha, bindings={
                        o['object_id']: dict(class_id=o['class_id']) for o in objects}), objects=objects)
    if edit_observations:
        edit_observations(observed)
    (root / 'observations.yaml').write_text(yaml.safe_dump(observed))
    plan = dict(result='PLAN_ONLY', execution_attempted=False, selected_object_id='runtime::' + target,
                batch_target_object_id='runtime::' + target, resolution_sha256='res-' + name,
                physics_pads=dict(allowance_m={'transfer': 0.015}), selected_extraction_intent=dict(candidate_id=grasp),
                physical_destination_walls=dict(selected_destination=dict(model=floor['model'])),
                task_intent_resolution=dict(resolution_sha256='res-' + name, grasp_resolution=dict(selected_candidate_id=grasp),
                                            place_resolution=dict(route=dict(class_id=cls, region_ref='default_drop_zone',
                                                                             asset_ref='target_bin_default'))))
    if edit_plan:
        edit_plan(plan)
    (root / 'plan.json').write_text(json.dumps(plan))
    return dict(frame_path=root / 'frame.yaml', observations_path=root / 'observations.yaml', plan_path=root / 'plan.json')


def authorise(t, cycle, inputs, path=None, target='cup_1', cls='cup', destination='default_drop_zone', occupancy=(),
              previous=None, current=None, **kw):
    return psq.g4_authorise_cycle(path or t['session'] / f'c{cycle}.json', t['path'], t['sha'],
                                  current=t['bound'] if current is None else current, cycle=cycle,
                                  target=dict(object_id=target, class_id=cls), destination=destination,
                                  occupancy=list(occupancy), previous=previous, **inputs, **kw)


def executed_summary(t, record_path, record_sha, *, tilt=0.0, edit=None):
    """An executor summary file as a strict contact-backed G4 cycle writes it, for this cycle record."""
    import math
    record = json.loads(Path(record_path).read_text())
    floor = routed(t['bound'])
    q = [math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)]
    pose = [floor['floor_xyz'][0], floor['floor_xyz'][1], floor['floor_z'] + 0.0125] + q
    target = 'runtime::' + record['target']['object_id']
    summary = dict(result='PASS', strict_retention_compliant=True, retention_policy='strict',
                   retention_findings={'findings': []}, stages=psq.FULL_CYCLE_STAGES, selected_object_id=target,
                   resolution_sha256=record['plan']['resolution_sha256'], execution_attempted=True,
                   release_evidence={'state': 'RELEASE_CONFIRMED', 'settled': True},
                   selected_extraction_intent={'candidate_id': record['grasp']},
                   physics_scope=dict(kind='G4_TRIAL', cycle_sha256=record_sha, session_sha256=record['session_sha256']),
                   commissioning_prerequisites=dict(results=psq.PREREQUISITE_RESULTS, capability_sha256=NATIVE['sha256'],
                                                    sources=NATIVE['sources']),
                   full_cycle_physical_acceptance=dict(attached_ids=[], baseline_acm_restored=True, final_collision_valid=True,
                                                       final_pose=pose, placement={'error_to_planned_m': 0.0001}),
                   measured_reconciliation=dict(held=False, target=target, acm_restored=True, measured_geometry_matches=True))
    if edit:
        edit(summary)
    path = Path(record_path).with_suffix('.summary.json')
    path.write_text(json.dumps(summary))
    return path


def test_a_plan_only_g4_session_never_authorises_and_g3c_never_admits_its_cell(g4_cell, cells, tmp_path):
    session = launch(tmp_path / 's', g4_cell)
    bound = g4_binding(g4_cell, session, max_cycles=4)
    assert sorted(o['object_id'] for o in bound['world']['objects']) == ['cube_1', 'cup_1', 'cup_2', 'cup_3']
    path = tmp_path / 'session.json'
    sha = psq.g4_open_session(path, bound)
    assert json.loads(path.read_text())['state'] == psq.G4_PLAN_ONLY
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_SCOPE_NOT_QUALIFIED'):            # no acceptance exists
        psq.g4_open_session(tmp_path / 'authorised.json', bound, acceptance={'claimed': True})
    t = dict(session=session, bound=bound, path=path, sha=sha)
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_SCOPE_PLAN_ONLY'):                 # so nothing may move
        authorise(t, 1, cycle_inputs(t, 'p'))
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_SCOPE_PLAN_ONLY'):
        psq.g4_admit_execution(session_path=path, session_sha256=sha, cycle_path=path, cycle_sha256=sha,
                               plan_path=path, observations_path=path, target_object_id='runtime::cup_1', current=bound)
    with pytest.raises(psq.PhysicsScopeRefused, match='one object into one destination, unrouted'):   # G3C stays narrow
        binding(session, g4_cell)


@pytest.mark.parametrize('kw,match', [
    (dict(max_cycles=0), 'max_cycles'), (dict(max_cycles=5), 'max_cycles'), (dict(max_cycles=True), 'max_cycles'),
    (dict(perception=dict(PERCEPTION, producer='replay')), 'simulator-rendered'),
    (dict(perception=dict(PERCEPTION, source_type='replayed_perception_in_simulation')), 'simulator-rendered'),
    (dict(perception=dict(PERCEPTION, max_age_s=1e9)), 'simulator-rendered'),               # no widened frame age
    (dict(perception=dict(PERCEPTION, epd_runtime_sha256='')), 'simulator-rendered'),
    (dict(perception={k: v for k, v in PERCEPTION.items() if k != 'max_age_s'}), 'simulator-rendered'),
])
def test_a_g4_binding_refuses_unsupported_scope(g4_cell, tmp_path, kw, match):
    with pytest.raises(psq.PhysicsScopeRefused, match=match):
        g4_binding(g4_cell, launch(tmp_path, g4_cell), **kw)


def test_a_g4_binding_needs_its_routed_multi_object_world_exactly_as_launched(g4_cell, cells, tmp_path):
    with pytest.raises(psq.PhysicsScopeRefused, match='no class routing'):                  # the G3C cell is unrouted
        g4_binding(cells['b'], launch(tmp_path / 'b', cells['b']))
    session = launch(tmp_path / 'g4', g4_cell)
    receipt = json.loads((session / 'runtime/receipt.json').read_text())
    receipt['destination_geometry_bindings'] = receipt['destination_geometry_bindings'][:1]
    (session / 'runtime/receipt.json').write_text(json.dumps(receipt))
    with pytest.raises(psq.PhysicsScopeRefused, match='multi-destination'):
        g4_binding(g4_cell, session)
    def move_cup(text):
        import xml.etree.ElementTree as ET
        root = ET.fromstring(text)
        pose = next(m for m in root.find('world').findall('model') if m.get('name') == 'cup_2').find('pose')
        values = pose.text.split()
        pose.text = ' '.join([repr(float(values[0]) + 0.01)] + values[1:])
        return ET.tostring(root, encoding='unicode')
    moved = launch(tmp_path / 'w', g4_cell, edit_world=move_cup)               # the launched part is not the cell's
    with pytest.raises(psq.PhysicsScopeRefused, match='PHYSICS_SCOPE_WORLD_MISMATCH'):        # geometry identity
        g4_binding(g4_cell, moved)


def test_a_trial_is_bounded_cli_only_and_bound_to_its_issuing_runner(g4_cell, tmp_path, monkeypatch):
    session = launch(tmp_path / 's', g4_cell)
    bound = g4_binding(g4_cell, session)
    me = psq.process_identity(os.getpid())                     # pytest: not run_g4_sorting.py --authorise-g4-trial
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_SCOPE_ISSUER_MISMATCH'):
        psq.g4_authorise_trial(tmp_path / 'real.json', bound, me)
    fake_processes(monkeypatch)
    issuer = psq.process_identity(os.getpid())
    for edit, match in ((dict(max_cycles=3), 'G4_TRIAL_LIMIT'), (dict(max_cycles=True), 'G4_TRIAL_LIMIT'),
                        (dict(evidence=None), 'G4_SCOPE_INCOMPLETE')):
        with pytest.raises(psq.PhysicsScopeRefused, match=match):
            psq.g4_authorise_trial(tmp_path / 'x.json', dict(bound, **edit), issuer)
    with pytest.raises(psq.PhysicsScopeRefused, match='only the issuing runner itself'):  # a live runner, not the caller
        psq.g4_authorise_trial(tmp_path / 'y.json', bound, psq.process_identity(os.getpid() + 1))
    path = tmp_path / 'trial.json'
    sha = psq.g4_authorise_trial(path, bound, issuer)
    record = json.loads(path.read_text())
    assert record['state'] == psq.G4_AUTHORISED_TRIAL and record['operation'] == psq.G4_TRIAL_OPERATION
    assert record['operation']['qualified'] is False and record['evidence_scope'] == 'G4_TRIAL_NOT_QUALIFIED'
    with pytest.raises(FileExistsError):                                                   # never replaced
        psq.g4_authorise_trial(path, bound, issuer)
    assert psq._g4_session(path, sha, current=bound, role='runner')['state'] == psq.G4_AUTHORISED_TRIAL
    for edit, match in ((lambda r: r.update(state=psq.G4_AUTHORISED_SESSION, operation=psq.G4_OPERATION),
                         'G4_SCOPE_NOT_QUALIFIED'),                                          # a claimed production session
                        (lambda r: r['operation'].update(max_cycles=9), 'G4_SCOPE_UNSUPPORTED'),   # widened operation
                        (lambda r: r['binding'].update(max_cycles=3), 'G4_SCOPE_UNSUPPORTED'),     # widened bound
                        (lambda r: r.update(evidence_scope='QUALIFIED'), 'G4_SCOPE_UNSUPPORTED'),
                        (lambda r: r['issuer'].update(pid=r['issuer']['pid'] + 1), 'G4_SCOPE_ISSUER_MISMATCH')):
        forged = tmp_path / 'forged.json'
        forged.write_bytes(path.read_bytes())
        forged_sha = rewrite(forged, edit)
        with pytest.raises(psq.PhysicsScopeRefused, match=match):
            psq._g4_session(forged, forged_sha, current=json.loads(forged.read_text())['binding'], role='runner')
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_SCOPE_UNVERIFIED'):              # never trusted unrecomputed
        psq._g4_session(path, sha, role='runner')
    monkeypatch.setattr(psq, 'process_identity', lambda pid: dict(pid=int(pid), start_ticks='42', cmdline=['bash']))
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_SCOPE_ISSUER_MISMATCH'):          # another live process
        psq._g4_session(path, sha, current=bound, role='executor')


@pytest.mark.parametrize('edit,key', [
    (dict(native=dict(NATIVE, sha256='rebuilt')), 'native'), (dict(runtime=dict(RUNTIME, record_sha256='other')), 'runtime'),
    (dict(planner='0' * 64), 'planner'), (dict(perception=dict(PERCEPTION, epd_runtime_sha256='f' * 64)), 'perception'),
    (dict(control='0' * 64), 'control'),
])
def test_every_trial_use_recomputes_software_native_and_perception_identity(trial, edit, key):
    current = g4_binding(trial['cell'], trial['session'], **edit)
    with pytest.raises(psq.PhysicsScopeRefused, match=f'G4_SCOPE_MISMATCH: {key}'):
        authorise(trial, 1, cycle_inputs(trial, 'i'), current=current)


def test_a_changed_cell_or_receipt_refuses_the_trial(trial):
    receipt = trial['session'] / 'runtime/receipt.json'
    original = receipt.read_bytes()
    data = json.loads(original)
    data['run_id'] = 'run-2'                                                          # another launch
    receipt.write_text(json.dumps(data))
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_SCOPE_MISMATCH: session'):
        authorise(trial, 1, cycle_inputs(trial, 'r'), current=g4_binding(trial['cell'], trial['session']))
    receipt.write_bytes(original)
    evidence = trial['session'] / 'commission-evidence.json'
    evidence.write_text('[{"result": "OTHER"}]')                                       # other prerequisite trials
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_SCOPE_MISMATCH: evidence'):
        authorise(trial, 1, cycle_inputs(trial, 'e'), current=g4_binding(trial['cell'], trial['session']))


def test_each_trial_cycle_is_authorised_once_from_its_own_verified_inputs(trial):
    t = trial
    for kw, match in (
            (dict(stop_latched=True), 'G4_SESSION_STOPPED'),
            (dict(cycle=3), 'G4_SESSION_LIMIT'), (dict(cycle=0), 'G4_SESSION_LIMIT'),
            (dict(cycle=2), 'G4_CYCLE_CHAIN_BROKEN'),                                    # cycle 2 without cycle 1
            (dict(inputs=dict(frame_age_s=400.0)), 'G4_OBSERVATION_STALE'),
            (dict(inputs=dict(frame_age_s=-30.0)), 'G4_OBSERVATION_STALE'),            # a frame from the future
            (dict(inputs=dict(edit_observations=lambda o: o.update(simulator_receipt_sha256='0' * 64))),
             'G4_OBSERVATION_FOREIGN'),
            (dict(inputs=dict(edit_observations=lambda o: o['perception_binding'].update(perception_sha256='0' * 64))),
             'G4_OBSERVATION_FOREIGN'),                                                 # classes from another frame
            (dict(inputs=dict(edit_observations=lambda o: o['perception_binding']['bindings']['cup_1'].update(
                class_id='bottle'))), 'G4_TARGET_CLASS_MISMATCH'),
            (dict(target='cube_1', cls='unperceived'), 'G4_TARGET_NOT_ROUTED'),         # never a target
            (dict(destination='place_zone_b'), 'G4_TARGET_NOT_ROUTED'),
            (dict(target='ghost'), 'G4_TARGET_UNKNOWN'),
            (dict(inputs=dict(edit_plan=lambda p: p.update(selected_object_id='runtime::cup_2'))), 'G4_PLAN_MISMATCH'),
            (dict(inputs=dict(edit_plan=lambda p: p.update(execution_attempted=True))), 'G4_PLAN_MISMATCH'),
            (dict(inputs=dict(edit_plan=lambda p: p.update(physics_pads=None))), 'G4_PLAN_MISMATCH'),
            (dict(inputs=dict(edit_plan=lambda p: p['physical_destination_walls']['selected_destination'].update(
                model='target_bin_physics_other'))), 'G4_PLAN_MISMATCH'),                # seated on another bin
            (dict(inputs=dict(edit_plan=lambda p: p.update(resolution_sha256='other'))), 'G4_PLAN_MISMATCH'),
            (dict(inputs=dict(grasp='finger_pinch_basic::000')), 'G4_GRASP_UNQUALIFIED'),   # no contact evidence
            (dict(inputs=dict(occupants=('cup_3',))), 'G4_OCCUPANCY_MISMATCH'),          # a part the ledger never placed
            (dict(occupancy=('cup_3',)), 'G4_OCCUPANCY_MISMATCH')):                      # a part that is not there
        kw = dict(kw)
        inputs = cycle_inputs(t, f'n{len(list(t["session"].iterdir()))}', **kw.pop('inputs', {}))
        with pytest.raises(psq.PhysicsScopeRefused, match=match):
            authorise(t, kw.pop('cycle', 1), inputs, **kw)
    assert not list(t['session'].glob('c[0-9]*.json'))                                   # nothing written by a refusal
    sha = authorise(t, 1, cycle_inputs(t, 'ok'))                                        # refusals never spent the number
    record = json.loads((t['session'] / 'c1.json').read_text())
    assert record['state'] == psq.G4_TRIAL_CYCLE and record['grasp'] == 'top_2f::000' and record['previous'] is None
    assert record['destination']['model'] == routed(t['bound'])['model'] and record['plan']['resolution_sha256'] == 'res-ok'
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_CYCLE_ALREADY_ISSUED'):          # never re-issued
        authorise(t, 1, cycle_inputs(t, 'again'), path=t['session'] / 'c1b.json')
    assert sha


def test_the_executor_admits_only_the_exact_authorised_cycle_and_only_once(trial, monkeypatch):
    t = trial
    inputs = cycle_inputs(t, 'x')
    sha = authorise(t, 1, inputs)
    record = t['session'] / 'c1.json'
    admit = lambda **kw: psq.g4_admit_execution(**{**dict(
        session_path=t['path'], session_sha256=t['sha'], cycle_path=record, cycle_sha256=sha,
        plan_path=inputs['plan_path'], observations_path=inputs['observations_path'], target_object_id='runtime::cup_1',
        current=t['bound']), **kw})
    admitted = admit()
    assert admitted['destination']['model'] == routed(t['bound'])['model'] and admitted['resolution']['resolution_sha256'] == 'res-x'
    other = cycle_inputs(t, 'y')
    for kw, match in ((dict(target_object_id='runtime::cup_2'), 'G4_TARGET_MISMATCH'),
                      (dict(observations_path=other['observations_path']), 'G4_OBSERVATION_CHANGED'),
                      (dict(plan_path=other['plan_path']), 'G4_PLAN_CHANGED'),
                      (dict(cycle_sha256='0' * 64), 'G4_CYCLE_RECORD_CHANGED'),
                      (dict(session_sha256='0' * 64), 'G4_SCOPE_RECORD_CHANGED'),
                      (dict(current=g4_binding(t['cell'], t['session'], planner='1' * 64)), 'G4_SCOPE_MISMATCH: planner')):
        with pytest.raises(psq.PhysicsScopeRefused, match=match):
            admit(**kw)
    copied = t['session'] / 'c2-copy.json'                                             # a record for an unissued number
    data = json.loads(record.read_text())
    data['cycle'] = 2
    copied.write_text(json.dumps(data))
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_CYCLE_UNSUPPORTED'):
        admit(cycle_path=copied, cycle_sha256=hashlib.sha256(copied.read_bytes()).hexdigest())
    registry = psq._g4_cycle_dir(t['path'], t['sha'])
    for edit, match in ((lambda r: r.update(grasp='finger_pinch_basic::000'), 'G4_CYCLE_UNSUPPORTED'),   # not the issued
                        (lambda r: r['destination'].update(region='place_zone_b'), 'G4_CYCLE_UNSUPPORTED')):  # digest
        forged = t['session'] / 'c1-forged.json'
        data = json.loads(record.read_text())
        edit(data)
        forged.write_text(json.dumps(data))
        with pytest.raises(psq.PhysicsScopeRefused, match=match):
            admit(cycle_path=forged, cycle_sha256=hashlib.sha256(forged.read_bytes()).hexdigest())
    # Even with the marker rewritten to name a forged record, its content is re-checked (defence in depth).
    original_marker = (registry / '1.issued').read_text()
    for edit, match in ((lambda r: r.update(grasp='finger_pinch_basic::000'), 'G4_GRASP_UNQUALIFIED'),
                        (lambda r: r['destination'].update(region='place_zone_b'), 'G4_TARGET_NOT_ROUTED'),
                        (lambda r: r.update(previous={'cycle_sha256': 'x'}), 'G4_CYCLE_CHAIN_BROKEN')):
        forged = t['session'] / 'c1-forged.json'
        data = json.loads(record.read_text())
        edit(data)
        forged.write_text(json.dumps(data))
        forged_sha = hashlib.sha256(forged.read_bytes()).hexdigest()
        (registry / '1.issued').write_text(forged_sha + '\n')
        with pytest.raises(psq.PhysicsScopeRefused, match=match):
            admit(cycle_path=forged, cycle_sha256=forged_sha)
    (registry / '1.issued').write_text(original_marker)
    frame = Path(json.loads(record.read_text())['frame']['path'])
    original = frame.read_bytes()
    frame.write_bytes(original + b'# edited\n')
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_OBSERVATION_STALE'):
        admit()
    frame.write_bytes(original)
    assert psq.g4_frame_age_problem(admitted['record'], t['bound']) is None
    monkeypatch.setattr(os, 'getppid', lambda: 1)                                       # not started by the issuer
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_SCOPE_ISSUER_MISMATCH'):
        admit()
    monkeypatch.setattr(os, 'getppid', os.getpid)
    psq.g4_consume_cycle(t['path'], t['sha'], 1)
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_CYCLE_REPLAYED'):             # a cycle moves once
        psq.g4_consume_cycle(t['path'], t['sha'], 1)
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_CYCLE_REPLAYED'):
        admit()


def test_routes_select_their_own_bin_and_every_rule_into_a_region_counts(trial):
    bound = trial['bound']
    assert psq.g4_routed_destination(bound, 'default_drop_zone', 'target_bin_default')['source_item_id'] == 'target_bin_default'
    assert psq.g4_routed_destination(bound, 'place_zone_b', 'target_bin_b')['source_item_id'] == 'target_bin_b'
    assert psq.g4_routed_destination(bound, 'default_drop_zone', 'target_bin_default')['model'] != \
        psq.g4_routed_destination(bound, 'place_zone_b', 'target_bin_b')['model']
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_DESTINATION_UNBOUND'):
        psq.g4_routed_destination(bound, 'nowhere', 'no_bin')
    two = dict(bound, routing=dict(bound['routing'], rules=bound['routing']['rules'] + [
        {'id': 'cans', 'when': {'attribute': 'class', 'equals': 'can'}, 'destination': 'default_drop_zone'}]))
    psq._g4_route(two, 'cup', 'default_drop_zone')                                       # both classes into one region
    psq._g4_route(two, 'can', 'default_drop_zone')


def test_a_trial_cycle_counts_only_from_verified_evidence_of_its_own_record(trial):
    t = trial
    sha = authorise(t, 1, cycle_inputs(t, 'v'))
    record = t['session'] / 'c1.json'
    good = psq.g4_verify_cycle(record, sha, executed_summary(t, record, sha), t['bound'])
    assert good['verified'] and good['cycle'] == 1 and good['evidence_sha256']
    import math
    for kw, match in ((dict(edit=lambda s: s['physics_scope'].update(cycle_sha256='0' * 64)), 'not the execution of this'),
                      (dict(edit=lambda s: s.update(resolution_sha256='other')), 'not the authorised plan'),
                      (dict(edit=lambda s: s.update(selected_object_id='runtime::cup_2')), 'not the authorised target'),
                      (dict(edit=lambda s: s['selected_extraction_intent'].update(candidate_id='top_2f::001')),
                       'not the authorised grasp'),
                      (dict(edit=lambda s: s.update(result='FAIL')), 'not a strict full-cycle PASS'),
                      (dict(edit=lambda s: s['measured_reconciliation'].update(held=True)), 'detached and reconciled'),
                      (dict(edit=lambda s: s['full_cycle_physical_acceptance']['placement'].update(error_to_planned_m=None)),
                       'placement error'),
                      (dict(tilt=math.radians(6)), 'not flat')):                         # leaning on a neighbour
        bad = psq.g4_verify_cycle(record, sha, executed_summary(t, record, sha, **kw), t['bound'])
        assert not bad['verified'] and any(match in p for p in bad['problems']), (match, bad['problems'])
    assert psq.g4_verify_cycle(record, '0' * 64, executed_summary(t, record, sha), t['bound'])['problems'] == \
        ['G4_CYCLE_RECORD_CHANGED']
    assert not psq.g4_verify_cycle(record, sha, t['session'] / 'missing.json', t['bound'])['verified']


def test_a_failed_cycle_blocks_every_later_cycle(trial):
    t = trial
    sha = authorise(t, 1, cycle_inputs(t, 'f'))
    record = t['session'] / 'c1.json'
    failed = executed_summary(t, record, sha, edit=lambda s: s.update(result='FAIL'))
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_CYCLE_CHAIN_BROKEN'):
        authorise(t, 2, cycle_inputs(t, 'f2', target='cup_2', occupants=('cup_1',)), occupancy=['cup_1'], target='cup_2',
                  previous=dict(path=record, sha256=sha, summary_path=failed))
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_CYCLE_ALREADY_ISSUED'):         # and cycle 1 never again
        authorise(t, 1, cycle_inputs(t, 'f1'), path=t['session'] / 'c1-again.json')
    passed = executed_summary(t, record, sha)                                          # only a verified cycle 1 chains
    inputs2 = cycle_inputs(t, 'p2', target='cup_2', occupants=('cup_1',))
    sha2 = authorise(t, 2, inputs2, occupancy=['cup_1'], target='cup_2',
                     previous=dict(path=record, sha256=sha, summary_path=passed))
    assert json.loads((t['session'] / 'c2.json').read_text())['previous']['cycle_sha256'] == sha
    with pytest.raises(psq.PhysicsScopeRefused, match='G4_CYCLE_CHAIN_BROKEN'):       # cycle 1 never ran: 2 cannot
        psq.g4_admit_execution(session_path=t['path'], session_sha256=t['sha'], cycle_path=t['session'] / 'c2.json',
                               cycle_sha256=sha2, plan_path=inputs2['plan_path'],
                               observations_path=inputs2['observations_path'], target_object_id='runtime::cup_2',
                               current=t['bound'])


def trial_run(t, root):
    """A complete two-cycle trial run directory in the runner's layout, every file written by the real authority."""
    session = root / 'session'
    (session / 'cycles').mkdir(parents=True)
    (session / 'g4-session.json').write_bytes(t['path'].read_bytes())
    t = dict(t, path=session / 'g4-session.json')
    cycles, previous = [], None
    for n, target, occupants in ((1, 'cup_1', ()), (2, 'cup_2', ('cup_1',))):
        directory = session / 'cycles' / f'{n:03d}'
        (directory / 'execute').mkdir(parents=True)
        sha = authorise(t, n, cycle_inputs(dict(t, session=directory), 'in', target=target, occupants=occupants),
                        path=directory / 'g4-cycle.json', target=target, occupancy=list(occupants), previous=previous)
        summary = executed_summary(t, directory / 'g4-cycle.json', sha)
        summary.rename(directory / 'execute/summary.json')
        previous = dict(path=directory / 'g4-cycle.json', sha256=sha, summary_path=directory / 'execute/summary.json')
        cycles.append(dict(cycle=n, outcome='COMPLETED', steps=dict(
            authorise=dict(granted=True, sha256=sha), execute=dict(result='PASS'), verify=dict(verified=True),
            reconcile=dict(others={'max_position_delta_m': 0.0}))))
    (root / 'g4-sorting-report.json').write_text(json.dumps(dict(
        schema='workcell_g4_sorting_run/v1', scope='G4_TRIAL_NOT_QUALIFIED', status='PASS', result='MAX_CYCLES',
        preflight=dict(repo_head=HEAD, repo_dirty=False), shutdown=dict(clean=True), cycles=cycles,
        g4_session=dict(path=str(t['path']), sha256=t['sha']))))
    return root


def test_g4_qualification_counts_only_strict_trial_evidence_and_issues_nothing(trial, tmp_path):
    run = trial_run(trial, tmp_path / 'runs/trial_1')
    ev = psq.g4_trial_evidence(run)
    assert ev['kind'] == 'STRICT_PASS' and [c['verified'] for c in ev['cycles']] == [True, True], ev['problems']
    ready = psq.g4_qualification_readiness(trial['cell'], [run], NATIVE, RUNTIME, PLANNER, evidence_roots=[tmp_path / 'runs'])
    assert ready['status'] == 'NOT_QUALIFIED' and ready['counted'] == [str(run.resolve())]
    assert any('1 of 3 accepted trial sessions' in u for u in ready['unmet']) and any('stop/cancel' in u for u in ready['unmet'])
    twice = psq.g4_qualification_readiness(trial['cell'], [run, run], NATIVE, RUNTIME, PLANNER, evidence_roots=[])
    assert any('counted twice' in u for u in twice['unmet'])
    assert any('native identity changed' in u for u in psq.g4_qualification_readiness(
        trial['cell'], [run], dict(NATIVE, sha256='other'), RUNTIME, PLANNER, evidence_roots=[])['unmet'])
    other = psq.g4_qualification_readiness(trial['cell'], [run], NATIVE, RUNTIME, '0' * 64, evidence_roots=[])
    assert other['other_versions'] and not other['counted']                              # listed, never counted
    other = psq.g4_qualification_readiness(trial['cell'], [run], NATIVE, RUNTIME, PLANNER, control='0' * 64,
                                           evidence_roots=[])
    assert other['other_versions'] and not other['counted']                              # other G4 control code
    unlisted = psq.g4_qualification_readiness(trial['cell'], [], NATIVE, RUNTIME, PLANNER, evidence_roots=[tmp_path / 'runs'])
    assert unlisted['unlisted'] == [str(run.resolve())]                                  # no cherry-picking
    summary = run / 'session/cycles/002/execute/summary.json'
    original = summary.read_text()
    data = json.loads(original)
    data['full_cycle_physical_acceptance']['placement']['error_to_planned_m'] = 0.004     # a forged-up second cycle
    summary.write_text(json.dumps(data))
    assert psq.g4_trial_evidence(run)['kind'] == 'FAILED'
    data = json.loads(original)
    data['commissioning_prerequisites']['capability_sha256'] = 'another build'
    summary.write_text(json.dumps(data))
    assert any('Stage-A prerequisite' in p for p in psq.g4_trial_evidence(run)['problems'])
    summary.write_text(original)
    report_path = run / 'g4-sorting-report.json'
    report = json.loads(report_path.read_text())
    report_path.write_text(json.dumps(dict(report, status='STOPPED', result='STOPPED_BY_OPERATOR',
                                           cycles=report['cycles'][:1])))
    stopped = psq.g4_trial_evidence(run)                                                  # every moved cycle verified
    assert stopped['kind'] == 'INCOMPLETE'
    lenient = psq.g4_qualification_readiness(trial['cell'], [run], NATIVE, RUNTIME, PLANNER, evidence_roots=[])
    assert lenient['incomplete'] and not any(str(run.resolve()) in u for u in lenient['unmet'])   # listed, not blocking
    report_path.write_text(json.dumps(dict(report, status='CANCELLED')))
    assert psq.g4_trial_evidence(run)['kind'] == 'CANCELLED'
    # A FAULT after motion blocks: the runner's reconcile found a neighbour moved after a verified executor PASS.
    faulted = copy.deepcopy(report)
    faulted['cycles'][1].update(outcome='G4_WORLD_CHANGED')
    del faulted['cycles'][1]['steps']['reconcile']
    report_path.write_text(json.dumps(dict(faulted, status='FAULT', result=None, stop={'code': 'G4_WORLD_CHANGED'})))
    judged = psq.g4_trial_evidence(run)
    assert judged['kind'] == 'FAILED' and any('did not complete this cycle' in p for p in judged['problems'])
    assert any(str(run.resolve()) in u for u in psq.g4_qualification_readiness(
        trial['cell'], [run], NATIVE, RUNTIME, PLANNER, evidence_roots=[])['unmet'])     # it blocks readiness
    # A cancel never hides an earlier failed cycle.
    report_path.write_text(json.dumps(dict(faulted, status='CANCELLED')))
    assert psq.g4_trial_evidence(run)['kind'] == 'FAILED'
    # Authorised but stopped before the executor ran: not a moved cycle.
    unstarted = copy.deepcopy(report)
    unstarted['cycles'][1] = dict(cycle=2, outcome='STOPPED_BEFORE_EXECUTION',
                                  steps=dict(authorise=report['cycles'][1]['steps']['authorise']))
    moved_summary = run / 'session/cycles/002/execute/summary.json'
    moved_summary.rename(moved_summary.with_suffix('.hidden'))
    report_path.write_text(json.dumps(dict(unstarted, status='STOPPED', result='STOPPED_BY_OPERATOR')))
    stopped_early = psq.g4_trial_evidence(run)
    assert stopped_early['kind'] == 'INCOMPLETE' and stopped_early['cycles'][1]['moved'] is False
    moved_summary.with_suffix('.hidden').rename(moved_summary)
    for scope in ('G4_PLAN_ONLY', None):                       # a trial's report relabelled or damaged: never NOT_A_TRIAL
        report_path.write_text(json.dumps(dict(report, scope=scope)))
        assert psq.g4_trial_evidence(run)['kind'] == 'FAILED'
    record_path = run / 'session/g4-session.json'
    record = json.loads(record_path.read_text())
    record['binding']['control'] = '0' * 64                    # bound control code is not the code the run's head holds
    record['binding_sha256'] = psq._sha256(psq._canonical(record['binding']))
    original_record = record_path.read_bytes()
    record_path.write_text(json.dumps(record))
    report_path.write_text(json.dumps(dict(report, g4_session=dict(report['g4_session'],
                                                                   sha256=psq._sha256(record_path.read_bytes())))))
    assert any('G4 control identity differs' in p for p in psq.g4_trial_evidence(run)['problems'])
    record_path.write_bytes(original_record)
    report_path.write_text(json.dumps(dict(report, cycles='not a list')))              # malformed: FAILED, no crash
    assert psq.g4_trial_evidence(run)['kind'] == 'FAILED'
    report_path.write_text(json.dumps(dict(report, cycles=[dict(cycle=None, steps=dict(authorise=dict(granted=True)))])))
    assert psq.g4_trial_evidence(run)['kind'] == 'FAILED'
    report_path.write_text(json.dumps(report))
    other_cell = tmp_path / 'runs/other_cell'                                             # same world, another task
    import shutil
    shutil.copytree(run, other_cell)
    record = json.loads((other_cell / 'session/g4-session.json').read_text())
    record['binding']['cell']['files']['config/workcell_builder_task_intent.yaml'] = '0' * 64
    (other_cell / 'session/g4-session.json').write_text(json.dumps(record))
    scan = psq.g4_qualification_readiness(trial['cell'], [run], NATIVE, RUNTIME, PLANNER, evidence_roots=[tmp_path / 'runs'])
    assert str(other_cell.resolve()) not in str(scan['unlisted'])
    shutil.rmtree(other_cell)
    # A self-consistent cycle pair of another session (record, its digest in the report, the summary naming both)
    # is caught only by the session binding.
    cycle_two, summary_two = run / 'session/cycles/002/g4-cycle.json', run / 'session/cycles/002/execute/summary.json'
    foreign = json.loads(cycle_two.read_text())
    foreign['session_sha256'] = '0' * 64
    cycle_two.write_text(json.dumps(foreign))
    foreign_sha = hashlib.sha256(cycle_two.read_bytes()).hexdigest()
    executed = json.loads(summary_two.read_text())
    executed['physics_scope'].update(cycle_sha256=foreign_sha, session_sha256='0' * 64)
    summary_two.write_text(json.dumps(executed))
    report['cycles'][1]['steps']['authorise']['sha256'] = foreign_sha
    report_path.write_text(json.dumps(report))
    judged = psq.g4_trial_evidence(run)
    assert judged['kind'] == 'FAILED' and any('another session' in p for p in judged['problems'])
    (run / 'session/g4-session.json').write_text('{ truncated')                           # unreadable evidence blocks
    broken = psq.g4_qualification_readiness(trial['cell'], [], NATIVE, RUNTIME, PLANNER, evidence_roots=[tmp_path / 'runs'])
    assert any('unreadable trial evidence' in u for u in broken['unlisted'])
