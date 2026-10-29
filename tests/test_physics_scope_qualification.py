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


def binding(session, cell, native=NATIVE, runtime=RUNTIME, resolution='resolution'):
    return psq.scope_binding(cell_dir=cell, receipt_path=session / 'runtime/receipt.json',
                             observations_path=session / 'parts.yaml', native=native, runtime=runtime,
                             commission_evidence_path=session / 'commission-evidence.json', resolution_sha256=resolution)


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
def strict_trial(root, cell, run_id, *, cell_dir=None, native=NATIVE, edit=None):
    """A run directory laid out like the qualification runner's, for one strict trial PASS session."""
    session = launch(root / 'launch', cell)
    receipt = json.loads((session / 'runtime/receipt.json').read_text())
    receipt['run_id'] = run_id
    (session / 'runtime/receipt.json').write_text(json.dumps(receipt))
    bound = binding(session, cell, native=native)
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
                  stage_a_acceptance=False, preflight={'repo_dirty': False})
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
    body = dict(schema=psq.SCHEMA, state=psq.QUALIFIED, contract=psq.QUALIFICATION_CONTRACT, trials=readiness['trials'],
                **{k: readiness[k] for k in ('cell', 'world', 'destination', 'native', 'runtime')})
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
        run=str(blocked.resolve()), finding='PREPLAN_TRANSFER: SEARCH_BUDGET_EXHAUSTED: candidate search budget exhausted')]


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
def stopped_trial(root, cell, edit=None, session_edit=None):
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
    run = strict_trial(root, cell, 'run-stop', edit=stop)
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
