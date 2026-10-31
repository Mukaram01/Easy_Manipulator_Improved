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


@pytest.fixture(autouse=True)
def committed_software(monkeypatch):
    planner, mechanism = psq.planner_identity, psq.mechanism_identity
    monkeypatch.setattr(psq, 'planner_identity', lambda repo=psq.REPO, revision=None: planner(repo, revision or HEAD))
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
