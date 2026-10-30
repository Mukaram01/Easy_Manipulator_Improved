"""G4 single-launch sorting runner (scripts/run_g4_sorting.py): the offline parts of every cycle.

Live behaviour (Gazebo, EPD, MoveIt) is covered by plan-only sessions; these tests pin what the runner decides itself:
the measured world is the session's world, the plan is the target's, a cycle counts only once verified against its own
authorisation, the G4 source type is only the routed simulated-camera task, and no code path commands motion.
"""
from __future__ import annotations

import copy
import hashlib
import json
import os
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

import run_g4_sorting as g4  # noqa: E402
from scripts import sorting_session as ss  # noqa: E402
from tests.test_sorting_session import DIMS, FLOOR_A, batch, obj, physical_session, physical_summary  # noqa: E402


def world(**moves):
    rows = [('cube_1', [0.40, -0.27, 0.0125], [0.0, 0.0, 0.0]), ('cup_1', [0.36, -0.33, 0.0125], [0.0, 0.0, math.pi / 2])]
    return [{'object_id': oid, 'pose': {'frame_id': 'world', 'xyz': [v + d for v, d in zip(xyz, moves.get(oid, (0, 0, 0)))],
                                        'rpy': list(rpy)}, 'dimensions': [0.025] * 3} for oid, xyz, rpy in rows]


def test_the_measured_world_must_be_the_sessions_world():
    assert g4.require_same_world(world(), world(cup_1=(4e-6, 0, 0)), 'observe')['max_position_delta_m'] < 1e-5
    wrapped = world()
    wrapped[1]['pose']['rpy'][2] -= 2 * math.pi                                     # the same orientation
    assert g4.require_same_world(world(), wrapped, 'observe')['max_orientation_delta_rad'] < 1e-9
    for measured, match in ((world(cup_1=(2e-5, 0, 0)), 'moved'),                  # nudged beyond the refresh tolerance
                            (world()[:1], 'became'),                                # an object vanished
                            ([dict(o, dimensions=[0.03] * 3) for o in world()], 'size')):
        with pytest.raises(g4.G4Stop, match=match) as stop:
            g4.require_same_world(world(), measured, 'observe')
        assert stop.value.code == 'G4_WORLD_CHANGED'


def test_measured_snapshots_become_session_worlds_unchanged():
    snapshot = {'objects': [dict(o, class_id='unclassified') for o in reversed(world())]}
    assert g4.measured_world(snapshot) == world()


def plan_summary(target='runtime::cup_1', cls='cup', region='default_drop_zone'):
    from run_stage_a1_finish import STAGES
    return {'result': 'PLAN_ONLY', 'full_cycle_prevalidated': True, 'execution_attempted': False,
            'plan_metadata': [{'stage': s, 'success': True, 'moveit_code': 1, 'points': 5} for s in STAGES],
            'task_intent_resolution': {'readiness_status': 'READY', 'place_resolution': {'route': {
                'class_id': cls, 'region_ref': region, 'rule_id': 'cups'}}},
            'selected_object_id': target, 'batch_target_object_id': target,
            'physics_pads': {'allowance_m': {'transfer': 0.015}, 'links': ['l'], 'boxes': 4},
            'physical_destination_walls': {'selected_destination': {'model': 'target_bin_physics'}}}


TARGET = {'id': 'runtime::cup_1', 'class_id': 'cup', 'destination': 'default_drop_zone'}
OBSERVED = {'objects': [{'object_id': 'cup_1', 'class_id': 'cup'}, {'object_id': 'cube_1', 'class_id': 'unperceived'}]}


def test_the_plan_must_be_the_targets_padded_plan_only_route():
    assert g4.check_plan(plan_summary(), TARGET, OBSERVED)['route']['region_ref'] == 'default_drop_zone'
    for summary, observed, match in (
            (plan_summary(target='runtime::cup_2'), OBSERVED, 'not the target'),
            (plan_summary(region='place_zone_b'), OBSERVED, 'resolved route'),
            (plan_summary(cls='bottle'), OBSERVED, 'resolved route'),
            (plan_summary(), {'objects': [{'object_id': 'cup_1', 'class_id': 'unperceived'}]}, 'target class'),
            (dict(plan_summary(), physics_pads=None), OBSERVED, 'padded plan-only')):
        with pytest.raises(g4.G4Stop, match=match):
            g4.check_plan(summary, TARGET, observed)
    with pytest.raises(RuntimeError, match='plan-only cycle did not pass'):     # anything that executed is no plan
        g4.check_plan(dict(plan_summary(), execution_attempted=True), TARGET, OBSERVED)


BINDING = {'destinations': [dict(FLOOR_A, model='bin_a_physics', source_item_id='bin_a')],
           'world': {'objects': [{'object_id': 'cup-1', 'dimensions_m': list(DIMS)}, {'object_id': 'cup-2',
                                                                                       'dimensions_m': list(DIMS)}]}}


def cycle_record(tmp_path, target, cycle=1, name='g4-cycle.json'):
    """A trial cycle record in the authority's format (only the fields the verifier reads)."""
    data = (json.dumps(dict(schema='workcell_g4_sorting_scope/v1', state='G4_AUTHORISED_TRIAL_CYCLE', cycle=cycle,
                            session_sha256='s' * 64, target=dict(object_id=target['id'].split('::')[-1],
                                                                class_id=target['class_id']),
                            destination=dict(region=target['destination'], model='bin_a_physics'), grasp='top_2f::000',
                            plan=dict(resolution_sha256='res-1'))) + '\n').encode()
    path = tmp_path / name
    path.write_bytes(data)
    return path, hashlib.sha256(data).hexdigest()


def summary_file(tmp_path, target, xyz, sha, edit=None):
    summary = dict(physical_summary(target, xyz), selected_extraction_intent={'candidate_id': 'top_2f::000'},
                   resolution_sha256='res-1', physics_scope=dict(cycle_sha256=sha, session_sha256='s' * 64))
    if edit:
        edit(summary)
    path = tmp_path / 'summary.json'
    path.write_text(json.dumps(summary))
    return path


@pytest.fixture(autouse=True)
def committed_control(monkeypatch):
    """The committed HEAD's G4 control identity stands for this checkout (the working tree may be mid-edit)."""
    import physics_scope_qualification as psq
    identity, head = psq.g4_control_identity, psq._git(psq.REPO, 'rev-parse', 'HEAD').strip()
    monkeypatch.setattr(psq, 'g4_control_identity', lambda repo=psq.REPO, revision=None: identity(repo, revision or head))


def test_the_runner_acts_only_for_the_g4_control_code_it_loaded():
    binding = dict(control='a' * 64)
    assert g4.require_loaded_control(binding, 'a' * 64) is binding
    with pytest.raises(g4.G4Stop, match='G4_SOFTWARE_CHANGED') as stop:                # the checkout moved under it
        g4.require_loaded_control(binding, 'b' * 64)
    assert stop.value.fault


def physical(tmp_path):
    s = physical_session()
    batch(s, obj('cup-1', 'cup', 0.30, DIMS), obj('cup-2', 'cup', 0.40, DIMS))
    s.transition('READY'); s.transition('RUNNING')
    s.part_dimensions = {o: list(DIMS) for o in s.objects}
    target = s.next_target(); s.begin_cycle(target)
    return s, target, *cycle_record(tmp_path, target)


def reconcile_worlds(target, placed):
    """The validated world and a fresh measurement after the cycle (the placed part re-measured 2 um away)."""
    before = [{'object_id': 'cup-1', 'pose': {'xyz': [0.30, -0.1, 0.05], 'rpy': [0, 0, 0]}, 'dimensions': list(DIMS)},
              {'object_id': 'cup-2', 'pose': {'xyz': [0.40, -0.1, 0.05], 'rpy': [0, 0, 0]}, 'dimensions': list(DIMS)}]
    after = copy.deepcopy(before)
    oid = target['id'].split('::')[-1]
    next(o for o in after if o['object_id'] == oid)['pose']['xyz'] = [placed[0] + 2e-6, placed[1], placed[2]]
    return before, after, oid


def test_a_cycle_is_counted_only_after_verify_and_reconcile_at_the_re_measured_pose(tmp_path):
    s, target, path, sha = physical(tmp_path)
    measured = [0.581, 0.219, FLOOR_A['floor_z'] + DIMS[2] / 2]
    summary = summary_file(tmp_path, target, measured, sha)
    assert g4.verify_cycle(s, path, sha, summary, BINDING)['verified'] and s.counts()['completed'] == 0
    before, after, _ = reconcile_worlds(target, measured)
    reconciled = g4.reconcile_measurement(s, before, after, target['id'], measured + [0.0, 0.0, 0.0, 1.0])
    assert reconciled['placed_delta_m'] < 1e-5 and s.counts()['completed'] == 0      # measured, not yet counted
    assert g4.count_cycle(s, summary, 0, tmp_path, reconciled) == ss.COMPLETED
    assert s.objects[target['id']]['pose'][0] == measured[0] + 2e-6 and s.objects[target['id']]['measured'] is True
    assert s.next_target()['id'] != target['id']                                   # never picked again


@pytest.mark.parametrize('tamper', ['record', 'target', 'unseated', 'grasp', 'other-cycle', 'missing'])
def test_an_unverified_cycle_never_reaches_the_ledger(tmp_path, tamper):
    s, target, path, sha = physical(tmp_path)
    edit = None
    if tamper == 'unseated':
        edit = lambda m: m['full_cycle_physical_acceptance']['final_pose'].__setitem__(2, 0.08)   # in the air
    elif tamper == 'grasp':
        edit = lambda m: m['selected_extraction_intent'].update(candidate_id='finger_pinch_basic::000')
    elif tamper == 'other-cycle':
        edit = lambda m: m['physics_scope'].update(cycle_sha256='0' * 64)          # the execution of another record
    summary = summary_file(tmp_path, target, [0.58, 0.22, FLOOR_A['floor_z'] + DIMS[2] / 2], sha, edit)
    if tamper == 'record':
        sha = '0' * 64                                                             # not the authorisation it ran under
    elif tamper == 'target':
        path, sha = cycle_record(tmp_path, dict(target, id='runtime::cup-2'), name='other.json')
    elif tamper == 'missing':
        summary = tmp_path / 'no-summary.json'
    with pytest.raises(g4.G4Stop, match='G4_CYCLE_UNVERIFIED') as stop:
        g4.verify_cycle(s, path, sha, summary, BINDING)
    assert stop.value.fault and s.counts()['completed'] == 0 and not s.objects[target['id']]['attempts']


@pytest.mark.parametrize('fault', ['neighbour', 'placed'])
def test_an_unreconciled_cycle_faults_the_session_uncounted(tmp_path, fault):
    s, target, path, sha = physical(tmp_path)
    placed = [0.581, 0.219, FLOOR_A['floor_z'] + DIMS[2] / 2]
    assert g4.verify_cycle(s, path, sha, summary_file(tmp_path, target, placed, sha), BINDING)['verified']
    before, after, oid = reconcile_worlds(target, placed)
    final = placed + [0.0, 0.0, 0.0, 1.0]
    if fault == 'neighbour':
        next(o for o in after if o['object_id'] != oid)['pose']['xyz'][0] += 5e-5   # a neighbour pushed by the place
    else:
        final = [placed[0] + 0.01] + placed[1:] + [0.0, 0.0, 0.0, 1.0]           # not where the executor measured it
    with pytest.raises(g4.G4Stop, match='G4_WORLD_CHANGED') as stop:
        g4.reconcile_measurement(s, before, after, target['id'], final)
    assert stop.value.fault and s.active is None and s.counts()['completed'] == 0


def test_the_sessions_expected_bin_contents_come_from_the_ledger_world():
    s = physical_session()
    batch(s, obj('cup-1', 'cup', 0.30, DIMS), obj('cup-2', 'cup', 0.40, DIMS))
    floor = dict(FLOOR_A, floor_xyz=[0.30, -0.1, 0.003], floor_size_m=[0.05, 0.05])   # a small bin under cup-1
    assert g4.expected_occupants(s, floor) == ['cup-1'] and g4.expected_occupants(s, FLOOR_A) == []


def test_a_refused_plan_is_a_motionless_ledger_attempt_and_the_session_continues(tmp_path):
    s, target, _, _ = physical(tmp_path)
    s.policy['max_attempts_per_object'] = 1
    path = tmp_path / 'summary.json'
    path.write_text(json.dumps(dict(result='FAIL', execution_attempted=False, failed_stage='PREPLAN_GRASP',
                                    failure='SEARCH_BUDGET_EXHAUSTED: candidate search budget exhausted')))
    assert g4.plan_refused(s, path, 1, tmp_path) == ss.REJECTED
    assert s.objects[target['id']]['status'] == ss.REJECTED and s.counts()['completed'] == 0
    assert s.next_target()['id'] != target['id']                                   # the next object, same session
    assert any(o['id'] == target['id'] for o in s.occupied())                      # still an obstacle


def test_a_refused_plan_without_evidence_or_with_motion_faults(tmp_path):
    s, target, _, _ = physical(tmp_path)
    with pytest.raises(ss.SortingFault, match='CYCLE_EVIDENCE_MISSING'):
        g4.plan_refused(s, tmp_path / 'missing.json', 1, tmp_path)
    s.begin_cycle(target)
    path = tmp_path / 'moved.json'
    path.write_text(json.dumps(dict(result='FAIL', execution_attempted=True)))
    with pytest.raises(g4.G4Stop, match='G4_PLAN_MOVED'):
        g4.plan_refused(s, path, 1, tmp_path)
    assert s.counts()['completed'] == 0


def test_a_plan_only_session_never_repeats_a_planned_object_and_counts_nothing():
    s = physical_session()
    batch(s, obj('cup-1', 'cup', 0.30, DIMS), obj('cup-2', 'cup', 0.40, DIMS), obj('box-1', 'box', 0.35, DIMS))
    s.transition('READY'); s.transition('RUNNING')
    first = s.next_target()
    second = s.next_target(exclude={first['id']})
    assert second['id'] not in (first['id'],) and s.next_target(exclude={first['id'], second['id']})['class_id'] == 'box'
    assert s.objects[first['id']]['status'] == ss.PENDING and s.counts()['completed'] == 0
    assert 'Gazebo Fortress physics' in s.status()['execution']


def test_the_g4_source_type_is_only_the_routed_simulated_camera_task_in_physics():
    from task_intent_v2 import EPD_BOUND_SIMULATION_SOURCES, freshness_source_type
    from tests.test_class_routing import routed_intent, rule, valid_intent
    routed = routed_intent([rule('cup', 'drop_a')])
    routed['perception'] = {'mode': 'live_epd', 'camera_source': 'simulated_rgbd'}
    routed['pick']['selection']['source_type'] = 'perception'
    for backend, expected in (('simulator', 'rendered_perception_in_simulation'), ('fake', 'simulated_rgbd_fake_hardware')):
        routed['safety'] = dict(routed.get('safety') or {}, execution_backend=backend)
        assert freshness_source_type(routed) == expected
    single = copy.deepcopy(valid_intent())                                         # unrouted: unchanged
    single['perception'] = {'mode': 'live_epd', 'camera_source': 'simulated_rgbd'}
    single['pick']['selection']['source_type'] = 'perception'
    single['safety'] = dict(single.get('safety') or {}, execution_backend='simulator')
    assert freshness_source_type(single) == 'perception'
    routed['perception'] = {'mode': 'live_epd', 'camera_source': 'realsense_d435i'}  # a real camera is never this
    routed['safety']['execution_backend'] = 'simulator'
    assert freshness_source_type(routed) == 'live_epd_realsense'
    assert EPD_BOUND_SIMULATION_SOURCES == ('replayed_perception_in_simulation', 'rendered_perception_in_simulation')


def test_motion_happens_only_after_a_granted_trial_authorisation():
    """The runner's one motion command comes after a granted authorisation and the stop latch; a plan-only session can
    never be granted (physics_scope_qualification tests); nothing is retried."""
    source = (ROOT / 'scripts/run_g4_sorting.py').read_text()
    assert source.count("gate='full-cycle'") == 1 and "'--start'" not in source and 'physics_scope=' not in source
    motion = source.index("gate='full-cycle'")
    assert source.index("record['steps']['authorise'] = dict(granted=True") < motion
    assert source.index("'STOPPED_BEFORE_EXECUTION'") < motion
    assert source.index("if not str(refused).startswith('G4_SCOPE_PLAN_ONLY')") < motion
    for flag in ("'--g4-session'", "'--g4-cycle'", "'--g4-plan'", "'--target-object-id'"):
        assert flag in source[motion:motion + 900]
    assert "G4_EXECUTION_REFUSED" in source and 'retry' not in source[motion:].lower().replace('never retried', '')


def test_the_builder_and_hmi_never_start_or_authorise_a_g4_trial():
    for source in (ROOT / 'workcell_builder').rglob('*'):
        if source.suffix in ('.py', '.cpp', '.hpp', '.h'):
            text = source.read_text(errors='replace')
            assert '--authorise-g4-trial' not in text and '--g4-cycle' not in text and 'run_g4_sorting' not in text, source


@pytest.mark.parametrize('argv,match', [
    (['--authorise-g4-trial', '--max-cycles', '2'], 'needs --prior-evidence'),
    (['--authorise-g4-trial', '--prior-evidence', 'p'], 'needs --prior-evidence and --max-cycles'),
    (['--authorise-g4-trial', '--prior-evidence', 'p', '--max-cycles', '3'], '--max-cycles 1..2'),
    (['--authorise-g4-trial', '--prior-evidence', 'p', '--max-cycles', '2', '--plan-every-target'], 'plan-only'),
    (['--prior-evidence', 'p'], 'applies to --authorise-g4-trial only'),
    (['--max-cycles', '0'], 'at least 1')])
def test_a_trial_is_explicit_and_bounded_at_the_command_line(tmp_path, capsys, argv, match):
    with pytest.raises(SystemExit):
        g4.main(['--scene-package', str(tmp_path), '--output', str(tmp_path / 'out'), *argv])
    assert match in capsys.readouterr().err and not (tmp_path / 'out').exists()


@pytest.mark.parametrize('extra,match', [
    (['--g4-session', 's.json'], 'a G4 cycle needs'),
    (['--g4-session', 's', '--g4-session-sha256', 'x', '--g4-cycle', 'c', '--g4-cycle-sha256', 'y', '--g4-plan', 'p'],
     'a G4 cycle is one strict simulator full-cycle'),                         # no --start / commission / evidence
    (['--start', '--simulator-commission', 'full-cycle', '--commission-evidence', 'e', '--backend', 'simulator',
      '--retention-policy', 'record', '--g4-session', 's', '--g4-session-sha256', 'x', '--g4-cycle', 'c',
      '--g4-cycle-sha256', 'y', '--g4-plan', 'p'], 'a G4 cycle is one strict simulator full-cycle'),
    (['--start', '--simulator-commission', 'full-cycle', '--commission-evidence', 'e', '--backend', 'simulator',
      '--resolve-task', '--g4-session', 's', '--g4-session-sha256', 'x', '--g4-cycle', 'c', '--g4-cycle-sha256', 'y',
      '--g4-plan', 'p'], 'a G4 cycle is one strict simulator full-cycle'),     # it consumes its plan, never re-searches
    (['--start', '--backend', 'simulator', '--resolve-task'], '--target-object-id is plan-only')])
def test_the_executor_command_line_admits_a_g4_cycle_only_in_its_exact_form(extra, match):
    import subprocess
    executor = ROOT / 'scripts/perceived_object_grasp_execute.py'
    base = [sys.executable, str(executor), '--summary-output', '/dev/null', '--detections', 'x.yaml',
            '--target-object-id', 'runtime::a']
    result = subprocess.run(base + extra, capture_output=True, text=True, timeout=60)
    assert result.returncode == 2 and match in result.stderr, result.stderr[-400:]


def test_the_executor_consumes_the_authorised_plan_and_consumes_the_cycle_at_the_motion_boundary():
    text = (ROOT / 'scripts/perceived_object_grasp_execute.py').read_text()
    admit, plan = text.index('g4_admission = scope_qualification.g4_admit_execution('), text.index('cycle = plan_authored_cycle(')
    consume, first_motion = text.index('scope_qualification.g4_consume_cycle('), text.index("for step in cycle['steps']:")
    assert admit < plan < consume < first_motion                                # admitted before planning, spent before motion
    assert "expected_resolution = consume_resolution(g4_admission['resolution'], intent, physical, document)" in text
    assert 'G4_PLAN_DIVERGED' in text and "summary['physics_scope'] = dict(kind='G4_TRIAL'" in text
    # Fail closed: --g4-* flags without an admitted multi-destination trial cycle never reach planning or motion.
    missing = text.index("raise RuntimeError('G4_ADMISSION_MISSING: a G4 cycle runs only when its admission ran")
    boundary = text.index("raise RuntimeError('G4_ADMISSION_MISSING at the motion boundary')")
    reverified = text.index("scope_qualification._g4_session(args.g4_session, args.g4_session_sha256, current=g4_current(), "
                            "role='executor')")
    assert admit < missing < plan < boundary < reverified < consume < first_motion
    assert text.count('destination_binding=scoped_destination') == 1 and 'scoped_destination = selected' in text


@pytest.mark.parametrize('argv,match', [(['--domain', '200'], 'render domains'), (['--domain', '0'], 'render domains'),
                                        (['--segment-planning-time', '0'], 'planning budget')])
def test_the_runner_refuses_unsafe_settings(tmp_path, capsys, argv, match):
    with pytest.raises(SystemExit):
        g4.main(['--scene-package', str(tmp_path), '--output', str(tmp_path / 'out'), *argv])
    assert match in capsys.readouterr().err and not (tmp_path / 'out').exists()


def test_a_cell_that_is_not_a_routed_simulated_camera_task_is_refused(tmp_path):
    """An unrouted G3C-style task never opens a G4 session (report written, nothing launched)."""
    cell = tmp_path / 'cell'
    (cell / 'config').mkdir(parents=True)
    from tests.test_class_routing import valid_intent
    intent = valid_intent()
    intent['safety'] = dict(intent.get('safety') or {}, execution_backend='simulator', require_fake_hardware=False,
                            real_hardware_enabled=False)
    intent['schema'] = 'workcell_builder_task_intent/v2'
    (cell / 'config/workcell_builder_task_intent.yaml').write_text(json.dumps(intent))
    for name in ('environment.yaml', 'cell_definition.yaml'):
        (cell / name).write_text('{}\n')
    assert g4.main(['--scene-package', str(cell), '--output', str(tmp_path / 'out')]) == 4
    report = json.loads((tmp_path / 'out' / g4.REPORT_NAME).read_text())
    assert report['stop']['code'] == 'G4_CELL_UNSUPPORTED' and 'shutdown' not in report


@pytest.mark.parametrize('outcome,clean,status,code', [
    ('pass', True, 'PASS', 0), ('pass', False, 'BLOCKED', 1),                   # a clean result is never claimed
    ('fault', False, 'FAULT', 5), ('cancel', False, 'CANCELLED', 3),            # a failed shutdown never hides these
    ('refused', True, 'STOPPED', 4), ('latched', True, 'STOPPED', 4)])
def test_a_shutdown_failure_never_hides_a_fault_and_any_latch_reports_stopped(tmp_path, monkeypatch, outcome, clean,
                                                                              status, code):
    def session(args, repo, workspace, report):
        report['_state'] = {'launch': object()}
        if outcome == 'fault':
            raise g4.G4Stop('G4_CYCLE_FAILED', 'moved, unverified', fault=True)
        if outcome == 'cancel':
            raise g4.OperatorCancel('SIGTERM')
        if outcome == 'refused':
            raise g4.G4Stop('G4_CELL_UNSUPPORTED', 'not a G4 cell')
        if outcome == 'latched':
            g4.STOP_AFTER_CYCLE.record = {'signal': 'SIGUSR1'}               # latched during the batch's last cycle
            return 'BATCH_COMPLETE'
        return 'MAX_CYCLES'
    monkeypatch.setattr(g4, 'session', session)
    monkeypatch.setattr(g4.stage_a, 'stop_owned', lambda launch, log=None: dict(clean=clean))
    try:
        assert g4.main(['--scene-package', str(tmp_path), '--output', str(tmp_path / 'out')]) == code
    finally:
        g4.STOP_AFTER_CYCLE.__init__()
    report = json.loads((tmp_path / 'out' / g4.REPORT_NAME).read_text())
    assert report['status'] == status and (clean or report['shutdown_failure'])
    assert report['runner']['pid'] == os.getpid() and 'planning_trace' in report       # the trial issuer, the trace


@pytest.mark.parametrize('trace', [None, 'relative/trace', 'non-empty', 'a-file'])
def test_a_trial_needs_its_own_absolute_planning_trace_before_anything_starts(tmp_path, monkeypatch, capsys, trace):
    """C1: the certified trajectories are trial evidence, so a trial without a fresh absolute trace never starts."""
    if trace == 'a-file':
        trace = tmp_path / 'trace_file'
        trace.write_text('x')
    elif trace == 'non-empty':
        trace = tmp_path / 'trace'
        trace.mkdir()
        (trace / 'certificate_1_0.result.txt').write_text('result 0\n')
    if trace is None:
        monkeypatch.delenv('WORKCELL_PLANNING_TRACE_DIR', raising=False)
    else:
        monkeypatch.setenv('WORKCELL_PLANNING_TRACE_DIR', str(trace))
    with pytest.raises(SystemExit):
        g4.main(['--scene-package', str(tmp_path), '--output', str(tmp_path / 'out'), '--authorise-g4-trial',
                 '--prior-evidence', str(tmp_path), '--max-cycles', '2'])
    assert 'WORKCELL_PLANNING_TRACE_DIR' in capsys.readouterr().err and not (tmp_path / 'out').exists()


def test_a_runner_that_cannot_identify_itself_is_blocked_with_a_report(tmp_path, monkeypatch):
    """C2: a failed /proc read is a BLOCKED report, never a runner crash without one."""
    import physics_scope_qualification as psq

    def unreadable(pid):
        raise OSError('no /proc')
    monkeypatch.setattr(psq, 'process_identity', unreadable)
    monkeypatch.setattr(g4, 'session', lambda *a: 'MAX_CYCLES')
    assert g4.main(['--scene-package', str(tmp_path), '--output', str(tmp_path / 'out')]) == 1
    report = json.loads((tmp_path / 'out' / g4.REPORT_NAME).read_text())
    assert report['status'] == 'BLOCKED' and 'no /proc' in report['failure'] and 'runner' not in report
