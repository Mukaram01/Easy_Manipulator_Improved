"""G4 single-launch sorting runner (scripts/run_g4_sorting.py): the offline parts of every cycle.

Live behaviour (Gazebo, EPD, MoveIt) is covered by plan-only sessions; these tests pin what the runner decides itself:
the measured world is the session's world, the plan is the target's, a cycle counts only once verified against its own
authorisation, the G4 source type is only the routed simulated-camera task, and no code path commands motion.
"""
from __future__ import annotations

import copy
import hashlib
import json
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


def cycle_record(tmp_path, target, cycle=1):
    data = (json.dumps(dict(schema='workcell_g4_sorting_scope/v1', state='G4_AUTHORISED_CYCLE', cycle=cycle,
                            target=dict(object_id=target['id'].split('::')[-1], class_id=target['class_id']),
                            destination=target['destination'], grasp='top_2f::000')) + '\n').encode()
    path = tmp_path / 'g4-cycle.json'
    path.write_bytes(data)
    return path, hashlib.sha256(data).hexdigest()


def summary_of(target, xyz):
    return dict(physical_summary(target, xyz), selected_extraction_intent={'candidate_id': 'top_2f::000'})


def physical(tmp_path):
    s = physical_session()
    batch(s, obj('cup-1', 'cup', 0.30, DIMS), obj('cup-2', 'cup', 0.40, DIMS))
    s.transition('READY'); s.transition('RUNNING')
    s.part_dimensions = {o: list(DIMS) for o in s.objects}
    target = s.next_target(); s.begin_cycle(target)
    return s, target, *cycle_record(tmp_path, target)


def test_a_verified_cycle_counts_and_reconciles_at_the_measured_pose(tmp_path):
    s, target, path, sha = physical(tmp_path)
    measured = [0.581, 0.219, FLOOR_A['floor_z'] + DIMS[2] / 2]
    verification, outcome = g4.verify_and_reconcile(s, path, sha, summary_of(target, measured), 0, tmp_path)
    assert verification['verified'] and outcome == ss.COMPLETED
    assert s.objects[target['id']]['pose'][:3] == measured and s.objects[target['id']]['measured'] is True
    assert s.next_target()['id'] != target['id']                                   # never picked again


@pytest.mark.parametrize('tamper', ['record', 'target', 'unseated', 'grasp'])
def test_an_unverified_cycle_never_reaches_the_ledger(tmp_path, tamper):
    s, target, path, sha = physical(tmp_path)
    summary = summary_of(target, [0.58, 0.22, FLOOR_A['floor_z'] + DIMS[2] / 2])
    if tamper == 'record':
        sha = '0' * 64                                                             # not the authorisation it ran under
    elif tamper == 'target':
        other = dict(target, id='runtime::cup-2')
        path, sha = cycle_record(tmp_path, other)                                  # authorised for another object
    elif tamper == 'unseated':
        summary['full_cycle_physical_acceptance']['final_pose'][2] = 0.08          # not resting on the bound floor
    else:
        summary['selected_extraction_intent']['candidate_id'] = 'finger_pinch_basic::000'   # not the authorised grasp
    with pytest.raises(g4.G4Stop, match='G4_CYCLE_UNVERIFIED'):
        g4.verify_and_reconcile(s, path, sha, summary, 0, tmp_path)
    assert s.counts()['completed'] == 0 and not s.objects[target['id']]['attempts']


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


def test_no_runner_path_commands_motion():
    """The runner never gives the executor --start or a commissioning gate: its only Execute step refuses."""
    source = (ROOT / 'scripts/run_g4_sorting.py').read_text()
    assert "'--start'" not in source and 'gate=' not in source and 'physics_scope=' not in source
    assert source.count('stage_a.executor_command(') == 1 and 'resolve=True' in source
    assert "G4Stop('G4_EXECUTOR_ADMISSION_MISSING'" in source


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
