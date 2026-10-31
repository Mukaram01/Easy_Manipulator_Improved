"""Operator view of a G4 physics sorting session: read from the session's evidence files only, never inferred."""
import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

import g4_session_view  # noqa: E402

FRAME = {'source': {'producer': 'simulator_render', 'provenance': 'SIMULATOR_RENDERED_SYNTHETIC',
                    'camera_id': 'realsense_overhead', 'capture_wall_ns': 1, 'world_revision': 'abc'}, 'objects': []}
OBSERVED = {'objects': [
    {'object_id': 'cup_1', 'class_id': 'cup', 'confidence': 0.74, 'pose': {'xyz': [0.36, -0.33, 0.0125]},
     'dimensions': [0.025] * 3},
    {'object_id': 'cube_1', 'class_id': 'unperceived', 'confidence': None, 'pose': {'xyz': [0.4, -0.27, 0.0125]},
     'dimensions': [0.025] * 3}]}
PLANNED = {'result': 'PLAN_ONLY', 'full_cycle_plan_success': True, 'execution_attempted': False,
           'batch_target_object_id': 'runtime::cup_1', 'selected_object_id': 'runtime::cup_1',
           'selected_extraction_intent': {'candidate_id': 'top_2f::000'},
           'plan_metadata': [{'stage': 'PREPLAN_APPROACH'}, {'stage': 'PREPLAN_HOME'}],
           'task_intent_resolution': {'place_resolution': {'route': {
               'class_id': 'cup', 'rule_id': 'cups', 'region_ref': 'default_drop_zone', 'asset_ref': 'target_bin_default',
               'world_pose': {'xyz_m': [0.58, 0.22, 0.0765]}}}}}
LEDGER = {'state': 'RUNNING', 'state_reason': 'sorting started', 'fault': None,
          'counts': {'completed': 0, 'pending': 1, 'not_eligible': 1},
          'objects': [{'id': 'runtime::cup_1', 'object_id': 'cup_1', 'class_id': 'cup', 'status': 'PENDING'},
                      {'id': 'runtime::cube_1', 'object_id': 'cube_1', 'class_id': 'unperceived', 'status': 'NOT_ELIGIBLE',
                       'reason': 'UNPERCEIVED: EPD did not classify it in the current frame; it stays an obstacle'}]}


def session(tmp_path, *, planned=True, report=None, launch=''):
    run = tmp_path / 'run'
    cycle = run / 'session/cycles/001'
    for sub in ('frame', 'classify', 'resolve'):
        (cycle / sub).mkdir(parents=True)
    (cycle / 'frame/frame.yaml').write_text(yaml.safe_dump(FRAME))
    (cycle / 'classify/observations.yaml').write_text(yaml.safe_dump(OBSERVED))
    if planned:
        (cycle / 'resolve/summary.json').write_text(json.dumps(PLANNED))
    (run / 'session/g4-session.json').write_text(json.dumps({'state': 'G4_PLAN_ONLY'}))
    (run / 'session/launch.log').write_text(launch)
    (run / 'sorting_session.json').write_text(json.dumps(LEDGER))
    if report is not None:
        (run / g4_session_view.REPORT).write_text(json.dumps(report))
    return run


def test_a_running_session_shows_what_its_files_hold_and_nothing_more(tmp_path):
    view = g4_session_view.view(session(tmp_path, planned=False))
    assert view['finished'] is False and view['status'] is None and view['shutdown'] is None
    assert view['session_state'] == 'G4_PLAN_ONLY'
    cycle, = view['cycles']
    assert cycle['frame']['producer'] == 'simulator_render'
    assert [(o['object_id'], o['class_id']) for o in cycle['objects']] == [('cup_1', 'cup'), ('cube_1', 'unperceived')]
    assert cycle['plan'] == {'state': 'PLANNING'}                    # the resolve has not written its summary yet
    assert cycle['authorisation'] is None and cycle['outcome'] is None   # decided by the runner, reported at the end
    assert view['motion'] == {'authorised_cycles': 0, 'executed_cycles': 0, 'controller_goals': 0}


def test_a_plan_is_the_resolve_summary_and_marks_its_object_planned_not_completed(tmp_path):
    view = g4_session_view.view(session(tmp_path))
    plan = view['cycles'][0]['plan']
    assert plan['state'] == 'PLANNED' and plan['target'] == 'runtime::cup_1' and plan['grasp'] == 'top_2f::000'
    assert (plan['destination'], plan['bin']) == ('default_drop_zone', 'target_bin_default')
    objects = {o['object_id']: o for o in view['objects']}
    assert objects['cup_1']['status'] == 'PENDING' and objects['cup_1']['plan']['state'] == 'PLANNED'
    assert objects['cube_1']['plan'] is None and objects['cube_1']['status'] == 'NOT_ELIGIBLE'
    assert view['ledger']['counts']['completed'] == 0


def test_the_runner_report_supplies_each_cycles_authorisation_and_the_shutdown(tmp_path):
    refusal = 'G4_SCOPE_PLAN_ONLY: this session admits planning only; no cycle may move'
    report = {'scope': 'G4_PLAN_ONLY_NO_MOTION', 'status': 'PASS', 'result': 'BATCH_COMPLETE', 'duration_s': 600.0,
              'cycles': [{'cycle': 1, 'outcome': 'PLAN_ONLY', 'steps': {'authorise': {'granted': False, 'refusal': refusal}}}],
              'shutdown': {'clean': True, 'surviving_owned_processes': []}}
    view = g4_session_view.view(session(tmp_path, report=report))
    assert view['finished'] and view['result'] == 'BATCH_COMPLETE' and view['shutdown'] == {'clean': True, 'surviving': 0}
    assert view['cycles'][0]['authorisation'] == {'granted': False, 'refusal': refusal}
    assert view['cycles'][0]['outcome'] == 'PLAN_ONLY'


def test_motion_is_counted_from_the_evidence_never_assumed(tmp_path):
    goal = '[move_group-5] [INFO] [1793403000.000000] [ur5_arm_controller]: Accepted new action goal\n'
    run = session(tmp_path, launch=goal * 3)
    (run / 'session/cycles/001/g4-cycle.json').write_text('{}')
    (run / 'session/cycles/001/execute').mkdir()
    (run / 'session/cycles/001/execute/summary.json').write_text('{}')
    view = g4_session_view.view(run)
    assert view['motion'] == {'authorised_cycles': 1, 'executed_cycles': 1, 'controller_goals': 3}
    assert view['cycles'][0]['executed'] is True


def test_cli_prints_the_view_and_refuses_bad_usage(tmp_path):
    run = session(tmp_path)
    out = subprocess.run([sys.executable, ROOT / 'scripts/g4_session_view.py', run], capture_output=True, text=True,
                         check=True)
    assert json.loads(out.stdout)['schema'] == g4_session_view.SCHEMA
    assert subprocess.run([sys.executable, ROOT / 'scripts/g4_session_view.py'], capture_output=True).returncode == 2
