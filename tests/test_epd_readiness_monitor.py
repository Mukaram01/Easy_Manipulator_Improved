"""EPD per-detection readiness -> Workcell Studio monitor contract.

Fixture: real EPD PR #234 (59c31f6) detection_readiness output recorded from the 12-frame
D435i replay (hybrid refinement + tracking), exported from CDR with source hashes.
"""
from pathlib import Path
import copy
import importlib.util
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'workcell_builder/workcell_builder/scripts/epd_readiness_monitor_node.py'
FIXTURE = ROOT / 'tests/fixtures/epd_pr234_detection_readiness_hybrid_tracking.jsonl'

spec = importlib.util.spec_from_file_location('epd_readiness_monitor', SCRIPT)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

REPORTS = {r['frame']: r for r in map(json.loads, FIXTURE.read_text().splitlines())}
T0 = 1000.0


def fresh(*frames, timeout=2.0):
    monitor = mod.ReadinessMonitor(stale_timeout_s=timeout)
    for i, frame in enumerate(frames):
        assert monitor.ingest(REPORTS[frame], T0 + i) in ('accepted', 'restart')
    return monitor, monitor.snapshot(T0 + len(frames) - 1, publisher_count=1)


def test_all_recorded_frames_report_every_proposal():
    monitor = mod.ReadinessMonitor()
    totals = {s: 0 for s in mod.STATES}
    for i, frame in enumerate(sorted(REPORTS)):
        assert monitor.ingest(REPORTS[frame], T0 + i) == 'accepted'
        snap = monitor.snapshot(T0 + i, 1)
        assert snap['contract_errors'] == []
        assert len(snap['objects']) == len(REPORTS[frame]['statuses'])
        for state, count in snap['state_counts'].items():
            totals[state] += count
    assert totals == {'GRASP_CANDIDATE': 28, 'LOCALISED': 4, 'REJECTED': 8, 'DETECTED': 0, 'POSE_VALID': 0}


def test_a_valid_grasp_candidate_displayed_with_pose_but_not_planning_input():
    _, snap = fresh('A_00')
    assert snap['connection_state'] == 'FRESH'
    assert snap['frame_id'] == 'camera_color_optical_frame'
    candidates = [o for o in snap['objects'] if o['state'] == 'GRASP_CANDIDATE']
    assert sorted(o['class'] for o in candidates) == ['apple', 'banana', 'banana', 'mouse']
    for o in candidates:
        assert o['orientation_valid'] is True
        assert len(o['position_m']) == 3 and o['position_m'][2] > 0
        assert o['track_id']
        assert o['planning_input'] is False
    assert snap['motion_authority'] is False


def test_b_round_apple_stays_localised_without_orientation():
    for frame in ('B_00', 'B_01', 'B_02', 'B_03'):
        _, snap = fresh(frame)
        apples = [o for o in snap['objects'] if o['class'] == 'apple']
        assert len(apples) == 1
        apple = apples[0]
        assert apple['state'] == 'LOCALISED'
        assert apple['orientation_valid'] is False
        assert apple['position_m'] and abs(apple['position_m'][2] - 0.4735) < 0.002
        assert 'ambiguous_axis_no_pose' in apple['reasons']
        assert apple['planning_input'] is False


def test_c_ownership_conflicts_remain_rejected_with_reasons():
    _, snap = fresh('C_00')
    rejected = [o for o in snap['objects'] if o['state'] == 'REJECTED']
    assert sorted(o['class'] for o in rejected) == ['apple', 'banana']
    for o in rejected:
        assert o['reasons'] == ['overlapping_ownership']
        assert o['position_m'] is None and o['orientation_valid'] is False


def test_d_identity_comes_only_from_epd_track_ids():
    monitor, _ = fresh('B_00', 'B_01', 'B_02', 'B_03', 'C_00')
    states = []
    for i, frame in enumerate(('B_03', 'C_00')):
        m = mod.ReadinessMonitor()
        m.ingest(REPORTS[frame], T0)
        apple = next(o for o in m.snapshot(T0, 1)['objects'] if o['class'] == 'apple' and o['state'] != 'REJECTED')
        states.append((apple['state'], apple['track_id']))
    assert states == [('LOCALISED', '6'), ('GRASP_CANDIDATE', '6')]
    # Localisation mode carries no track id: the monitor never invents one.
    report = copy.deepcopy(REPORTS['B_00'])
    for status in report['statuses']:
        status['values']['track_id'] = ''
    m = mod.ReadinessMonitor()
    m.ingest(report, T0)
    assert all(o['track_id'] is None for o in m.snapshot(T0, 1)['objects'])


def test_e_disconnect_stale_out_of_order_and_restart():
    monitor = mod.ReadinessMonitor(stale_timeout_s=2.0)
    assert monitor.snapshot(T0, None)['connection_state'] == 'UNKNOWN'
    monitor.ingest(REPORTS['B_01'], T0)
    assert monitor.snapshot(T0 + 1, 1)['connection_state'] == 'FRESH'
    stale = monitor.snapshot(T0 + 5, 0)
    assert stale['connection_state'] == 'STALE'
    assert 'disconnected' in stale['connection_reason']
    assert stale['objects'] == [] and stale['last_object_count'] == 3
    # Older sensor stamp is rejected; identical stamp is a duplicate.
    assert monitor.ingest(REPORTS['B_00'], T0 + 6) == 'out_of_order'
    assert monitor.ingest(REPORTS['B_01'], T0 + 6) == 'duplicate'
    assert monitor.snapshot(T0 + 6, 1)['connection_state'] == 'STALE'
    # Reconnect with newer data.
    assert monitor.ingest(REPORTS['B_02'], T0 + 7) == 'accepted'
    assert monitor.snapshot(T0 + 7, 1)['connection_state'] == 'FRESH'
    # EPD restart: sensor time advances, observation counter resets.
    restarted = copy.deepcopy(REPORTS['B_03'])
    for status in restarted['statuses']:
        status['values']['observation_id'] = '1'
    assert monitor.ingest(restarted, T0 + 8) == 'restart'
    snap = monitor.snapshot(T0 + 8, 1)
    assert snap['epd_session'] == 1 and snap['counters']['epd_restarts'] == 1
    assert snap['counters']['out_of_order'] == 1 and snap['counters']['duplicate'] == 1


def test_disappearing_objects_and_empty_frames_are_not_carried_forward():
    monitor = mod.ReadinessMonitor()
    monitor.ingest(REPORTS['C_01'], T0)
    assert len(monitor.snapshot(T0, 1)['objects']) == 4
    monitor.ingest(REPORTS['C_02'], T0 + 1)
    assert [o['class'] for o in monitor.snapshot(T0 + 1, 1)['objects']] == ['apple', 'banana']
    empty = dict(REPORTS['C_03'], statuses=[], stamp=[REPORTS['C_03']['stamp'][0] + 1, 0])
    assert monitor.ingest(empty, T0 + 2) == 'accepted'
    assert monitor.snapshot(T0 + 2, 1)['objects'] == []


def test_f_legacy_epd_without_readiness_topic_is_unavailable_not_fabricated():
    snap = mod.ReadinessMonitor().snapshot(T0, publisher_count=0)
    assert snap['connection_state'] == 'UNAVAILABLE'
    assert snap['objects'] == [] and snap['observation_id'] is None
    # Existing legacy snapshot connector is untouched and still imports.
    legacy = ROOT / 'workcell_builder/workcell_builder/scripts/epd_to_workcell_snapshot_node.py'
    legacy_spec = importlib.util.spec_from_file_location('legacy_connector', legacy)
    legacy_spec.loader.exec_module(importlib.util.module_from_spec(legacy_spec))


def test_contract_violations_are_never_promoted():
    report = copy.deepcopy(REPORTS['B_00'])
    apple = report['statuses'][0]['values']
    apple['orientation_valid'] = 'true'
    apple['axis'] = '1,0,0'
    rejected = report['statuses'][1]['values']
    rejected['position_valid'] = 'true'
    rejected['centroid_m'] = '0.1,0.1,0.5'
    report['statuses'][2]['values']['state'] = 'EXECUTE_NOW'
    m = mod.ReadinessMonitor()
    m.ingest(report, T0)
    snap = m.snapshot(T0, 1)
    objects = {o['index']: o for o in snap['objects']}
    assert objects[0]['state'] == 'LOCALISED' and objects[0]['orientation_valid'] is False
    assert objects[1]['state'] == 'REJECTED' and objects[1]['position_m'] is None
    assert objects[2]['state'] == 'UNKNOWN'
    for index in (0, 1, 2):
        assert any(e.startswith(f'detection {index}:') for e in snap['contract_errors'])
    assert objects[2]['position_m'] is None and objects[2]['orientation_valid'] is False


def test_frame_and_stamp_are_required():
    m = mod.ReadinessMonitor(expected_frame_id='camera_color_optical_frame')
    assert m.ingest(dict(REPORTS['A_00'], frame_id='other'), T0) == 'invalid'
    assert m.ingest(dict(REPORTS['A_00'], stamp=[0, 0]), T0) == 'invalid'
    assert m.snapshot(T0, 1)['connection_state'] == 'UNKNOWN'


def test_atomic_write_round_trip(tmp_path):
    monitor, snap = fresh('B_00')
    out = tmp_path / 'preview' / 'epd_detection_readiness.json'
    mod.write_atomic(out, snap)
    assert json.loads(out.read_text())['schema_version'] == mod.SCHEMA
    assert [p.name for p in out.parent.iterdir()] == ['epd_detection_readiness.json']


def test_readiness_never_reaches_planning_or_publishes():
    source = SCRIPT.read_text()
    assert 'create_publisher' not in source and 'create_client' not in source
    planners = subprocess.run(
        ['grep', '-rl', 'detection_readiness', str(ROOT / 'easy_manipulation_deployment')],
        capture_output=True, text=True)
    assert planners.stdout.strip() == ''
