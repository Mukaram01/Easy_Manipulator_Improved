"""G4 numeric tracking evidence: certificate chaining, controller goal windows, the measurement stream and the JTC
quintic (the full metric is exercised on recorded trial evidence; see log/qualification/g4)."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

import g4_tracking_evidence as tracking  # noqa: E402


def trajectory(*states, step=1.0):
    duration = lambda k: SimpleNamespace(sec=int(k * step), nanosec=int((k * step % 1) * 1e9))
    return SimpleNamespace(joint_names=[f'j{i}' for i in range(6)], points=[
        SimpleNamespace(positions=list(q), velocities=[0.0] * 6, accelerations=[0.0] * 6, time_from_start=duration(k))
        for k, q in enumerate(states)])


def test_each_stage_is_the_certificate_chained_back_from_the_executed_home_inside_the_cycle():
    q = [[float(i)] * 6 for i in range(9)]
    stages = dict(APPROACH=(q[0], q[1]), GRASP=(q[1], q[2]), LIFT=(q[2], q[3]), TRANSFER=(q[3], q[4]),
                  PLACE=(q[4], q[5]), RETREAT=(q[5], q[6]), HOME=(q[6], q[7]))
    certs = [(100 + 10 * k, f'certificate_{k}', trajectory(*states), True) for k, states in enumerate(stages.values())]
    earlier = (95, 'certificate_old_approach', trajectory(q[0], q[1]), True)     # an earlier attempt of the same stage
    refused = (101, 'certificate_refused_approach', trajectory(q[0], [0.5] * 6, q[1]), False)   # traced, refused
    decoy = (300, 'certificate_next_cycle', trajectory(q[6], q[7]), True)          # the next cycle's identical home
    chain = tracking.stage_chain(sorted([*certs, earlier, refused, decoy]), [q[6], q[7]], 90, 200)
    assert [chain[s][0] for s in tracking.ARM_STAGES] == [f'certificate_{k}' for k in range(7)]
    twin = (105, 'certificate_other_path', trajectory(q[0], [0.5] * 6, q[1]), True)   # same ends, another certified path
    try:
        tracking.stage_chain(sorted([*certs, twin]), [q[6], q[7]], 90, 200)
    except ValueError as exc:
        assert 'ambiguous APPROACH' in str(exc)
    else:
        raise AssertionError('an ambiguous stage match was accepted')
    try:
        tracking.stage_chain(certs[1:], [q[6], q[7]], 90, 200)               # the approach was never certified
    except ValueError as exc:
        assert 'APPROACH' in str(exc)
    else:
        raise AssertionError('a broken chain was accepted')


def test_goal_windows_are_the_arm_controllers_accepted_and_reached_times():
    log = '\n'.join(['[x] [INFO] [10.000000000] [ur5_arm_controller]: Accepted new action goal',
                     '[x] [INFO] [12.500000000] [ur5_arm_controller]: Goal reached, success!',
                     '[x] [INFO] [13.000000000] [ur5_gripper_controller]: Accepted new action goal',
                     '[x] [INFO] [20.000000000] [ur5_arm_controller]: Accepted new action goal'])
    assert tracking.goal_windows(log, 9_000_000_000, 15_000_000_000) == [(10_000_000_000, 12_500_000_000)]
    assert tracking.goal_windows(log, 9_000_000_000, 21_000_000_000)[1] == (20_000_000_000, None)


def test_the_measurement_stream_is_read_in_both_json_spellings(tmp_path):
    stream = tmp_path / 'summary.measurements.jsonl'
    lines = []
    for k in range(20):                                       # the executor writes both "wall_ns": 1 and "wall_ns":1
        m = dict(run_id='r', sim_ns=k, wall_ns=1000 + k, poses={})
        text = json.dumps(dict(phase='p', measurement=m))
        lines.append(text if k % 2 else text.replace('"wall_ns": ', '"wall_ns":'))
    stream.write_text('\n'.join(lines) + '\n')
    stream.write_text(stream.read_text() + '{"phase": "p", "measurement": {"truncated\n')
    found, unreadable = tracking.measurement_windows(stream, [(1000, 1009), (1010, 1019)])
    assert [m['wall_ns'] for m in found[0]] == list(range(1000, 1010)) and len(found[1]) == 10 and unreadable == 1


def test_a_stream_that_does_not_cover_a_goal_is_a_problem():
    ms = 1_000_000
    goal = trajectory([0.0] * 6, [1.0] * 6, step=1.0)                          # a 1 s certified trajectory
    samples = [dict(wall_ns=1000 * ms + k * ms, sim_ns=k * ms) for k in range(1001)]
    assert tracking.coverage_problem(samples, (1000 * ms, 2000 * ms), goal) is None
    assert 'cover' in tracking.coverage_problem(samples[100:], (1000 * ms, 2000 * ms), goal)     # starts 100 ms late
    holed = samples[:400] + samples[450:]
    assert 'gap' in tracking.coverage_problem(holed, (1000 * ms, 2000 * ms), goal)              # a 50 ms hole
    short = [dict(m, sim_ns=m['sim_ns'] // 2) for m in samples]                                 # half the simulated time
    assert 'span' in tracking.coverage_problem(short, (1000 * ms, 2000 * ms), goal)


def test_the_quintic_segment_meets_both_waypoints():
    a, b = trajectory([0.0] * 6, [1.0] * 6, step=2.0).points
    assert tracking.quintic(a, b, 0.0) == [0.0] * 6
    assert all(abs(v - 1.0) < 1e-12 for v in tracking.quintic(a, b, 1.0))
    assert all(abs(v - 0.5) < 1e-12 for v in tracking.quintic(a, b, 0.5))     # rest-to-rest: symmetric midpoint


def test_only_the_free_space_stages_have_an_allowance():
    allowance = tracking.tracking_allowances()
    assert allowance == {'APPROACH': 0.015, 'TRANSFER': 0.015, 'HOME': 0.015, 'RETREAT': 0.001}


def test_a_sample_of_another_measurement_run_never_counts(tmp_path):
    stream = tmp_path / 'summary.measurements.jsonl'
    stream.write_text('\n'.join(json.dumps(dict(phase='p', measurement=dict(run_id=run, sim_ns=k, wall_ns=k)))
                                for k, run in enumerate(['mine', 'other', 'mine'])) + '\n')
    found, unreadable = tracking.measurement_windows(stream, [(0, 10)], 'mine')
    assert [m['run_id'] for m in found[0]] == ['mine', 'mine'] and unreadable == 1


def test_the_corners_are_the_receipt_bound_simulated_boxes():
    import math
    box = dict(box_size_m=[0.02, 0.04, 0.01], box_centre_m=[0.01, -0.02, 0.0], pose=[0.01, -0.02, 0.0, 0.0, 0.0, math.pi / 2])
    receipt = dict(simulator_collision_primitives=dict(links=[dict(link=link, boxes=[box]) for link in tracking.FINGERTIPS]))
    corners = tracking.fingertip_corners(receipt)['gripper_finger1_finger_tip_link']
    centre = [sum(c[i] for c in corners) / 8 for i in range(3)]
    assert max(abs(a - b) for a, b in zip(centre, box['box_centre_m'])) < 1e-12
    xs, ys = [c[0] for c in corners], [c[1] for c in corners]                 # yawed a quarter turn: x/y extents swap
    assert abs((max(xs) - min(xs)) - 0.04) < 1e-12 and abs((max(ys) - min(ys)) - 0.02) < 1e-12
    try:
        tracking.fingertip_corners(dict(simulator_collision_primitives=dict(links=[])))
    except ValueError:
        pass
    else:
        raise AssertionError('a receipt without simulated fingertip boxes was accepted')


def test_a_twin_with_the_same_waypoints_but_other_timing_is_ambiguous():
    q = [[float(i)] * 6 for i in range(9)]
    certs = [(100 + 10 * k, f'certificate_{k}', trajectory(q[k], q[k + 1]), True) for k in range(7)]
    slow = (105, 'certificate_slow_approach', trajectory(q[0], q[1], step=3.0), True)     # same points, slower
    try:
        tracking.stage_chain(sorted([*certs, slow]), [q[6], q[7]], 90, 200)
    except ValueError as exc:
        assert 'ambiguous APPROACH' in str(exc)
    else:
        raise AssertionError('a twin with other timing was chosen silently')


def test_the_judge_cli_gives_no_verdict_from_another_g4_control_version(monkeypatch, capsys, tmp_path):
    import g4_qualification
    monkeypatch.setattr(g4_qualification.scope, 'planner_identity', lambda *a, **k: 'p' * 64)
    monkeypatch.setattr(g4_qualification.scope, 'g4_control_identity', lambda *a, **k: 'b' * 64)
    for planner, control in (('p' * 64, 'a' * 64), ('q' * 64, 'b' * 64)):      # another control, another planner
        monkeypatch.setattr(g4_qualification.scope, 'g4_session_evidence', lambda run, tracking, p=planner, c=control:
                            dict(kind='STRICT_PASS', planner=p, control=c, problems=[]))
        assert g4_qualification.main(['judge', 'session', str(tmp_path)]) == 1
        assert 'not its verifier' in capsys.readouterr().out
    monkeypatch.setattr(g4_qualification.scope, 'g4_session_evidence',
                        lambda run, tracking: dict(kind='STRICT_PASS', planner='p' * 64, control='b' * 64, problems=[]))
    assert g4_qualification.main(['judge', 'session', str(tmp_path)]) == 0      # its own versions: a verdict
