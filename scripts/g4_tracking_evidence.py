#!/usr/bin/env python3
"""G4 numeric tracking evidence, offline and read-only: the Cartesian deviation of the physical fingertip boxes (Gazebo
link poses from the executor's own measurement stream) from the certified controller spline of every executed arm stage.

This is the metric behind the physics-pad tracking allowances (simulation_collision.PHYSICS_PAD_ALLOWANCE_M, measured over
five G3C trials in log/qualification/g3c/clearance_fix): path-space (nearest point of the densely sampled JTC quintic
spline), maximum over every fingertip box corner. The free-space stages are certified against pads inflated by that
allowance, so each must stay within it; the contact stages (grasp, lift, place) are bound by the contact guard and strict
retention and are reported, not bounded here.

Each executed stage's trajectory is the motion session's own certificate (result 0), recovered by chaining exact
start/end states backwards from the executed EXECUTE_HOME goal (the executor runs exactly the certified trajectories);
an ambiguous or uncertified match, or a point count other than the executor's own plan, is a problem. The planned
corners use the certificate's own gripper state and the receipt-bound simulated fingertip boxes (the geometry the pads
were built from). Every input is bound to the trial record: the receipt to its session digest, each cycle's URDF to the
bound robot description, every measurement to the executor's own measurement run. Every measured sample is used, and its coverage of each controller goal is checked (start, end, gaps,
simulated span), so a gap in the stream cannot hide a deviation. Forward kinematics of the cycle's qualification URDF is
validated against Gazebo's measured fingertip poses at both ends of every stage before any deviation is trusted.
usage: g4_tracking_evidence.py <run dir> [trace dir]   (with ROS 2 sourced)"""
from __future__ import annotations

import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

ARM_STAGES = ('APPROACH', 'GRASP', 'LIFT', 'TRANSFER', 'PLACE', 'RETREAT', 'HOME')   # arm controller goal order
FINGERTIPS = ('gripper_finger1_finger_tip_link', 'gripper_finger2_finger_tip_link')
ROBOT = 'a0::workcell_robot::'
SAME_STATE_RAD = 1e-6              # chained certificates: exact start/end joint states
FK_VALIDATION_M = 5e-4             # model check: FK of the measured joints against Gazebo's measured fingertip pose
DENSE_PER_SEGMENT = 20             # the G3C clearance_fix sampling of the controller spline
# Stream coverage of a controller goal (the executor samples every simulated millisecond): the first sample within
# 20 ms of the goal's acceptance, the last within 20 ms of it being reached, no simulated gap above 10 ms, and a
# simulated span covering the certified duration less 50 ms.
COVERAGE_EDGE_NS, COVERAGE_GAP_NS, COVERAGE_SPAN_SLACK_NS = 20_000_000, 10_000_000, 50_000_000
GOAL_LINE = r'\[(\d+\.\d+)\] \[ur5_arm_controller\]: '


def tracking_allowances():
    """{stage: allowance_m} for the free-space stages (the planning pads' tracking allowance)."""
    from simulation_collision import PHYSICS_PAD_ALLOWANCE_M
    return {name.replace('PREPLAN_', ''): value for name, value in PHYSICS_PAD_ALLOWANCE_M.items()}


def fingertip_corners(receipt):
    """{fingertip link: [box corner xyz in the link frame]} of every collision box Gazebo simulates on the fingertips:
    the receipt-bound simulator_collision_primitives (centre and roll-pitch-yaw in the link frame)."""
    from perceived_object_grasp_plan import quaternion_from_rpy
    from simulator_execution import compose_pose
    links = {entry['link']: entry for entry in (receipt.get('simulator_collision_primitives') or {}).get('links') or []}
    corners = {}
    for link in FINGERTIPS:
        boxes = (links.get(link) or {}).get('boxes') or []
        if not boxes:
            raise ValueError(f'no receipt-bound simulated collision box on {link}')
        corners[link] = [compose_pose(list(b['pose'][:3]) + quaternion_from_rpy(b['pose'][3:6]),
                                      [sx * b['box_size_m'][0] / 2, sy * b['box_size_m'][1] / 2, sz * b['box_size_m'][2] / 2,
                                       0, 0, 0, 1])[:3]
                         for b in boxes for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    return corners


class Kinematics:
    """Forward kinematics of a URDF (revolute/continuous/prismatic/fixed, mimic joints) in the world frame."""
    def __init__(self, urdf):
        from perceived_object_grasp_plan import quaternion_from_rpy
        from simulator_execution import compose_pose
        self.compose, self.rpy = compose_pose, quaternion_from_rpy
        self.joints = {j.find('child').get('link'): j for j in ET.fromstring(urdf).findall('joint')}

    def pose(self, link, q):
        chain = []
        while link in self.joints:
            chain.append(self.joints[link])
            link = self.joints[link].find('parent').get('link')
        pose = [0, 0, 0, 0, 0, 0, 1]
        for joint in reversed(chain):
            origin = joint.find('origin')
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            pose = self.compose(pose, xyz + self.rpy(rpy))
            if joint.get('type') in ('revolute', 'continuous', 'prismatic'):
                axis = [float(v) for v in joint.find('axis').get('xyz').split()]
                value = q.get(joint.get('name'), 0.)
                mimic = joint.find('mimic')
                if mimic is not None:
                    value = q.get(mimic.get('joint'), 0.) * float(mimic.get('multiplier', '1')) + float(mimic.get('offset', '0'))
                if joint.get('type') == 'prismatic':
                    pose = self.compose(pose, [a * value for a in axis] + [0, 0, 0, 1])
                else:
                    n, s = math.sqrt(sum(a * a for a in axis)), math.sin(value / 2)
                    pose = self.compose(pose, [0, 0, 0, axis[0] / n * s, axis[1] / n * s, axis[2] / n * s, math.cos(value / 2)])
        return pose


def quintic(a, b, s):
    """The JTC segment between trajectory points a and b at fraction s (Humble trajectory.cpp, the certificate's model)."""
    duration = (b.time_from_start.sec + b.time_from_start.nanosec * 1e-9) - (a.time_from_start.sec + a.time_from_start.nanosec * 1e-9)
    out, t = [], s * duration
    for j in range(len(a.positions)):
        p0, p1, u, w = a.positions[j], b.positions[j], a.velocities[j], b.velocities[j]
        x, y = a.accelerations[j], b.accelerations[j]
        T = duration
        c = [p0, u, x / 2, (-20 * p0 + 20 * p1 - 3 * x * T * T + y * T * T - 12 * u * T - 8 * w * T) / (2 * T ** 3),
             (30 * p0 - 30 * p1 + (3 * x - 2 * y) * T * T + (16 * u + 14 * w) * T) / (2 * T ** 4),
             (-12 * p0 + 12 * p1 - (x - y) * T * T - 6 * (u + w) * T) / (2 * T ** 5)]
        out.append(sum(c[k] * t ** k for k in range(6)))
    return out


def certificates(trace):
    """[(wall_ns, path stem, JointTrajectory, certified)] of every arm-only trajectory in the planning trace, in order;
    certified: its result file says result 0 (the adapter also traces the trajectories it refuses)."""
    from moveit_msgs.msg import RobotTrajectory
    from rclpy.serialization import deserialize_message
    found = []
    for path in Path(trace).glob('certificate_*.trajectory.cdr'):
        trajectory = deserialize_message(path.read_bytes(), RobotTrajectory).joint_trajectory
        if len(trajectory.joint_names) == 6:
            stem = str(path)[:-len('.trajectory.cdr')]
            result = Path(stem + '.result.txt')
            certified = result.is_file() and 'result 0' in result.read_text().splitlines()
            found.append((int(path.name.split('_')[1]), stem, trajectory, certified))
    return sorted(found, key=lambda c: c[0])


def certified_joint_state(stem):
    """{joint: position} of the robot state the certificate's planning scene carries (its gripper state included)."""
    from moveit_msgs.msg import PlanningScene
    from rclpy.serialization import deserialize_message
    state = deserialize_message(Path(stem + '.scene.cdr').read_bytes(), PlanningScene).robot_state.joint_state
    return dict(zip(state.name, state.position))


def stage_chain(certs, home, begin_ns, end_ns):
    """{stage: (stem, trajectory)} for one cycle: its executed home goal, then each earlier arm stage whose end state is
    exactly the next one's start, all certified (result 0) inside the cycle's own executor window. Every certified
    candidate with the chosen one's start and end states must be identical to it (otherwise the match is ambiguous)."""
    same = lambda a, b: len(a) == len(b) and max(abs(x - y) for x, y in zip(a, b)) <= SAME_STATE_RAD
    pts = lambda t: [list(p.positions) for p in t.points]
    full = lambda t: [list(p.positions) + list(p.velocities) + list(p.accelerations) +
                      [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9] for p in t.points]
    window = [c for c in certs if begin_ns <= c[0] <= end_ns and c[3]]
    homes = [c for c in window if len(c[2].points) == len(home) and all(same(a, b) for a, b in zip(pts(c[2]), home))]
    if not homes:
        raise ValueError('the executed home goal is not among the cycle\'s certified trajectories')
    chain = {'HOME': homes[-1]}
    for stage in reversed(ARM_STAGES[:-1]):
        later = chain[ARM_STAGES[ARM_STAGES.index(stage) + 1]]
        start = pts(later[2])[0]
        earlier = [c for c in window if c[0] < later[0] and same(pts(c[2])[-1], start)]
        if not earlier:
            raise ValueError(f'no certified {stage} trajectory ends where {ARM_STAGES[ARM_STAGES.index(stage) + 1]} starts')
        chosen = earlier[-1]
        twins = [c for c in earlier if same(pts(c[2])[0], pts(chosen[2])[0])]
        if any(len(c[2].points) != len(chosen[2].points) or not all(same(a, b) for a, b in zip(full(c[2]), full(chosen[2])))
               for c in twins):              # positions, velocities, accelerations and timing: the whole controller input
            raise ValueError(f'ambiguous {stage}: certified trajectories with its start and end states differ')
        chain[stage] = chosen
    return {stage: (c[1], c[2]) for stage, c in chain.items()}


def goal_windows(launch_log, begin_ns, end_ns):
    """[(accepted_ns, reached_ns)] of the arm controller's goals inside one cycle's executor window, in order."""
    accepted = [int(float(t) * 1e9) for t in re.findall(GOAL_LINE + 'Accepted new action goal', launch_log)]
    reached = [int(float(t) * 1e9) for t in re.findall(GOAL_LINE + 'Goal reached', launch_log)]
    out = []
    for a in (t for t in accepted if begin_ns <= t <= end_ns):
        done = [r for r in reached if r >= a]
        out.append((a, done[0] if done else None))
    return out


def measurement_windows(stream, windows, run_id=None):
    """({window index: [measurement]} of every executor measurement inside each (begin, end) window, count of stream
    lines without a readable wall time or of another measurement run than ``run_id``)."""
    out, unreadable = {i: [] for i in range(len(windows))}, 0
    wall = re.compile(r'"wall_ns": ?(\d+)')
    with open(stream) as lines:
        for line in lines:
            if not line.strip():
                continue
            match = wall.search(line, 0, 600)
            if not match:
                unreadable += 1
                continue
            t = int(match.group(1))
            if run_id is not None and f'"run_id": "{run_id}"' not in line[:200] and f'"run_id":"{run_id}"' not in line[:200]:
                unreadable += 1                                   # a sample of another measurement run never counts
                continue
            for i, (a, b) in enumerate(windows):
                if a <= t <= (b if b is not None else a + 120_000_000_000):
                    out[i].append(json.loads(line)['measurement'])
    return out, unreadable


def coverage_problem(samples, window, trajectory):
    """Why the samples do not cover a controller goal completely (None when they do)."""
    accepted, reached = window
    duration = trajectory.points[-1].time_from_start.sec * 10 ** 9 + trajectory.points[-1].time_from_start.nanosec
    sim = [m['sim_ns'] for m in samples]
    gap = max((b - a for a, b in zip(sim, sim[1:])), default=None)
    if samples[0]['wall_ns'] - accepted > COVERAGE_EDGE_NS or reached is None or reached - samples[-1]['wall_ns'] > COVERAGE_EDGE_NS:
        return 'the measurement stream does not cover the goal from acceptance to completion'
    if gap is None or gap > COVERAGE_GAP_NS:
        return f'the measurement stream has a {gap / 1e6 if gap else 0:.1f} ms simulated gap'
    if sim[-1] - sim[0] < duration - COVERAGE_SPAN_SLACK_NS:
        return f'the measured simulated span {(sim[-1] - sim[0]) / 1e9:.3f} s is shorter than the certified {duration / 1e9:.3f} s'
    return None


def stage_deviation(kinematics, corners, trajectory, samples, certified_state):
    """(max deviation m, at s, deviation at start m, FK validation error m, robot-contact samples) of one stage. The
    planned corners carry the certificate's own non-arm joint state (the certified gripper), not the measured one."""
    import numpy as np
    names = list(trajectory.joint_names)
    joints = lambda m: {k.split('::')[-1]: v[0] for k, v in m['joints'].items() if k.startswith(ROBOT)}
    fk_error = max(math.dist(kinematics.pose(link, joints(m))[:3], m['poses'][ROBOT + link][:3])
                   for m in (samples[0], samples[-1]) for link in corners)
    grip = {n: v for n, v in certified_state.items() if n not in names}
    dense = [quintic(a, b, k / DENSE_PER_SEGMENT) for a, b in zip(trajectory.points, trajectory.points[1:])
             for k in range(DENSE_PER_SEGMENT)] + [list(trajectory.points[-1].positions)]
    corner_list = [(link, c) for link in corners for c in corners[link]]
    planned = np.array([[kinematics.compose(kinematics.pose(link, dict(grip, **dict(zip(names, q)))), c + [0, 0, 0, 1])[:3]
                         for link, c in corner_list] for q in dense])                    # (dense, corners, 3)
    measured = np.array([[kinematics.compose(m['poses'][ROBOT + link], c + [0, 0, 0, 1])[:3] for link, c in corner_list]
                         for m in samples])                                               # (samples, corners, 3)
    deviation = np.empty(len(measured))
    for i in range(0, len(measured), 64):
        chunk = measured[i:i + 64, None, :, :] - planned[None, :, :, :]                   # (chunk, dense, corners, 3)
        deviation[i:i + 64] = np.linalg.norm(chunk, axis=3).max(axis=2).min(axis=1)
    worst = int(deviation.argmax())
    contacts = sum(1 for m in samples if any('workcell_robot' in c['a'] + c['b'] for c in m.get('contacts') or []))
    return (float(deviation[worst]), (samples[worst]['sim_ns'] - samples[0]['sim_ns']) / 1e9, float(deviation[0]),
            fk_error, contacts)


def cycle_tracking(cycle_dir, trace, launch_log, receipt, certs=None, robot_description_sha256=None):
    """Numeric tracking evidence of one executed cycle: every arm stage's deviation, each free stage against its
    allowance. {stages: {...}, problems: [...]}. ``robot_description_sha256``: the bound robot description the cycle's
    qualification URDF must be."""
    import hashlib
    cycle_dir = Path(cycle_dir)
    summary = json.loads((cycle_dir / 'execute/summary.json').read_text())
    urdf = (cycle_dir / 'execute/qualification.urdf').read_bytes()
    if robot_description_sha256 is not None and hashlib.sha256(urdf).hexdigest() != robot_description_sha256:
        return dict(stages={}, problems=['the qualification URDF is not the bound robot description'])
    run_id = (summary.get('pile_contact_certification') or {}).get('run_id')
    if not run_id or run_id != (summary.get('release_evidence') or {}).get('run_id'):
        return dict(stages={}, problems=['the executor\'s measurement run is not recorded consistently'])
    events = {e['stage']: int(e['timestamp'] * 1e9) for e in summary.get('stage_events') or []}
    goal = summary.get('owned_execution_goal') or {}
    if goal.get('stage') != 'EXECUTE_HOME' or 'EXECUTE_APPROACH' not in events or 'EXECUTE_HOME' not in events:
        return dict(stages={}, problems=['the executed approach/home stages are not recorded'])
    home = [p['positions'] for p in goal['trajectory']['joint_trajectory']['points']]
    problems, stages = [], {}
    try:   # certified while the executor revalidated, before its home stage began
        chain = stage_chain(certs if certs is not None else certificates(trace), home, min(events.values()),
                            events['EXECUTE_HOME'])
    except ValueError as exc:
        return dict(stages={}, problems=[str(exc)])
    # The cycle's own arm goals: from its approach until just after its home goal began (any other goal breaks the count).
    windows = goal_windows(launch_log, events['EXECUTE_APPROACH'] - 1_000_000_000, events['EXECUTE_HOME'] + 5_000_000_000)
    if len(windows) != len(ARM_STAGES) or any(b is None for _, b in windows):
        return dict(stages={}, problems=[f'{len(windows)} completed arm controller goals in the cycle, not {len(ARM_STAGES)}'])
    allowance = tracking_allowances()
    bound = (summary.get('physics_pads') or {}).get('allowance_m') or {}
    if ({k.replace('PREPLAN_', ''): v for k, v in bound.items()} != allowance or
            set(allowance) != {'APPROACH', 'TRANSFER', 'RETREAT', 'HOME'}):
        problems.append(f'the executor\'s pad allowances {bound} are not the tracking allowances {allowance}')
    planned = {m.get('stage'): m.get('points') for m in summary.get('plan_metadata') or []}
    kinematics = Kinematics(urdf.decode())
    corners = fingertip_corners(receipt)
    samples, unreadable = measurement_windows(cycle_dir / 'execute/summary.measurements.jsonl', windows, run_id)
    if unreadable:
        problems.append(f'{unreadable} measurement stream line(s) unreadable or of another measurement run')
    for i, stage in enumerate(ARM_STAGES):
        stem, trajectory = chain[stage]
        if planned.get('PREPLAN_' + stage) != len(trajectory.points):
            problems.append(f'{stage}: the certificate has {len(trajectory.points)} points, the executed plan '
                            f"{planned.get('PREPLAN_' + stage)}")
        if len(samples[i]) < 2:
            problems.append(f'{stage}: no measured samples in its controller goal window')
            continue
        gap = coverage_problem(samples[i], windows[i], trajectory)
        if gap:
            problems.append(f'{stage}: {gap}')
        worst, at, start, fk_error, contacts = stage_deviation(kinematics, corners, trajectory, samples[i],
                                                               certified_joint_state(stem))
        record = dict(certificate=Path(chain[stage][0]).name, goal_window_ns=list(windows[i]), samples=len(samples[i]),
                      max_deviation_m=worst, at_s=at, start_deviation_m=start, fk_validation_m=fk_error,
                      robot_contact_samples=contacts, allowance_m=allowance.get(stage))
        if fk_error > FK_VALIDATION_M:
            problems.append(f'{stage}: forward kinematics disagrees with Gazebo by {fk_error * 1000:.2f} mm')
        if stage in allowance:
            record['within_allowance'] = worst <= allowance[stage]
            if not record['within_allowance']:
                problems.append(f'{stage}: tracking deviation {worst * 1000:.2f} mm exceeds its {allowance[stage] * 1000:.0f} '
                                'mm allowance')
        stages[stage] = record
    return dict(stages=stages, problems=problems)


def session_tracking(run_dir, trace=None):
    """{cycles: {n: cycle_tracking}, problems} for every executed cycle of a G4 run (trace default: '<run>.trace'). The
    receipt (fingertip geometry) must be the trial record's bound session receipt and each URDF its robot description."""
    import hashlib
    run = Path(run_dir)
    trace = Path(trace) if trace is not None else Path(str(run) + '.trace')
    session = run / 'session'
    launch_log = (session / 'launch.log').read_text(errors='replace')
    receipt_bytes = (session / 'runtime/receipt.json').read_bytes()
    receipt = json.loads(receipt_bytes)
    bound = json.loads((session / 'g4-session.json').read_text())['binding']
    out = dict(trace=str(trace), cycles={}, problems=[])
    if hashlib.sha256(receipt_bytes).hexdigest() != bound['session']['receipt_sha256']:
        out['problems'].append('the receipt is not the trial\'s bound session receipt')
        return out
    if not trace.is_dir():
        out['problems'].append(f'no planning trace at {trace}')
        return out
    certs = certificates(trace)
    for cycle_dir in sorted((session / 'cycles').iterdir()):
        if not (cycle_dir / 'execute/summary.measurements.jsonl').is_file():
            continue
        found = cycle_tracking(cycle_dir, trace, launch_log, receipt, certs, bound['robot']['description_sha256'])
        out['cycles'][int(cycle_dir.name)] = found
        out['problems'] += [f'cycle {int(cycle_dir.name)}: {p}' for p in found['problems']]
    return out


if __name__ == '__main__':
    result = session_tracking(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
    for n, cycle in result['cycles'].items():
        for stage, r in cycle['stages'].items():
            bound = f" / allowance {r['allowance_m'] * 1000:.0f} mm" if r['allowance_m'] else ' (contact stage)'
            print(f"cycle {n} {stage:8s} max {r['max_deviation_m'] * 1000:6.2f} mm at {r['at_s']:5.2f} s (start "
                  f"{r['start_deviation_m'] * 1000:.2f}){bound}; FK {r['fk_validation_m'] * 1000:.3f} mm; "
                  f"{r['samples']} samples; robot contacts {r['robot_contact_samples']}")
    print('G4 TRACKING:', 'PASS' if not result['problems'] and result['cycles'] else f"FAIL {result['problems']}")
