#!/usr/bin/env python3
"""G4: one continuous physical sorting session of a Workcell Builder cell in ONE persistent Gazebo world.

Every cycle runs, in order:
  Observe    measure every object in Gazebo (simulator_observations) and prove the session's verified world unchanged;
  Classify   render that world with the cell camera and classify it with EPD (perception_replay session-frame), then
             bind EPD's classes to the measured objects (simulator_observations --perception);
  Select     the sorting ledger (sorting_session, backend 'simulator') picks the next routed object; unclassified,
             unrouted, completed and removed objects are never targets and stay obstacles;
  Plan       the executor resolves that one target (plan-only, --target-object-id) with the physics pads, every other
             object and both physical bins as obstacles;
  Validate   re-measure: the geometry the plan used is unchanged and the frame is fresh;
  Authorise  one per-cycle record under the G4 session (physics_scope_qualification.g4_authorise_cycle);
  Execute    the executor's guarded full-cycle for that target, Verify (g4_verify_cycle) and Reconcile (the ledger at the
             measured final pose); then Observe again.

No G4 session can be authorised today (no G4 acceptance verifier exists), so Authorise refuses with
G4_SCOPE_PLAN_ONLY and the session stops there: zero motion. ``--plan-every-target`` keeps the same launch and plans
each further routed object once (Observe again first); a plan-only cycle is never a ledger attempt or a count.
The executor admits no G4 cycle record yet either (Execute would refuse G4_EXECUTOR_ADMISSION_MISSING).

Stop at cycle end: SIGUSR1 (the G5B latch). Cancel: SIGTERM/SIGINT (forwarded to the running child, G5A).
Pause/stop requests in the control file are honoured at cycle boundaries (sorting_session.poll_control).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

import run_stage_a1_finish as stage_a  # noqa: E402  (launch, overlays, executor command, stop/cancel)
from run_stage_a1_finish import (STOP_AFTER_CYCLE, OperatorCancel, StageForwarder, cancel_on_signal, run,  # noqa: E402
                                 stream)

REPORT_NAME = 'g4-sorting-report.json'
# A resting object re-measured later must be where the session's world says (the existing refresh tolerance).
WORLD_TOLERANCE_M, WORLD_TOLERANCE_RAD = 1e-5, 1e-5
# The render step picks ROS domains 170..219 (perception_replay.record); the physics launch stays outside them.
RENDER_DOMAINS = range(170, 220)


class G4Stop(Exception):
    """A G4 stop condition: the session ends at this boundary with ``code``."""
    def __init__(self, code, reason):
        super().__init__(f'{code}: {reason}')
        self.code, self.reason = code, reason


def measured_world(snapshot):
    """A measured detected_objects/v1 snapshot as a session world [{object_id, pose: {frame_id, xyz, rpy}, dimensions}]."""
    return [{'object_id': o['object_id'], 'pose': {'frame_id': 'world', 'xyz': [float(v) for v in o['pose']['xyz']],
                                                   'rpy': [float(v) for v in o['pose']['rpy']]},
             'dimensions': [float(v) for v in o['dimensions']]}
            for o in sorted(snapshot['objects'], key=lambda o: o['object_id'])]


def world_difference(expected, measured):
    """(max position delta m, max orientation delta rad) between two worlds of the same objects; raises if they differ."""
    a = {o['object_id']: o for o in expected}
    b = {o['object_id']: o for o in measured}
    if set(a) != set(b):
        raise G4Stop('G4_WORLD_CHANGED', f'objects {sorted(a)} became {sorted(b)}')
    position = orientation = 0.0
    for oid, item in a.items():
        other = b[oid]
        if [round(v, 9) for v in item['dimensions']] != [round(v, 9) for v in other['dimensions']]:
            raise G4Stop('G4_WORLD_CHANGED', f'{oid} changed size')
        position = max(position, math.dist(item['pose']['xyz'], other['pose']['xyz']))
        orientation = max(orientation, *(abs(math.remainder(x - y, math.tau))
                                         for x, y in zip(item['pose']['rpy'], other['pose']['rpy'])))
    return position, orientation


def require_same_world(expected, measured, what):
    position, orientation = world_difference(expected, measured)
    if position > WORLD_TOLERANCE_M or orientation > WORLD_TOLERANCE_RAD:
        raise G4Stop('G4_WORLD_CHANGED', f'{what}: the measured world moved {position:.3g} m / {orientation:.3g} rad from '
                     "the session's verified world; an unexplained change is never planned against")
    return dict(max_position_delta_m=position, max_orientation_delta_rad=orientation)


def frame_record(frame_path):
    """{sha256, world_revision, age_s} of a rendered frame, for the per-cycle authorisation."""
    raw = Path(frame_path).read_bytes()
    source = yaml.safe_load(raw)['source']
    return dict(sha256=hashlib.sha256(raw).hexdigest(), world_revision=source['world_revision'],
                age_s=(time.time_ns() - int(source['capture_wall_ns'])) / 1e9)


def check_plan(summary, target, observations):
    """The plan-only resolve is the target's: the right object, its route and class, nothing executed."""
    stage_a.assert_plan(summary, require_resolved=True)
    problems = []
    if summary.get('selected_object_id') != target['id'] or summary.get('batch_target_object_id') != target['id']:
        problems.append(f"planned {summary.get('selected_object_id')!r}, not the target {target['id']!r}")
    route = ((summary.get('task_intent_resolution') or {}).get('place_resolution') or {}).get('route') or {}
    if route.get('class_id') != target['class_id'] or route.get('region_ref') != target['destination']:
        problems.append(f"resolved route {route.get('class_id')!r} -> {route.get('region_ref')!r}, not "
                        f"{target['class_id']!r} -> {target['destination']!r}")
    bound = {o['object_id']: o['class_id'] for o in observations['objects']}
    if bound.get(target['id'].split('::', 1)[-1]) != target['class_id']:
        problems.append('the measured object bound to the frame does not carry the target class')
    if summary.get('execution_attempted') is not False or not summary.get('physics_pads'):
        problems.append('not a padded plan-only resolve')
    if problems:
        raise G4Stop('G4_PLAN_MISMATCH', '; '.join(problems))
    grasp = (summary.get('selected_extraction_intent') or {}).get('candidate_id')
    import physics_scope_qualification as scope
    return dict(route=route, physics_pads=summary['physics_pads'], grasp=grasp,
                grasp_qualified=str(grasp).split('::', 1)[0] in scope.G4_QUALIFIED_GRASP_STRATEGIES,
                stages=[p.get('stage') for p in summary.get('plan_metadata', [])],
                destination=(summary.get('physical_destination_walls') or {}).get('selected_destination'))


def plan_refused(ledger, summary_path, returncode, evidence_dir):
    """A target's plan-only resolve failed: nothing moved, so the ledger records the attempt by its own policy
    (deterministic refusals and exhausted attempts reject the object, which stays an obstacle) and the session observes
    again. Missing evidence, or evidence of motion, faults the session (CYCLE_EVIDENCE_MISSING / G4_PLAN_MOVED)."""
    try:
        summary = json.loads(Path(summary_path).read_text())
    except (OSError, ValueError):
        summary = None
    if isinstance(summary, dict) and summary.get('execution_attempted') is not False:
        ledger.active = None
        raise G4Stop('G4_PLAN_MOVED', 'a plan-only resolve reports execution; the world is unverified')
    return ledger.record_cycle(summary, returncode, [], evidence_dir)


def verify_and_reconcile(ledger, cycle_path, cycle_sha256, summary, returncode, evidence_dir):
    """Verify an executed cycle against its own authorisation, then let the ledger count and reconcile it at the
    measured final pose. Returns (verification, ledger outcome). A cycle that moved without proof faults the session
    before the ledger sees it, so an unverified cycle is never counted."""
    import physics_scope_qualification as scope
    target = ledger.active
    verification = scope.g4_verify_cycle(cycle_path, cycle_sha256, summary if isinstance(summary, dict) else {},
                                         ledger.physical_destinations.get(target['destination']),
                                         {'dimensions_m': ledger.part_dimensions.get(target['id'])})
    if isinstance(summary, dict) and summary.get('execution_attempted') is True and not verification['verified']:
        ledger.active = None
        raise G4Stop('G4_CYCLE_UNVERIFIED', f"{target['id']}: " + '; '.join(verification['problems']))
    return verification, ledger.record_cycle(summary, returncode, [], evidence_dir)


def physical_destinations(intent, environment, receipt):
    """{route region: its receipt-bound physics floor} for every enabled route of the saved task."""
    from task_intent_resolver import class_routes
    bindings = {b['source_item_id']: b for b in receipt['destination_geometry_bindings']}
    out = {}
    for route in class_routes(intent, environment).values():
        if route['asset_ref'] not in bindings:
            raise G4Stop('G4_DESTINATION_UNBOUND', f"route {route['rule_id']} -> {route['asset_ref']} has no physics bin")
        out[route['region_ref']] = bindings[route['asset_ref']]
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--scene-package', type=Path, required=True, help='the Workcell Builder cell (saved and generated)')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=226, help='ROS domain of the physics launch (not 170..219)')
    parser.add_argument('--partition-prefix', default='g4_sorting')
    parser.add_argument('--max-cycles', type=int, help='session bound (default: every object)')
    parser.add_argument('--plan-every-target', action='store_true',
                        help='plan-only: after a refused authorisation, plan each further routed object once')
    parser.add_argument('--segment-planning-time', type=float, default=3.0)
    parser.add_argument('--search-timeout', type=float, default=300.0)
    parser.add_argument('--control-file', type=Path, help='{"commands": [{"seq": n, "command": pause|resume|stop}]}')
    parser.add_argument('--stream-status', action='store_true')
    args = parser.parse_args(argv)
    if not 1 <= args.domain <= 232 or args.domain in RENDER_DOMAINS:
        parser.error('--domain must be 1..232 and outside the render domains 170..219')
    if not 0 < args.segment_planning_time <= 10 or not 0 < args.search_timeout < math.inf:
        parser.error('invalid planning budget')
    repo, workspace = SCRIPTS.parent, SCRIPTS.parents[2]
    args.output = args.output.resolve()
    if args.output.exists():
        parser.error('choose a new output directory')
    args.output.mkdir(parents=True)
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, cancel_on_signal)
    STOP_AFTER_CYCLE.__init__()
    STOP_AFTER_CYCLE.args = args
    signal.signal(signal.SIGUSR1, STOP_AFTER_CYCLE.request)
    started = time.monotonic()
    report = dict(schema='workcell_g4_sorting_run/v1', status='BLOCKED', scope='G4_PLAN_ONLY_NO_MOTION',
                  started_wall_ns=time.time_ns(), argv=list(sys.argv[1:] if argv is None else argv), cycles=[])
    try:
        report['result'] = session(args, repo, workspace, report)
        report['status'] = 'STOPPED' if STOP_AFTER_CYCLE.record is not None else 'PASS'
        return 4 if report['status'] == 'STOPPED' else 0
    except G4Stop as stop:
        report.update(status='STOPPED', stop=dict(code=stop.code, reason=stop.reason))
        return 4
    except Exception as exc:
        report.update(status='BLOCKED', failure=str(exc))
        return 1
    except OperatorCancel as exc:
        report.update(status='CANCELLED', failure=f'cancelled by the operator ({exc})',
                      operator_cancel=dict(signal=str(exc), child=getattr(exc, 'child', None)))
        return 3
    finally:
        state = report.pop('_state', {})
        launch, launch_log, ledger = state.get('launch'), state.get('launch_log'), state.get('ledger')
        if ledger is not None:
            if report['status'] in ('BLOCKED', 'CANCELLED') and ledger.state not in ('STOPPED', 'FAULT'):
                ledger.fail('SESSION_INTERRUPTED' if report['status'] == 'CANCELLED' else 'SESSION_BLOCKED',
                            report.get('failure', ''))
            report['ledger'] = ledger.report()
            report['counts'] = ledger.counts()
        if launch is not None:
            stream(args, 'SIMULATOR_SHUTDOWN')
            report['shutdown'] = stage_a.stop_owned(launch, args.output / 'session/launch.log')
            if not report['shutdown']['clean']:
                report.update(status='BLOCKED', shutdown_failure='owned shutdown failed')
        if launch_log is not None:
            launch_log.close()
        if STOP_AFTER_CYCLE.record is not None:
            report['stop_after_cycle'] = STOP_AFTER_CYCLE.record
        report.update(finished_wall_ns=time.time_ns(), duration_s=round(time.monotonic() - started, 1))
        (args.output / REPORT_NAME).write_text(json.dumps(report, indent=2, default=str) + '\n')
        print(json.dumps({key: report.get(key) for key in ('status', 'result', 'stop', 'failure', 'counts')}, default=str))


def session(args, repo, workspace, report):
    """Run the session; returns its result. The launch, its log and the ledger are kept in report['_state']."""
    import physics_scope_qualification as scope
    from perception_replay import _perception_runtime
    from runtime_pick_inputs import normalize, rendered_frame, world_revision
    from simulator_world import TEMPLATE, derive_world, engine_equivalent
    from sorting_session import SortingFault, SortingSession, write_json_atomic
    from task_intent_v2 import OBSERVATION_PRODUCER_CEILING_S, class_routing_rules, freshness_source_type
    import perceived_object_grasp_plan as geometry
    state = report.setdefault('_state', {})
    out = args.output

    # --- the cell, its derived two-bin world and this checkout's qualified runtime --------------------------------
    cell = stage_a.authored_cell(args.scene_package)
    intent = yaml.safe_load((Path(cell['package']) / 'config/workcell_builder_task_intent.yaml').read_text())
    if class_routing_rules(intent) is None or freshness_source_type(intent) != 'rendered_perception_in_simulation':
        raise G4Stop('G4_CELL_UNSUPPORTED', 'a G4 session sorts a class-routed task observed through the simulated RGB-D '
                     'camera in the physics simulator (Task Editor: perception live EPD, simulated camera, routing)')
    if stage_a.git_blob(TEMPLATE, repo) != stage_a.SOURCE_WORLDS['sparse'][1]:
        raise RuntimeError('qualified world template changed')
    world_text, world_report = derive_world(cell['package'], physical_destinations='all')
    world_report['engine_equivalent_to_qualified_template'] = engine_equivalent(world_text)
    (out / 'authored-world').mkdir()
    source_world = out / 'authored-world/world.sdf'
    source_world.write_text(world_text)
    (out / 'authored-world/derivation.json').write_text(json.dumps(world_report, indent=2) + '\n')
    report['cell'] = dict(cell, world_sha256=world_report['world_sha256'],
                          objects=[o['object_id'] for o in world_report['objects']],
                          destinations=[d['asset_ref'] for d in world_report['destinations']])
    stage_a.launch_checkout_guard(repo)
    from simulator_backend import active_bridge_overlay, bridge_overlay_identity, moveit_overlay_identity
    bridge = active_bridge_overlay()
    bridge_identity = bridge_overlay_identity(bridge)
    build = stage_a.build_commissioning(repo, workspace, out, dict(os.environ))
    moveit_identity = moveit_overlay_identity(build['moveit_overlay'])
    report['preflight'] = dict(
        repo_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip(),
        repo_dirty=bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=repo, text=True).strip()),
        commissioning_build=build, bridge_overlay=bridge, source_world_sha256=stage_a.sha256(source_world))

    # --- one persistent launch -------------------------------------------------------------------------------------
    root = out / 'session'
    root.mkdir()
    scene = stage_a.prepare_authored_scene(repo, Path(cell['package']), root / cell['name'])
    runtime = root / 'runtime'
    receipt_path = runtime / 'receipt.json'
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), IGN_PARTITION=f'{args.partition_prefix}_{args.domain}',
               ROS_LOCALHOST_ONLY='1', ROS2CLI_DISABLE_DAEMON='1', PYTHONUNBUFFERED='1')
    stage_a.assert_domain_free(env, repo)
    stage_a.generate_scene(scene)
    state['launch_log'] = (root / 'launch.log').open('w')
    state['launch'] = launch = subprocess.Popen(
        ['ros2', 'launch', '--noninteractive', str(repo / 'scenes/ur5_2f_test/launch/demo.launch.py'),
         'execution_backend:=simulator', 'simulator_commissioning:=true', f'simulator_world:={source_world}',
         f'simulator_output:={runtime}', 'use_sim_time:=true', 'launch_rviz:=false', 'use_fake_hardware:=true',
         'allow_trajectory_execution:=true', f'scene_package_dir:={scene}'],
        cwd=repo, env=env, stdout=state['launch_log'], stderr=subprocess.STDOUT, start_new_session=True)
    stream(args, 'SIMULATOR_LAUNCH')
    stage_a.wait_file(receipt_path, launch, 90, root / 'launch.log')
    report['controllers_ready'] = stage_a.wait_controllers_active(receipt_path, launch, env, repo, 60)
    report['moveit_overlay'] = stage_a.verify_session_moveit(args.domain, env['IGN_PARTITION'], moveit_identity)
    report['bridge_overlay'] = stage_a.verify_session_bridge(args.domain, env['IGN_PARTITION'], bridge_identity)
    stream(args, 'SIMULATOR_READY')
    receipt = json.loads(receipt_path.read_text())

    # --- the G4 session record (plan-only: no acceptance exists) -----------------------------------------------------
    objects = world_report['objects']
    max_cycles = args.max_cycles or len(objects)
    runtime_record = workspace / 'workcell_perception_runtime.json'
    _perception_runtime(workspace)
    binding = scope.g4_session_binding(
        cell_dir=scene, receipt_path=receipt_path,
        native=scope.native_identity(workspace / 'install/workcell_builder/share/workcell_builder/commission_execute_build.json'),
        runtime=scope.runtime_identity(workspace), max_cycles=max_cycles,
        perception=dict(producer='simulator_render', source_type='rendered_perception_in_simulation',
                        epd_runtime_sha256=stage_a.sha256(runtime_record),
                        max_age_s=OBSERVATION_PRODUCER_CEILING_S['simulator_render']))
    session_path = root / 'g4-session.json'
    session_sha = scope.g4_open_session(session_path, binding)
    report['g4_session'] = dict(path=str(session_path), sha256=session_sha, state=scope.G4_PLAN_ONLY,
                                planner=binding['planner'], max_cycles=max_cycles)

    # --- the ledger: physics backend, continuous rendered frames ----------------------------------------------------
    document = yaml.safe_load((scene / 'environment.yaml').read_text())
    environment = document.get('environment', document)
    ledger = state['ledger'] = SortingSession(
        intent, environment, session_id=out.name, mode='continuous', scene=str(scene), backend='simulator',
        physical_destinations=physical_destinations(intent, environment, receipt),
        part_dimensions={'runtime::' + o['object_id']: o['dimensions_m'] for o in objects})
    control = args.control_file or out / 'sorting_control.json'
    status_path = out / 'sorting_session.json'

    def emit(extra=None):
        write_json_atomic(status_path, ledger.report())
        if args.stream_status:
            print(json.dumps(dict(event='sorting', status=ledger.status(), **(extra or {})), default=str), flush=True)

    def measure(directory, classes):
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / 'observations.yaml'
        run([sys.executable, SCRIPTS / 'simulator_observations.py', '--receipt', receipt_path, '--output', path, *classes],
            env=env, cwd=repo, log=directory / 'observations.log', timeout=90)
        return path, yaml.safe_load(path.read_text())

    unclassified = ['--class-map', json.dumps({o['object_id']: 'unclassified' for o in objects})]
    planned, index, previous = set(), 0, None      # previous: the last verified cycle (chains the next authorisation)
    ledger.transition('READY', 'Gazebo + MoveIt launched; G4 session opened (plan-only)')
    ledger.transition('RUNNING', 'sorting started')
    emit()
    while True:
        # --- cycle boundary: operator pause/stop, the stop-after-cycle latch, the session bound -------------------
        for applied in ledger.poll_control(control, cycle_active=False):
            emit({'control': applied})
        ledger.boundary()
        if ledger.state == 'STOPPED' or STOP_AFTER_CYCLE.record is not None:
            if ledger.state != 'STOPPED':
                ledger.transition('STOPPING', 'stop after cycle (SIGUSR1)')
                ledger.boundary()
            emit()
            return 'STOPPED_BY_OPERATOR'
        if ledger.state == 'PAUSED':
            time.sleep(0.5)
            continue
        if index >= max_cycles:
            ledger.transition('STOPPING', f'G4_SESSION_LIMIT: {max_cycles} cycle(s)')
            ledger.boundary()
            emit()
            return 'MAX_CYCLES'
        index += 1
        cycle_dir = root / 'cycles' / f'{index:03d}'
        record = dict(cycle=index, steps={})
        report['cycles'].append(record)
        try:
            # --- Observe: the measured world is the session's verified world ----------------------------------
            stream(args, 'G4_OBSERVE', cycle=index, motion=False)
            _, measured = measure(cycle_dir / 'measure', unclassified)
            now_world = measured_world(measured)
            expected = ledger.initial_world if not ledger.objects else ledger.render_world()
            if not ledger.objects:
                ledger.initial_world = expected = now_world
            record['steps']['observe'] = require_same_world(expected, now_world, 'observe')
            # --- Classify: render that world, EPD, bind classes to the measured objects ------------------------
            stream(args, 'G4_CLASSIFY', cycle=index, motion=False)
            revision = world_revision(expected)
            world_path = cycle_dir / 'world.json'
            world_path.write_text(json.dumps({'revision': revision, 'objects': expected}, indent=1) + '\n')
            ledger.observing = f'cycle {index}: rendering the measured world with the cell camera and classifying it with EPD'
            emit()
            run([sys.executable, SCRIPTS / 'perception_replay.py', 'session-frame', '--cell', scene, '--world', world_path,
                 '--output', cycle_dir / 'frame'],
                env={k: v for k, v in os.environ.items() if k not in ('ROS_DOMAIN_ID', 'IGN_PARTITION')},
                cwd=repo, log=cycle_dir / 'frame.log', timeout=900)
            ledger.observing = None
            frame_path = cycle_dir / 'frame/frame.yaml'
            raw = frame_path.read_bytes()
            frame = yaml.safe_load(raw)
            source = rendered_frame(frame)
            if source is None or source.get('world_revision') != revision:
                raise G4Stop('FRAME_WORLD_MISMATCH', f'cycle {index}: the frame is not a rendering of world {revision}')
            ledger.ingest_frame(frame, normalize(frame, time.time(), geometry),
                                source_sha256=hashlib.sha256(raw).hexdigest())
            observations_path, observations = measure(cycle_dir / 'classify', ['--perception', str(frame_path)])
            record['steps']['classify'] = dict(
                frame=str(frame_path), frame_sha256=hashlib.sha256(raw).hexdigest(), world_revision=revision,
                classes={o['object_id']: o['class_id'] for o in observations['objects']},
                confidence={o['object_id']: o.get('confidence') for o in observations['objects']},
                unbound_detections=len(observations['perception_binding']['unbound_detections']),
                world=require_same_world(expected, measured_world(observations), 'classify'))
            emit({'frame': index})
            # --- Select ---------------------------------------------------------------------------------------
            target = ledger.next_target(exclude=planned)
            record['steps']['select'] = dict(target=target, ledger={e['id']: dict(status=e['status'], reason=e.get('reason'))
                                                                    for e in ledger.objects.values()})
            if target is None:
                ledger.transition('STOPPING', 'BATCH_COMPLETE: no further routed object' +
                                  (' to plan' if planned else ''))
                ledger.boundary()
                emit()
                record['outcome'] = 'NO_ELIGIBLE_OBJECT'
                return 'BATCH_COMPLETE'
            ledger.begin_cycle(target)
            emit()
            # --- Plan (padded, plan-only, this target only) ---------------------------------------------------
            stream(args, 'G4_PLAN', cycle=index, target=target['id'], motion=False)
            plan_summary = cycle_dir / 'resolve/summary.json'
            with StageForwarder(args, cycle_dir / 'resolve/executor.log', 'g4', 'resolve'):
                planned_run = run(stage_a.executor_command(repo, scene, receipt_path, observations_path, plan_summary,
                                                           args.segment_planning_time, resolve=True,
                                                           search_timeout=args.search_timeout)
                                  + ['--target-object-id', target['id']],
                                  env=env, cwd=repo, log=cycle_dir / 'resolve/executor.log',
                                  timeout=args.search_timeout + 120 + stage_a.SERVICE_RETRY_ALLOWANCE_S, check=False)
            if planned_run.returncode != 0:
                outcome = plan_refused(ledger, plan_summary, planned_run.returncode, cycle_dir)
                record['steps']['plan'] = dict(refused=True, outcome=outcome,
                                               failure=ledger.objects[target['id']]['attempts'][-1]['failure'])
                record['outcome'] = 'PLAN_REFUSED'
                emit({'cycle_outcome': 'PLAN_REFUSED'})
                continue                                           # Observe again; the object stays an obstacle
            record['steps']['plan'] = check_plan(json.loads(plan_summary.read_text()), target, observations)
            # --- Validate: the planned-against geometry is still there; the frame is fresh --------------------
            stream(args, 'G4_VALIDATE', cycle=index, motion=False)
            run([sys.executable, SCRIPTS / 'simulator_observations.py', '--receipt', receipt_path, '--output',
                 cycle_dir / 'validate/observations.yaml', '--perception', frame_path, '--refresh-from', observations_path],
                env=env, cwd=repo, log=cycle_dir / 'validate/observations.log', timeout=90)
            frame_now = frame_record(frame_path)
            record['steps']['validate'] = dict(geometry='unchanged since classification (simulator refresh)', frame=frame_now)
            # --- Authorise ------------------------------------------------------------------------------------
            stream(args, 'G4_AUTHORISE', cycle=index, motion=False)
            occupancy = [dict(id=o['id'], pose=o['pose']) for o in ledger.occupied()
                         if ledger.objects[o['id']].get('destination') == target['destination']]
            try:
                cycle_sha = scope.g4_authorise_cycle(
                    cycle_dir / 'g4-cycle.json', session_path, session_sha, cycle=previous['cycle'] + 1 if previous else 1, frame=frame_now,
                    target=dict(object_id=target['id'].split('::', 1)[-1], class_id=target['class_id']),
                    destination=target['destination'], occupancy=occupancy, grasp=record['steps']['plan']['grasp'],
                    previous=previous, stop_latched=STOP_AFTER_CYCLE.record is not None)
            except scope.PhysicsScopeRefused as refused:
                record['steps']['authorise'] = dict(granted=False, refusal=str(refused))
                if not str(refused).startswith('G4_SCOPE_PLAN_ONLY'):
                    raise G4Stop(str(refused).split(':', 1)[0], str(refused))
                record['outcome'] = 'PLAN_ONLY'
                ledger.active, ledger.stage = None, None          # nothing executed: not a ledger attempt, not a count
                planned.add(target['id'])
                emit({'cycle_outcome': 'PLAN_ONLY'})
                if args.plan_every_target:
                    continue                                       # Observe again, then the next routed object
                ledger.transition('STOPPING', 'G4_SCOPE_PLAN_ONLY: no G4 scope is qualified; no cycle may move')
                ledger.boundary()
                emit()
                return 'PLAN_ONLY'
            # --- Execute / Verify / Reconcile (unreachable until a G4 scope and executor admission exist) -------
            record['steps']['authorise'] = dict(granted=True, record=str(cycle_dir / 'g4-cycle.json'), sha256=cycle_sha)
            raise G4Stop('G4_EXECUTOR_ADMISSION_MISSING', 'the executor admits no G4 cycle record yet; nothing moves')
        except SortingFault as fault:
            ledger.fail(fault.code, fault.reason)
            raise G4Stop(fault.code, fault.reason) from fault
        except G4Stop as stop:
            record['outcome'] = stop.code
            if ledger.state not in ('STOPPED', 'FAULT'):
                ledger.fail(stop.code, stop.reason)
            raise


if __name__ == '__main__':
    raise SystemExit(main())
