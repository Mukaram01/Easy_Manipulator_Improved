#!/usr/bin/env python3
"""G4: one continuous physical sorting session of a Workcell Builder cell in ONE persistent Gazebo world.

Every cycle runs, in order:
  Observe    measure every object in Gazebo (simulator_observations) and prove the session's verified world unchanged;
  Classify   render that world with the cell camera and classify it with EPD (perception_replay session-frame), then
             bind EPD's classes to the measured objects (simulator_observations --perception);
  Select     the sorting ledger (sorting_session, backend 'simulator') picks the next routed object; unclassified,
             unrouted, completed and removed objects are never targets and stay obstacles;
  Plan       the executor resolves that one target (plan-only, --target-object-id) on its routed bin, with the physics
             pads, every other object and both physical bins as obstacles and every measured object as occupancy;
  Validate   re-measure: the geometry the plan used is unchanged; the routed bin holds exactly what the ledger expects;
  Authorise  one per-cycle record under the G4 session (physics_scope_qualification.g4_authorise_cycle);
  Execute    the executor's guarded strict full-cycle of exactly that authorised plan (--g4-cycle);
  Verify     the executor's gate evidence and g4_verify_cycle, independently, before anything is counted;
  Reconcile  a fresh Gazebo measurement must show every other object unmoved and the placed part where the executor
             measured it; only then the ledger counts the cycle (at the re-measured pose); then Observe again.

Without ``--authorise-g4-trial`` the session is plan-only (G4_PLAN_ONLY): Authorise refuses with G4_SCOPE_PLAN_ONLY and
nothing moves; ``--plan-every-target`` plans each further routed object once in the same launch (never a ledger attempt
or a count). With ``--authorise-g4-trial`` (CLI only; ``--prior-evidence`` must be the workspace's registered Stage-A
acceptance; ``--max-cycles`` 1..2) the runner issues a G4_AUTHORISED_TRIAL record bound to this process: it alone
authorises cycles and only an executor it starts may move. Any refused, failed, unverified or unreconciled cycle
ends the session as a FAULT; a failed or timed-out helper step (measure, render, plan-only resolve, refresh) ends it
BLOCKED. Nothing is retried. The evidence is labelled G4_TRIAL_NOT_QUALIFIED.

Stop at cycle end: SIGUSR1 to this process only (the G5B latch; the running cycle finishes under every guard and is
verified, no further cycle starts). Cancel: SIGTERM/SIGINT (forwarded to the running child, G5A; status CANCELLED and
the ledger fails SESSION_INTERRUPTED).
Pause/stop requests in the control file are honoured at cycle boundaries (sorting_session.poll_control).
Exit codes: 0 PASS, 1 BLOCKED, 3 CANCELLED, 4 STOPPED (operator stop or refused input), 5 FAULT.
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
# The placed part re-measured from Gazebo's pose stream against the executor's own final measurement (seat tolerance).
PLACED_TOLERANCE_M, PLACED_TOLERANCE_RAD = 1e-3, 1e-3
# The render step picks ROS domains 170..219 (perception_replay.record); the physics launch stays outside them.
RENDER_DOMAINS = range(170, 220)
EXIT_CODES = {'PASS': 0, 'BLOCKED': 1, 'CANCELLED': 3, 'STOPPED': 4, 'FAULT': 5}
PREREQUISITE_GATES = ('cancel', 'telemetry', 'stationary', 'contact-release')


class G4Stop(Exception):
    """A G4 stop condition: the session ends at this boundary with ``code``. ``fault``: the world, a plan or a cycle
    is not what the session can prove (FAULT), as opposed to an operator stop or a refused input (STOPPED)."""
    def __init__(self, code, reason, *, fault=False):
        super().__init__(f'{code}: {reason}')
        self.code, self.reason, self.fault = code, reason, fault


def measured_world(snapshot):
    """A measured detected_objects/v1 snapshot as a session world [{object_id, pose: {frame_id, xyz, rpy}, dimensions}]."""
    return [{'object_id': o['object_id'], 'pose': {'frame_id': 'world', 'xyz': [float(v) for v in o['pose']['xyz']],
                                                   'rpy': [float(v) for v in o['pose']['rpy']]},
             'dimensions': [float(v) for v in o['dimensions']]}
            for o in sorted(snapshot['objects'], key=lambda o: o['object_id'])]


def world_difference(expected, measured, only=None):
    """(max position delta m, max orientation delta rad) between two worlds of the same objects (``only``: these ids);
    raises if the object sets differ."""
    a = {o['object_id']: o for o in expected}
    b = {o['object_id']: o for o in measured}
    if set(a) != set(b):
        raise G4Stop('G4_WORLD_CHANGED', f'objects {sorted(a)} became {sorted(b)}', fault=True)
    position = orientation = 0.0
    for oid, item in a.items():
        if only is not None and oid not in only:
            continue
        other = b[oid]
        if [round(v, 9) for v in item['dimensions']] != [round(v, 9) for v in other['dimensions']]:
            raise G4Stop('G4_WORLD_CHANGED', f'{oid} changed size', fault=True)
        position = max(position, math.dist(item['pose']['xyz'], other['pose']['xyz']))
        orientation = max(orientation, *(abs(math.remainder(x - y, math.tau))
                                         for x, y in zip(item['pose']['rpy'], other['pose']['rpy'])))
    return position, orientation


def require_same_world(expected, measured, what, only=None):
    position, orientation = world_difference(expected, measured, only)
    if position > WORLD_TOLERANCE_M or orientation > WORLD_TOLERANCE_RAD:
        raise G4Stop('G4_WORLD_CHANGED', f'{what}: the measured world moved {position:.3g} m / {orientation:.3g} rad from '
                     "the session's verified world; an unexplained change is never planned against", fault=True)
    return dict(max_position_delta_m=position, max_orientation_delta_rad=orientation)


def frame_record(frame_path):
    """{sha256, world_revision, age_s} of a rendered frame (report only; the authority re-reads the file itself)."""
    raw = Path(frame_path).read_bytes()
    source = yaml.safe_load(raw)['source']
    return dict(sha256=hashlib.sha256(raw).hexdigest(), world_revision=source['world_revision'],
                age_s=(time.time_ns() - int(source['capture_wall_ns'])) / 1e9)


def check_plan(summary, target, observations, routed_model=None):
    """The plan-only resolve is the target's: the right object, its route, class and routed bin, nothing executed."""
    import physics_scope_qualification as scope
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
    destination = (summary.get('physical_destination_walls') or {}).get('selected_destination') or {}
    if routed_model is not None and destination.get('model') != routed_model:
        problems.append(f"seated on {destination.get('model')!r}, not the routed bin {routed_model!r}")
    if problems:
        raise G4Stop('G4_PLAN_MISMATCH', '; '.join(problems), fault=True)
    grasp = (summary.get('selected_extraction_intent') or {}).get('candidate_id')
    return dict(route=route, physics_pads=summary['physics_pads'], grasp=grasp,
                grasp_qualified=str(grasp).split('::', 1)[0] in scope.G4_QUALIFIED_GRASP_STRATEGIES,
                resolution_sha256=summary.get('resolution_sha256'),
                stages=[p.get('stage') for p in summary.get('plan_metadata', [])], destination=destination)


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
        raise G4Stop('G4_PLAN_MOVED', 'a plan-only resolve reports execution; the world is unverified', fault=True)
    return ledger.record_cycle(summary, returncode, [], evidence_dir)


def verify_cycle(ledger, cycle_path, cycle_sha256, summary_path, binding, session_sha256=None):
    """Verify an executed cycle against its own authorisation (g4_verify_cycle, from the summary FILE). An unverified
    cycle faults the session; the ledger never sees it."""
    import physics_scope_qualification as scope
    target = ledger.active
    verification = scope.g4_verify_cycle(cycle_path, cycle_sha256, summary_path, binding, session_sha256=session_sha256)
    if not verification['verified']:
        ledger.active = None
        raise G4Stop('G4_CYCLE_UNVERIFIED', f"{target['id']}: " + '; '.join(verification['problems']), fault=True)
    return verification


def reconcile_measurement(ledger, before, after, target_id, final_pose):
    """After a verified cycle, before the ledger counts it: every other object unmoved (to the world tolerance) and the
    placed part re-measured from Gazebo's pose stream where the executor measured it. A mismatch faults the session
    with the cycle uncounted. Returns evidence, with the re-measured pose the ledger keeps (count_cycle)."""
    import perceived_object_grasp_plan as geometry
    oid = target_id.split('::', 1)[-1]
    others = {o['object_id'] for o in before} - {oid}
    try:
        moved = require_same_world(before, after, 'reconcile (objects other than the placed part)', only=others)
        placed = next(o for o in after if o['object_id'] == oid)
        position = math.dist(placed['pose']['xyz'], final_pose[:3])
        quaternion = geometry.quaternion_from_rpy(placed['pose']['rpy'])
        rotation = 2 * math.acos(min(1.0, abs(sum(a * b for a, b in zip(quaternion, final_pose[3:])))))
        if position > PLACED_TOLERANCE_M or rotation > PLACED_TOLERANCE_RAD:
            raise G4Stop('G4_WORLD_CHANGED', f'reconcile: the placed part is {position:.3g} m / {rotation:.3g} rad from '
                         "the executor's final measurement", fault=True)
    except G4Stop:
        ledger.active = None
        raise
    return dict(others=moved, placed_delta_m=position, placed_delta_rad=rotation,
                placed_pose=list(placed['pose']['xyz']) + list(quaternion))


def count_cycle(ledger, summary_path, returncode, evidence_dir, reconciled):
    """Only a verified and reconciled cycle is counted: the ledger records it from the executor evidence, then keeps the
    re-measured pose. Returns the ledger outcome."""
    target = ledger.active
    outcome = ledger.record_cycle(json.loads(Path(summary_path).read_text()), returncode, [], evidence_dir)
    if outcome != 'COMPLETED':
        raise G4Stop('G4_CYCLE_UNVERIFIED', f"{target['id']}: the ledger did not count the verified cycle ({outcome})",
                     fault=True)
    entry = ledger.objects[target['id']]
    entry['pose'] = list(reconciled['placed_pose'])
    entry['reconciled_from'] = 'Gazebo pose stream after the verified cycle'
    return outcome


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


def expected_occupants(ledger, binding):
    """The ledger's view of what rests in a bin: every known object whose ledger pose lies over the bin's floor."""
    import physics_scope_qualification as scope
    world = ledger.render_world() if ledger.objects else ledger.initial_world
    return scope.destination_occupants({'objects': world}, binding)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--scene-package', type=Path, required=True, help='the Workcell Builder cell (saved and generated)')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=226, help='ROS domain of the physics launch (not 170..219)')
    parser.add_argument('--partition-prefix', default='g4_sorting')
    parser.add_argument('--max-cycles', type=int, help='session bound (default: every object; a trial: 1..2, required)')
    parser.add_argument('--plan-every-target', action='store_true',
                        help='plan-only: after a refused authorisation, plan each further routed object once')
    parser.add_argument('--authorise-g4-trial', action='store_true',
                        help='CLI only, never Builder/HMI: issue a bounded G4 trial (G4_TRIAL_NOT_QUALIFIED) for '
                             'at most 2 guarded physical cycles in this session')
    parser.add_argument('--prior-evidence', type=Path,
                        help='with --authorise-g4-trial: the workspace\'s registered Stage-A acceptance run')
    parser.add_argument('--segment-planning-time', type=float, default=3.0)
    parser.add_argument('--search-timeout', type=float, default=300.0)
    parser.add_argument('--control-file', type=Path, help='{"commands": [{"seq": n, "command": pause|resume|stop}]}')
    parser.add_argument('--stream-status', action='store_true')
    args = parser.parse_args(argv)
    if not 1 <= args.domain <= 232 or args.domain in RENDER_DOMAINS:
        parser.error('--domain must be 1..232 and outside the render domains 170..219')
    if not 0 < args.segment_planning_time <= 10 or not 0 < args.search_timeout < math.inf:
        parser.error('invalid planning budget')
    if args.max_cycles is not None and args.max_cycles < 1:
        parser.error('--max-cycles must be at least 1')
    if args.authorise_g4_trial:
        import physics_scope_qualification as scope
        if args.prior_evidence is None or args.max_cycles is None or not args.max_cycles <= scope.G4_TRIAL_MAX_CYCLES:
            parser.error(f'--authorise-g4-trial needs --prior-evidence and --max-cycles 1..{scope.G4_TRIAL_MAX_CYCLES}')
        if args.plan_every_target:
            parser.error('--plan-every-target is plan-only; a trial executes its authorised cycles')
    elif args.prior_evidence is not None:
        parser.error('--prior-evidence applies to --authorise-g4-trial only')
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
    report = dict(schema='workcell_g4_sorting_run/v1', status='BLOCKED',
                  scope='G4_TRIAL_NOT_QUALIFIED' if args.authorise_g4_trial else 'G4_PLAN_ONLY_NO_MOTION',
                  started_wall_ns=time.time_ns(), argv=list(sys.argv[1:] if argv is None else argv), cycles=[])
    try:
        report['result'] = session(args, repo, workspace, report)
        report['status'] = ('STOPPED' if report['result'] == 'STOPPED_BY_OPERATOR' or STOP_AFTER_CYCLE.record is not None
                            else 'PASS')
    except G4Stop as stop:
        report.update(status='FAULT' if stop.fault else 'STOPPED', stop=dict(code=stop.code, reason=stop.reason,
                                                                             fault=stop.fault))
    except Exception as exc:
        report.update(status='BLOCKED', failure=str(exc))
    except OperatorCancel as exc:
        report.update(status='CANCELLED', failure=f'cancelled by the operator ({exc})',
                      operator_cancel=dict(signal=str(exc), child=getattr(exc, 'child', None)))
        if STOP_AFTER_CYCLE.record is not None:
            STOP_AFTER_CYCLE.record['superseded_by'] = str(exc)      # cancel takes precedence
    finally:
        for sig in (signal.SIGTERM, signal.SIGINT):
            signal.signal(sig, signal.SIG_IGN)                       # cleanup is never re-interrupted
        state = report.pop('_state', {})
        launch, launch_log, ledger = state.get('launch'), state.get('launch_log'), state.get('ledger')
        if ledger is not None and report['status'] in ('BLOCKED', 'CANCELLED', 'FAULT') and \
                ledger.state not in ('STOPPED', 'FAULT'):
            ledger.fail({'CANCELLED': 'SESSION_INTERRUPTED', 'FAULT': (report.get('stop') or {}).get('code')}.get(
                report['status'], 'SESSION_BLOCKED'), report.get('failure') or (report.get('stop') or {}).get('reason', ''))
        if launch is not None:
            stream(args, 'SIMULATOR_SHUTDOWN')
            report['shutdown'] = stage_a.stop_owned(launch, args.output / 'session/launch.log')
            if not report['shutdown']['clean']:
                report['shutdown_failure'] = 'owned shutdown failed'
                if report['status'] == 'PASS':           # a clean result is never claimed; a fault or cancel is kept
                    report['status'] = 'BLOCKED'
        if launch_log is not None:
            launch_log.close()
        if ledger is not None:
            report['ledger'] = ledger.report()
            report['counts'] = ledger.counts()
        if STOP_AFTER_CYCLE.record is not None:
            report['stop_after_cycle'] = STOP_AFTER_CYCLE.record
        report.update(finished_wall_ns=time.time_ns(), duration_s=round(time.monotonic() - started, 1))
        (args.output / REPORT_NAME).write_text(json.dumps(report, indent=2, default=str) + '\n')
        print(json.dumps({key: report.get(key) for key in ('status', 'result', 'stop', 'failure', 'counts')}, default=str))
    return EXIT_CODES[report['status']]


def session(args, repo, workspace, report):
    """Run the session; returns its result. The launch, its log and the ledger are kept in report['_state']."""
    import physics_scope_qualification as scope
    from runtime_pick_inputs import normalize, rendered_frame, world_revision
    from simulator_world import TEMPLATE, derive_world, engine_equivalent
    from sorting_session import SortingFault, SortingSession, write_json_atomic
    from task_intent_v2 import class_routing_rules, freshness_source_type
    import perceived_object_grasp_plan as geometry
    state = report.setdefault('_state', {})
    out = args.output
    trial = args.authorise_g4_trial

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
    root = out / 'session'
    root.mkdir()
    commission = None
    if trial:
        # A trial reuses only the same-build prerequisite trials of the workspace's registered Stage-A acceptance.
        from qualified_physics_runtime import load as load_qualified_runtime
        registered = Path(load_qualified_runtime(workspace)['stage_a_acceptance']['evidence']).resolve()
        if Path(args.prior_evidence).resolve() != registered:
            raise G4Stop('G4_EVIDENCE_UNREGISTERED', f'a trial reuses the registered Stage-A acceptance {registered} only')
        prior = stage_a.prior_evidence(args.prior_evidence, build)
        if any(name not in prior for name in PREREQUISITE_GATES):
            raise G4Stop('G4_EVIDENCE_UNREGISTERED', 'the registered Stage-A acceptance lacks a prerequisite trial')
        commission = root / 'commission-evidence.json'
        commission.write_text(json.dumps([json.loads(prior[name].read_text()) for name in PREREQUISITE_GATES],
                                         indent=2) + '\n')
        report['prior_evidence'] = {name: str(prior[name]) for name in PREREQUISITE_GATES}

    # --- one persistent launch -------------------------------------------------------------------------------------
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

    # --- the G4 session record: plan-only, or a bounded CLI trial bound to this process ------------------------------
    objects = world_report['objects']
    max_cycles = args.max_cycles or len(objects)
    native_manifest = workspace / 'install/workcell_builder/share/workcell_builder/commission_execute_build.json'

    def current_binding():
        """The session binding recomputed from the current files (as the executor recomputes it)."""
        return scope.g4_session_binding(
            cell_dir=scene, receipt_path=receipt_path, native=scope.native_identity(native_manifest),
            runtime=scope.runtime_identity(workspace), perception=scope.g4_perception(workspace, intent),
            max_cycles=max_cycles, commission_evidence_path=commission)

    binding = current_binding()
    session_path = root / 'g4-session.json'
    if trial:
        session_sha = scope.g4_authorise_trial(session_path, binding, scope.process_identity(os.getpid()))
        session_state = scope.G4_AUTHORISED_TRIAL
    else:
        session_sha = scope.g4_open_session(session_path, binding)
        session_state = scope.G4_PLAN_ONLY
    report['g4_session'] = dict(path=str(session_path), sha256=session_sha, state=session_state,
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

    def stop_cleanly(reason):
        """A latched stop before motion: no cycle starts; the session ends STOPPED, not FAULT."""
        ledger.active, ledger.stage = None, None
        if ledger.state not in ('STOPPING', 'STOPPED'):
            ledger.transition('STOPPING', reason)
        ledger.boundary()
        emit()
        return 'STOPPED_BY_OPERATOR'

    unclassified = ['--class-map', json.dumps({o['object_id']: 'unclassified' for o in objects})]
    planned, index, executed, previous = set(), 0, 0, None   # previous: the last verified cycle {path, sha256, summary_path}
    ledger.transition('READY', 'Gazebo + MoveIt launched; G4 session opened (' + session_state + ')')
    ledger.transition('RUNNING', 'sorting started')
    emit()
    while True:
        # --- cycle boundary: operator pause/stop, the stop-after-cycle latch, the session bound -------------------
        for applied in ledger.poll_control(control, cycle_active=False):
            emit({'control': applied})
        ledger.boundary()
        if ledger.state == 'STOPPED' or STOP_AFTER_CYCLE.record is not None:
            return stop_cleanly('stop after cycle (SIGUSR1)') if ledger.state != 'STOPPED' else 'STOPPED_BY_OPERATOR'
        if ledger.state == 'PAUSED':
            time.sleep(0.5)
            continue
        if (executed if trial else index) >= max_cycles:
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
                raise G4Stop('FRAME_WORLD_MISMATCH', f'cycle {index}: the frame is not a rendering of world {revision}',
                             fault=True)
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
            routed = ledger.physical_destinations[target['destination']]
            # --- Plan (padded, plan-only, this target only, on its routed bin) --------------------------------
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
                failure = ledger.objects[target['id']]['attempts'][-1]['failure']
                record['steps']['plan'] = dict(refused=True, outcome=outcome, failure=failure)
                record['outcome'] = 'PLAN_REFUSED'
                emit({'cycle_outcome': 'PLAN_REFUSED'})
                if trial:                                          # a trial aborts on the first non-PASS: no re-plan
                    raise G4Stop('G4_PLAN_REFUSED', f"{target['id']}: {failure}; nothing moved", fault=True)
                continue                                           # plan-only: Observe again; it stays an obstacle
            record['steps']['plan'] = check_plan(json.loads(plan_summary.read_text()), target, observations,
                                                 routed['model'])
            # --- Validate: the planned-against geometry is still there; the bin holds what the ledger expects -----
            stream(args, 'G4_VALIDATE', cycle=index, motion=False)
            validated_path = cycle_dir / 'validate/observations.yaml'
            run([sys.executable, SCRIPTS / 'simulator_observations.py', '--receipt', receipt_path, '--output',
                 validated_path, '--perception', frame_path, '--refresh-from', observations_path],
                env=env, cwd=repo, log=cycle_dir / 'validate/observations.log', timeout=90)
            expected_in_bin = expected_occupants(ledger, routed)
            measured_in_bin = scope.destination_occupants(yaml.safe_load(validated_path.read_text()), routed)
            if measured_in_bin != expected_in_bin:
                raise G4Stop('G4_OCCUPANCY_MISMATCH', f"{routed['source_item_id']} holds {measured_in_bin}; the session "
                             f'expects {expected_in_bin}', fault=True)
            record['steps']['validate'] = dict(geometry='unchanged since classification (simulator refresh)',
                                               frame=frame_record(frame_path), destination=routed['source_item_id'],
                                               occupants=measured_in_bin)
            # --- Authorise ------------------------------------------------------------------------------------
            if STOP_AFTER_CYCLE.record is not None:
                record['outcome'] = 'STOPPED_BEFORE_AUTHORISATION'
                return stop_cleanly('stop after cycle (SIGUSR1) before authorisation')
            stream(args, 'G4_AUTHORISE', cycle=index, motion=False)
            cycle_path = cycle_dir / 'g4-cycle.json'
            try:
                cycle_sha = scope.g4_authorise_cycle(
                    cycle_path, session_path, session_sha, current=current_binding() if trial else None,
                    cycle=executed + 1, target=dict(object_id=target['id'].split('::', 1)[-1], class_id=target['class_id']),
                    destination=target['destination'], frame_path=frame_path, observations_path=validated_path,
                    plan_path=plan_summary, occupancy=expected_in_bin, previous=previous,
                    stop_latched=STOP_AFTER_CYCLE.record is not None)
            except scope.PhysicsScopeRefused as refused:
                record['steps']['authorise'] = dict(granted=False, refusal=str(refused))
                if str(refused).startswith('G4_SESSION_STOPPED'):   # latched while authorising: nothing issued
                    record['outcome'] = 'STOPPED_BEFORE_AUTHORISATION'
                    return stop_cleanly('stop after cycle (SIGUSR1) before authorisation')
                if not str(refused).startswith('G4_SCOPE_PLAN_ONLY'):
                    raise G4Stop(str(refused).split(':', 1)[0], str(refused), fault=True)
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
            record['steps']['authorise'] = dict(granted=True, record=str(cycle_path), sha256=cycle_sha, cycle=executed + 1)
            if STOP_AFTER_CYCLE.record is not None:                # latched while authorising: the cycle never runs
                record['outcome'] = 'STOPPED_BEFORE_EXECUTION'
                return stop_cleanly('stop after cycle (SIGUSR1) before execution')
            # --- Execute: exactly the authorised plan, under every guard; never retried --------------------------
            stream(args, 'G4_EXECUTE', cycle=index, target=target['id'], motion=True)
            summary_path = cycle_dir / 'execute/summary.json'
            command = stage_a.executor_command(repo, scene, receipt_path, validated_path, summary_path,
                                               args.segment_planning_time, gate='full-cycle', evidence=commission,
                                               search_timeout=args.search_timeout) + [
                '--target-object-id', target['id'], '--g4-session', session_path, '--g4-session-sha256', session_sha,
                '--g4-cycle', cycle_path, '--g4-cycle-sha256', cycle_sha, '--g4-plan', plan_summary]
            def partial_summary():
                try:
                    return json.loads(summary_path.read_text())
                except (OSError, ValueError):
                    return {'summary_unreadable': True}
            try:
                with StageForwarder(args, cycle_dir / 'execute/executor.log', 'g4', 'execute'):
                    executed_run = run(command, env=env, cwd=repo, log=cycle_dir / 'execute/executor.log',
                                       timeout=args.search_timeout + stage_a.EXECUTION_GATE_MARGIN_S, check=False)
            except OperatorCancel:
                partial = partial_summary()
                record['steps']['execute'] = {key: partial.get(key) for key in (
                    'execution_attempted', 'failed_stage', 'failure', 'cancellation_confirmed', 'recovery_required',
                    'transfer_cancellation', 'summary_unreadable')}
                record['outcome'] = 'CANCELLED'
                raise
            except Exception as exc:                               # e.g. a timeout: run() SIGKILLed the executor
                record['steps']['execute'] = dict(error=str(exc), **{key: partial_summary().get(key) for key in (
                    'execution_attempted', 'failed_stage', 'failure', 'summary_unreadable')})
                raise G4Stop('G4_CYCLE_FAILED', f'cycle {executed + 1}: the executor did not finish ({exc}); the world '
                             'is unverified', fault=True) from exc
            executed += 1                                          # this cycle number is spent, whatever happened
            summary = partial_summary()
            summary = None if summary.get('summary_unreadable') else summary   # unreadable after motion: a FAULT below
            record['steps']['execute'] = dict(summary=str(summary_path), returncode=executed_run.returncode,
                                              result=(summary or {}).get('result'),
                                              execution_attempted=(summary or {}).get('execution_attempted'),
                                              failure=(summary or {}).get('failure'))
            if executed_run.returncode != 0 or not isinstance(summary, dict) or summary.get('result') != 'PASS':
                moved = not isinstance(summary, dict) or summary.get('execution_attempted') is not False
                raise G4Stop('G4_CYCLE_FAILED' if moved else 'G4_EXECUTION_REFUSED',
                             f"cycle {executed}: {(summary or {}).get('failure') or 'no executor evidence'}"
                             + ('; the world is unverified' if moved else '; nothing moved'), fault=True)
            # --- Verify: the gate evidence, then the cycle against its own authorisation --------------------------
            stream(args, 'G4_VERIFY', cycle=index, motion=False)
            try:
                stage_a.assert_gate('full-cycle', summary, build['sha256'], moveit_identity)
                verification = verify_cycle(ledger, cycle_path, cycle_sha, summary_path, binding,
                                            session_sha256=session_sha)
                record['steps']['verify'] = dict(verified=True, evidence_sha256=verification['evidence_sha256'])
                # --- Reconcile: a fresh Gazebo measurement, then the ledger counts the cycle ---------------------
                _, after = measure(cycle_dir / 'reconcile', unclassified)
                record['steps']['reconcile'] = reconcile_measurement(
                    ledger, measured_world(yaml.safe_load(validated_path.read_text())), measured_world(after),
                    target['id'], summary['full_cycle_physical_acceptance']['final_pose'])
                record['steps']['verify']['ledger_outcome'] = count_cycle(
                    ledger, summary_path, executed_run.returncode, cycle_dir / 'execute', record['steps']['reconcile'])
            except (G4Stop, SortingFault):
                raise
            except Exception as exc:                               # anything after motion that cannot be proven
                raise G4Stop('G4_CYCLE_UNVERIFIED', f'cycle {executed}: {exc}', fault=True) from exc
            previous = dict(path=str(cycle_path), sha256=cycle_sha, summary_path=str(summary_path))
            record['outcome'] = 'COMPLETED'
            emit({'cycle_outcome': 'COMPLETED'})
        except SortingFault as fault:
            ledger.fail(fault.code, fault.reason)
            raise G4Stop(fault.code, fault.reason, fault=True) from fault
        except G4Stop as stop:
            record['outcome'] = stop.code
            if ledger.state not in ('STOPPED', 'FAULT'):
                ledger.fail(stop.code, stop.reason)
            raise


if __name__ == '__main__':
    raise SystemExit(main())
