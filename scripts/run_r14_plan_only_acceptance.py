#!/usr/bin/env python3
"""Owned replay acceptance: R1.4 plan-only, explicit R1.5 --execute on mock hardware, or an R2.4 sorting session."""
import argparse
import json
import hashlib
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import time

STAGES = ['PREPLAN_APPROACH', 'PREPLAN_GRASP', 'PREPLAN_CLOSE_GRIPPER',
          'PREPLAN_LIFT', 'PREPLAN_TRANSFER', 'PREPLAN_PLACE',
          'PREPLAN_OPEN_GRIPPER', 'PREPLAN_RETREAT', 'PREPLAN_HOME']


def check_result(result, execution_goals):
    if (result.get('result') != 'PLAN_ONLY' or not result.get('full_cycle_prevalidated') or
            result.get('execution_attempted') is not False or
            result.get('prevalidation_left_live_scene_unchanged') is not True):
        raise RuntimeError('executor did not prove an unchanged, complete plan-only cycle')
    plans = result.get('plan_metadata', [])
    if [p['stage'] for p in plans] != STAGES or any(
            not p.get('success') or p.get('moveit_code') != 1 or p.get('points', 0) < 2 for p in plans):
        raise RuntimeError('not all nine manipulation stages planned successfully')
    if not result.get('selected_object_id') or not result.get('inserted_object_ids'):
        raise RuntimeError('missing target ingestion/grasp evidence')
    if execution_goals:
        raise RuntimeError('unexpected trajectory execution action observed')


def process_failures(log_text):
    return [line for line in log_text.splitlines()
            if (match := re.search(r'process has died.*exit code (-?\d+)', line))
            and int(match[1]) not in (0, -signal.SIGINT, -signal.SIGTERM)]


# Visualization may abort in upstream teardown; it is reported, never judged as the planning stack.
VISUALIZATION_PROCESSES = ('rviz2',)


def process_exits(log_text):
    """Each owned launch child's actual terminal status, as ros2 launch recorded it."""
    exits = []
    for line in log_text.splitlines():
        match = re.search(r'\[([^\]\s]+?)-\d+\]: process has (finished cleanly|died) \[pid (\d+)(?:, exit code (-?\d+))?', line)
        if match:
            exits.append(dict(process=match[1], pid=int(match[3]),
                              exit_code=0 if match[2] == 'finished cleanly' else int(match[4])))
    return exits


def shutdown_failures(exits):
    """Owned planning-stack processes that crashed instead of exiting after the owned stop."""
    return [e for e in exits if e['exit_code'] not in (0, -signal.SIGINT, -signal.SIGTERM)
            and e['process'] not in VISUALIZATION_PROCESSES]


def moveit_teardown_dependency():
    """Installed Humble MoveIt 2.5.10 unloads its controller plugin before the controller node and
    exits -11 on every owned shutdown (docs/manuals/MOVEIT_HUMBLE_TEARDOWN_OVERLAY.md). An environment
    that selects any other MoveIt must select exactly the qualified teardown overlay."""
    from ament_index_python.packages import get_package_prefix
    if get_package_prefix('moveit_ros_planning').startswith('/opt/'):
        return dict(status='INSTALLED_UNPATCHED'), None
    from simulator_backend import active_moveit_overlay, moveit_overlay_identity
    overlay = active_moveit_overlay()
    return dict(status='QUALIFIED_OVERLAY', manifest=overlay['manifest_path'],
                identity=list(moveit_overlay_identity(overlay))), overlay


def owned_move_group_overlay(pgid, overlay):
    """Bind the qualified dependency to the owned MoveGroup's actual mappings, not to the environment."""
    from simulator_backend import process_info, verify_moveit_overlay_maps
    matches = []
    for entry in Path('/proc').iterdir():
        try:
            if (not entry.name.isdigit() or
                    int((entry/'stat').read_text().rsplit(')', 1)[1].split()[2]) != pgid):
                continue
            info = process_info(int(entry.name))
        except (OSError, ValueError, IndexError):
            continue
        if '/moveit_ros_move_group/move_group ' in info['command']:
            loaded = verify_moveit_overlay_maps(overlay, info['libraries'])
            matches.append({k: v for k, v in loaded.items() if k.startswith('loaded_')})
    if len(matches) != 1:
        raise RuntimeError('MOVEIT_OVERLAY_REJECTED: exactly one owned MoveGroup process required')
    return matches[0]


def check_sequence_result(result, execution_goals):
    if (result.get('result') != 'PLAN_ONLY' or result.get('sequence_planning_completed') is not True or
            result.get('execution_attempted') is not False or not isinstance(result.get('sequence'), list)):
        raise RuntimeError('executor did not complete a plan-only sequence evaluation')
    for entry in result['sequence']:
        if entry.get('result') == 'PLANNED' and (
                [p['stage'] for p in entry.get('plan_metadata', [])] != STAGES or
                any(not p.get('success') or p.get('moveit_code') != 1 or p.get('points', 0) < 2
                    for p in entry['plan_metadata'])):
            raise RuntimeError(f"sequence entry {entry.get('object_id')} lacks nine successful stages")
    if execution_goals:
        raise RuntimeError('unexpected trajectory execution action observed')


def check_execution_result(result, statuses):
    required = ['full_cycle_prevalidated', 'prevalidation_left_live_scene_unchanged',
                'execution_attempted', 'full_cycle_execution_success', 'attach_verified',
                'detach_verified', 'home_verified', 'destination_verified',
                'final_collision_valid', 'baseline_acm_restored', 'shutdown_clean']
    if result.get('result') != 'PASS' or any(result.get(k) is not True for k in required):
        raise RuntimeError('incomplete state-verified execution')
    plans = result.get('plan_metadata', [])
    if [p['stage'] for p in plans] != STAGES or any(
            not p.get('success') or p.get('moveit_code') != 1 or p.get('points', 0) < 2 for p in plans):
        raise RuntimeError('incomplete nine-stage prevalidation')
    expected = [s.replace('PREPLAN_', 'EXECUTE_') for s in STAGES]
    results = result.get('execution_results', [])
    if [r['stage'] for r in results] != expected or any(
            r.get('code') != 1 or r.get('action_status') != 4 for r in results):
        raise RuntimeError('incomplete successful action results')
    for topic, count in [('/execute_trajectory', 9),
                         ('/ur5_arm_controller/follow_joint_trajectory', 7),
                         ('/ur5_gripper_controller/follow_joint_trajectory', 2)]:
        observed = [status for (t, _), status in statuses.items() if t == topic]
        if len(observed) != count or any(status != 4 for status in observed):
            raise RuntimeError(f'controller terminal success missing: {topic}: {observed}')
    guard = result.get('fake_hardware_guard', {})
    if (guard.get('move_group_use_fake_hardware') is not True or guard.get('real_hardware') is not False
            or not guard.get('hardware_classes')
            or set(guard['hardware_classes']) != {'mock_components/GenericSystem'}):
        raise RuntimeError('exclusive mock hardware not proven')
    world = result['final_planning_scene']['world_ids']
    if (result.get('selected_object_id') != 'runtime::sample-cup' or result.get('selected_grasp_index') != 3
            or world.count('runtime::sample-cup') != 1 or world.count('runtime::sample-bottle') != 1
            or result['final_planning_scene']['attached_ids']):
        raise RuntimeError('canonical target/distractor final state not proven')


def group_alive(pgid):
    # A launch may exit before a child. Check owned group members, excluding zombies.
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] != 'Z' and int(fields[2]) == pgid:
                return True
        except (OSError, ValueError, IndexError):
            pass
    return False


def stop(process):
    if process is None:
        return True
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    deadline = time.monotonic() + 10
    while group_alive(process.pid) and time.monotonic() < deadline:
        process.poll()
        time.sleep(.1)
    graceful = not group_alive(process.pid)
    for sig in (signal.SIGTERM, signal.SIGKILL):
        if not group_alive(process.pid):
            break
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + 3
        while group_alive(process.pid) and time.monotonic() < deadline:
            process.poll()
            time.sleep(.1)
    process.wait(timeout=3)
    return graceful and not group_alive(process.pid)


class _SessionDone(Exception):
    """The sorting session ended (completed, stopped or faulted) and wrote its own report."""


def run_sorting_session(args, out, scene_dir, input_path, replay_input, intent, executor_path, elog, statuses,
                        spin_until, stream_stages, last_stage, audit, node, rclpy):
    """R2.4: one owned fake-hardware launch, one executor run per object, until the batch is done or stopped.

    The session (scripts/sorting_session.py) chooses the target and reconciles the world; every cycle is the
    existing executor resolving and executing exactly that target. Operator pause/stop are read from the
    control file and honoured at cycle boundaries. Returns None; the audit carries the session report.
    """
    import hashlib
    import yaml
    from sorting_session import SortingFault, SortingSession, write_json_atomic
    from runtime_pick_inputs import normalize, replay_snapshot
    import perceived_object_grasp_plan as geometry
    document = yaml.safe_load((scene_dir / 'environment.yaml').read_text())
    environment = document.get('environment', document)
    mode = 'batch_replay' if replay_input else 'continuous'
    control = args.control_file or out / 'sorting_control.json'
    report_path = out / 'sorting_session.json'
    session = SortingSession(intent, environment, session_id=out.name, mode=mode, scene=str(scene_dir))
    child = None

    def emit(extra=None):
        status = session.status()
        status['stage'] = last_stage[0] if session.active else None
        audit['sorting_status'] = status
        write_json_atomic(report_path, session.report())
        if args.stream_status:
            print(json.dumps(dict(event='sorting', status=status, **(extra or {}))), flush=True)

    def apply_control(cycle_active):
        for applied in session.poll_control(control, cycle_active=cycle_active):
            emit({'control': applied})

    try:
        if mode == 'continuous':
            raise SortingFault('CONTINUOUS_SOURCE_UNAVAILABLE',
                               'continuous sorting needs successive live frames (camera or simulator measurement); '
                               'this runner has no live frame source for the saved task. Use a replayed batch.')
        raw = input_path.read_bytes()
        base = yaml.safe_load(raw)
        now = time.time()
        session.ingest_frame(base, normalize(replay_snapshot(base, now) if replay_input else base, now, geometry),
                             source_sha256=hashlib.sha256(raw).hexdigest())
        session.transition('READY', 'fake-hardware MoveIt launched; replayed batch frame ingested')
        emit()
        session.transition('RUNNING', 'sorting started')
        emit()
        while True:
            apply_control(cycle_active=False)
            before_boundary = session.state
            session.boundary()
            if session.state != before_boundary:
                emit()                                   # PAUSING -> PAUSED / STOPPING -> STOPPED are reported too
            if session.state == 'STOPPED':
                break
            if session.state == 'PAUSED':
                spin_until_or(lambda: False, spin_until, 0.5)
                continue
            target = session.next_target()
            if target is None:
                session.transition('STOPPING', 'BATCH_COMPLETE: no further object can be sorted from this batch')
                session.boundary()
                emit()
                break
            session.begin_cycle(target)
            emit()
            cycle_dir = out / 'cycles' / f"{session.active['cycle']:03d}"
            cycle_dir.mkdir(parents=True)
            snapshot_path = cycle_dir / 'observations.yaml'
            snapshot_path.write_text(yaml.safe_dump(session.world_snapshot(base), sort_keys=False))
            before = set(statuses)
            child = subprocess.Popen([sys.executable, str(executor_path), '--scene-package', str(scene_dir),
                '--detections', str(snapshot_path), '--replay', '--resolve-task', '--start',
                '--target-object-id', target['id'], '--timeout', str(args.cycle_timeout),
                '--sorting-cycle', json.dumps({'session_id': session.session_id, 'cycle': session.active['cycle']}),
                '--summary-output', str(cycle_dir / 'executor.json')],
                stdout=elog, stderr=subprocess.STDOUT, start_new_session=True)
            def finished():
                apply_control(cycle_active=True)
                return child.poll() is not None
            spin_until(finished)
            stream_stages()
            for _ in range(10):   # status messages published just before the executor exited
                rclpy.spin_once(node, timeout_sec=.1)
            summary = json.loads((cycle_dir / 'executor.json').read_text()) if (cycle_dir / 'executor.json').exists() else {}
            new_statuses = [status for key, status in statuses.items() if key not in before]
            outcome = session.record_cycle(summary, child.returncode, new_statuses, cycle_dir)
            child = None
            emit({'cycle_outcome': outcome})
        audit['result'] = 'PASS'
    except SortingFault as fault:
        session.fail(fault.code, fault.reason)
        audit['result'], audit['failure'] = 'FAIL', str(fault)
    except BaseException as exc:
        session.fail('SESSION_INTERRUPTED', str(exc))
        audit['result'], audit['failure'] = 'FAIL', f'SESSION_INTERRUPTED: {exc}'
    finally:
        if child is not None:
            audit['sorting_cycle_executor_stopped_cleanly'] = stop(child)
        audit['sorting_session'] = session.report()
        audit['sorting_counts'] = session.counts()
        emit()
    return None


def spin_until_or(predicate, spin_until, seconds):
    """Spin (launch liveness and deadline still checked) for at most `seconds` or until predicate()."""
    end = time.monotonic() + seconds
    spin_until(lambda: predicate() or time.monotonic() >= end)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=120, help='Total launch/planning budget; cleanup adds at most 16 seconds per process')
    parser.add_argument('--domain-id', type=int, default=179, help='An unused, isolated ROS domain')
    parser.add_argument('--execute', action='store_true', help='R1.5: execute the complete cycle on exclusively mock hardware')
    parser.add_argument('--launch-rviz', action='store_true', help='Show the single owned scene in RViz')
    parser.add_argument('--scene-dir', type=Path, help='Saved authored scene; defaults to canonical regression')
    parser.add_argument('--resolve-task', action='store_true', help='Use saved TaskIntent through the shared resolver')
    parser.add_argument('--resolve-if-needed', action='store_true', help='Studio: consume current resolution, or resolve a blocked/unresolved saved task')
    parser.add_argument('--detections', type=Path, help='Replay observation input, never a task override')
    parser.add_argument('--stream-status', action='store_true', help='Emit structured stage/result events for Studio')
    parser.add_argument('--target-object-id', help='Multi-object plan check: plan only this eligible object; all '
                        'other observations stay in the fresh PlanningScene as obstacles. Writes no task artifacts.')
    parser.add_argument('--sequence-object-ids', help='Sequential plan-only check of these eligible IDs, in order, '
                        'against validated virtual revisions inside this owned launch. Writes no task artifacts.')
    parser.add_argument('--sequence-placements', help='JSON object of operator-chosen target-local places per '
                        'sequence object ID (plan-only; passed to the executor unchanged)')
    parser.add_argument('--sorting-session', action='store_true', help='R2.4: sort the saved class-routed task one '
                        'object per cycle on exclusively mock hardware in one owned launch, until the batch is done '
                        'or the operator stops it (control file: pause/resume/stop at cycle boundaries)')
    parser.add_argument('--control-file', type=Path, help='Sorting session operator commands '
                        '({"commands": [{"seq": n, "command": "pause|resume|stop"}]}); default <output>/sorting_control.json')
    parser.add_argument('--cycle-timeout', type=float, default=300, help='Sorting: candidate search budget per cycle')
    args = parser.parse_args()
    if args.sorting_session and (args.execute or args.target_object_id or args.sequence_object_ids or args.detections
                                 or args.resolve_if_needed):
        parser.error('--sorting-session runs the saved task and its saved input; it excludes the other modes')
    if args.sequence_placements is not None and args.sequence_object_ids is None:
        parser.error('--sequence-placements requires --sequence-object-ids')
    if args.target_object_id is not None and args.execute:
        parser.error('--target-object-id is plan-only')
    if args.sequence_object_ids is not None and (args.execute or args.target_object_id is not None):
        parser.error('--sequence-object-ids is plan-only and excludes --target-object-id')
    if not 0 < args.timeout <= (7200 if args.sorting_session else 3600 if args.sequence_object_ids else 900) or not 0 <= args.domain_id <= 232:
        parser.error('timeout must be in (0,900]; domain-id in [0,232]')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1')
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    root = Path(__file__).resolve().parents[1]
    scene_dir = (args.scene_dir or root / 'scenes/ur5_2f_test').resolve()
    scene_name = scene_dir.name
    import yaml
    if args.target_object_id is not None or args.sequence_object_ids is not None or args.sorting_session:
        args.resolve_if_needed, args.resolve_task = False, True
    if args.resolve_if_needed and not args.resolve_task:
        from task_intent_resolver import read_scene_task, scene_resolution
        intent, physical, document = read_scene_task(scene_dir)
        current = scene_resolution(scene_dir, intent, physical, document)
        args.resolve_task = current['readiness_status'] == 'BLOCKED'
    uses_authored_task = args.resolve_if_needed or args.resolve_task or yaml.safe_load((scene_dir / 'cell_definition.yaml').read_text()).get('builder_task_intent', {}).get('schema') == 'workcell_builder_task_intent/v2'
    if uses_authored_task:
        from task_intent_resolver import read_scene_task
        from task_intent_authoring import planning_input
        saved_intent, _, _ = read_scene_task(scene_dir)
        input_path, replay_input = planning_input(scene_dir, saved_intent, args.detections)
        if args.sorting_session:
            from task_intent_v2 import class_routing_rules
            if class_routing_rules(saved_intent) is None:
                parser.error('SORTING_REQUIRES_CLASS_ROUTING: the saved task has no class routing table')
            if saved_intent.get('safety', {}).get('execution_backend', 'fake') != 'fake':
                from simulator_world import physics_task_qualification
                verdict = physics_task_qualification(saved_intent, sorting_session=True)
                parser.error(f"{verdict['code'] or 'SORTING_BACKEND_UNSUPPORTED'}: "
                             + '; '.join(verdict['problems'] or ['this runner sorts on MoveIt fake hardware only']))
    else:
        input_path, replay_input = args.detections or root/'config/runtime/r1_4_replay.yaml', True

    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out/'acceptance.json').exists():
        parser.error('choose a fresh evidence directory')
    summary_path = out / 'executor.json'
    summary_path.unlink(missing_ok=True)
    rclpy.init()
    def interrupted(signum, frame):
        raise InterruptedError(f'acceptance interrupted by signal {signum}')
    signal.signal(signal.SIGTERM, interrupted)
    node = rclpy.create_node('r14_plan_only_acceptance')
    goals = set()
    statuses = {}
    def observe(msg, topic):
        for status in msg.status_list:
            key = (topic, bytes(status.goal_info.goal_id.uuid).hex())
            goals.add(key)
            statuses[key] = status.status
    qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = [node.create_subscription(GoalStatusArray, topic + '/_action/status',
        lambda msg, topic=topic: observe(msg, topic), qos)
        for topic in ['/execute_trajectory', '/ur5_arm_controller/follow_joint_trajectory',
                      '/ur5_gripper_controller/follow_joint_trajectory']]
    installed_scripts = Path(get_package_prefix('workcell_builder'))/'lib/workcell_builder'
    executor_path = installed_scripts/'perceived_object_grasp_execute.py'
    launch = executor = None
    audit = dict(result='FAIL', execution_action_goals=[], shutdown_clean=False,
                 scene_share=get_package_share_directory(scene_name), executor=str(executor_path),
                 mode=('R2.4 sorting session (fake hardware)' if args.sorting_session else
                       'R1.5 execution' if args.execute else 'R1.4 plan-only'), domain_id=args.domain_id,
                 source_commit=subprocess.check_output(['git','rev-parse','HEAD'], cwd=root, text=True).strip(),
                 source_dirty=bool(subprocess.check_output(['git','status','--porcelain'], cwd=root, text=True)),
                 executor_hashes={}, task_resolution_mode='resolve' if args.resolve_task else 'consume' if uses_authored_task else 'legacy')
    deadline = time.monotonic() + args.timeout
    started_at = time.monotonic()
    log_offset = 0
    last_stage = [None]
    def stream_stages():
        nonlocal log_offset
        if not (out/'executor.log').exists() or (not args.stream_status and not args.sorting_session):
            return
        with (out/'executor.log').open() as log:
            log.seek(log_offset)
            while True:
                line = log.readline()
                if not line.endswith('\n'):
                    break
                log_offset = log.tell()
                try:
                    event = json.loads(line)
                except ValueError:
                    continue
                if isinstance(event, dict) and set(event) == {'stage'}:
                    last_stage[0] = event['stage']
                    if args.stream_status:
                        print(json.dumps(event), flush=True)
    def spin_until(predicate):
        while not predicate():
            stream_stages()
            if time.monotonic() >= deadline:
                raise TimeoutError('launch/planning deadline exceeded')
            if launch.poll() is not None:
                raise RuntimeError('scene launch exited unexpectedly')
            rclpy.spin_once(node, timeout_sec=.1)
    try:
        # Refuse to mix this owned run with an existing graph on the selected domain.
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=.1)
        if any(name != node.get_name() for name, _ in node.get_node_names_and_namespaces()):
            raise RuntimeError('ROS domain is already occupied; retry with an unused domain')
        for name in ['perceived_object_grasp_execute.py', 'perceived_object_grasp_plan.py', 'runtime_pick_inputs.py', 'task_intent_resolver.py', 'task_intent_v2.py', 'full_cycle_preplanner.py', 'physical_destination.py', 'grasp_strategy_candidates.py']:
            if (installed_scripts/name).read_bytes() != (root/'scripts'/name).read_bytes():
                raise RuntimeError(f'installed executor differs from this checkout: {name}')
            audit['executor_hashes'][name] = hashlib.sha256((installed_scripts/name).read_bytes()).hexdigest()
        # Do not accidentally accept a different installed/dirty scene overlay.
        audit['scene_hashes'] = {}
        for relative in ['environment.yaml', 'cell_definition.yaml', 'config/moveit_collision_objects.yaml']:
            installed = (Path(audit['scene_share'])/relative).read_bytes()
            if installed != (scene_dir/relative).read_bytes():
                raise RuntimeError(f'installed scene differs from this checkout: {relative}')
            audit['scene_hashes'][relative] = hashlib.sha256(installed).hexdigest()
        audit['moveit_teardown'], overlay = moveit_teardown_dependency()
        with (out/'launch.log').open('w') as log, (out/'executor.log').open('w') as elog:
            if args.stream_status:
                print(json.dumps(dict(stage='LAUNCHING_FAKE_HARDWARE')), flush=True)
            launch = subprocess.Popen(['ros2', 'launch', scene_name, 'demo.launch.py',
                'use_fake_hardware:=true', 'allow_trajectory_execution:='+str(args.execute or args.sorting_session).lower(),
                'launch_rviz:='+str(args.launch_rviz).lower()],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            spin_until(lambda: 'authored collision objects; MoveIt is planning truth' in (out/'launch.log').read_text())
            if overlay is not None:
                audit['moveit_teardown'].update(owned_move_group_overlay(launch.pid, overlay))
            if args.sorting_session:
                executor = run_sorting_session(args, out, scene_dir, input_path, replay_input, saved_intent, executor_path,
                                               elog, statuses, spin_until, stream_stages, last_stage, audit, node, rclpy)
                raise _SessionDone()
            executor = subprocess.Popen([sys.executable, str(executor_path),
                '--scene-package', str(scene_dir),
                '--detections', str(input_path),
                '--timeout', str(min(3600 if args.sequence_object_ids else 300, max(1, deadline-time.monotonic()-5))),
                '--summary-output', str(summary_path)]
                + (['--replay'] if replay_input else [])
                + (['--resolve-task'] if args.resolve_task else [] if uses_authored_task else ['--task-request', str(root/'config/runtime/r1_4b_task.yaml')])
                + (['--start'] if args.execute else [])
                + (['--target-object-id', args.target_object_id] if args.target_object_id is not None else [])
                + (['--sequence-object-ids', args.sequence_object_ids] if args.sequence_object_ids is not None else [])
                + (['--sequence-placements', args.sequence_placements] if args.sequence_placements is not None else []),
                stdout=elog, stderr=subprocess.STDOUT, start_new_session=True)
            spin_until(lambda: executor.poll() is not None)
            stream_stages()
            # Drain status messages published immediately before the executor exited.
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=.1)
            result = json.loads(summary_path.read_text())
            if args.execute:
                check_execution_result(result, statuses)
            elif args.sequence_object_ids is not None:
                check_sequence_result(result, goals)
            else:
                check_result(result, goals)
            if executor.returncode:
                raise RuntimeError(f'executor exited {executor.returncode}')
            audit.update(result)
            if args.sequence_object_ids is not None:
                audit.update(result='PASS', execution_attempted=False)
            else:
                audit.update(result='PASS', selected_object_id=result['selected_object_id'],
                             selected_grasp_index=result['selected_grasp_index'], planned_stages=STAGES,
                             full_cycle_prevalidated=True, execution_attempted=args.execute)
    except _SessionDone:
        pass
    except (Exception, KeyboardInterrupt) as exc:
        if not args.sorting_session and summary_path.exists():
            audit.update(json.loads(summary_path.read_text()))
        audit['result'] = 'FAIL'
        audit['failure'] = str(exc)
    finally:
        if args.stream_status:
            print(json.dumps(dict(stage='STOPPING_SIMULATION')), flush=True)
        executor_clean = stop(executor)
        if audit.get('result') != 'PASS' and summary_path.exists() and not args.sorting_session:
            failure = audit.get('failure')
            audit.update(json.loads(summary_path.read_text()))
            audit.update(result='FAIL', orchestration_failure=failure)
        launch_clean = stop(launch)
        log_text = (out/'launch.log').read_text() if (out/'launch.log').exists() else ''
        crashes = process_failures(log_text)
        exits = process_exits(log_text)
        stack_failures = shutdown_failures(exits)
        remaining_groups = [p.pid for p in (executor, launch) if p and group_alive(p.pid)]
        # Every owned launch child's exit status is reported. A planning-stack crash
        # (e.g. MoveGroup -11 in unpatched MoveIt teardown) is an unclean shutdown even
        # when both supervisors exit 0; an RViz abort is reported separately.
        audit.update(shutdown_clean=(executor_clean and launch_clean and not remaining_groups
                                     and not stack_failures),
                     process_exits=exits,
                     visualization_shutdown_clean=not [
                         e for e in exits if e['process'] in VISUALIZATION_PROCESSES
                         and e['exit_code'] not in (0, -signal.SIGINT, -signal.SIGTERM)],
                     shutdown_crashes=crashes, execution_action_goals=sorted(goals),
                     action_terminal_statuses=[dict(topic=t, goal_id=g, status=s) for (t,g),s in sorted(statuses.items())],
                     remaining_owned_process_groups=remaining_groups)
        audit['runtime_seconds'] = time.monotonic() - started_at
        audit['owned_processes'] = {name: dict(pid=p.pid, returncode=p.returncode)
                                   for name,p in [('executor',executor),('launch',launch)] if p}
        if not audit['shutdown_clean'] or (goals and not (args.execute or args.sorting_session)):
            if audit['result'] == 'PASS' and stack_failures:
                crashed = ', '.join(f"{e['process']} exit {e['exit_code']}" for e in stack_failures)
                hint = (' (installed MoveIt teardown defect; source the qualified MoveIt teardown overlay)'
                        if audit.get('moveit_teardown', {}).get('status') == 'INSTALLED_UNPATCHED'
                        and any(e['process'] == 'move_group' for e in stack_failures) else '')
                audit['failure'] = f'SHUTDOWN_UNCLEAN: {crashed} after the owned stop{hint}'
            audit['result'] = 'FAIL'
        node.destroy_node()
        rclpy.try_shutdown()
        (out/'acceptance.json').write_text(json.dumps(audit, indent=2)+'\n')
        if args.stream_status:
            print(json.dumps(dict(event='cycle_result', summary=audit)), flush=True)
        else:
            print(json.dumps(audit, indent=2))
    return 0 if audit['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
