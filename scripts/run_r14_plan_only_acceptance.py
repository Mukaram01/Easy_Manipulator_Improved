#!/usr/bin/env python3
"""Owned replay acceptance: R1.4 plan-only, or explicit R1.5 --execute on mock hardware."""
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=120, help='Total launch/planning budget; cleanup adds at most 16 seconds per process')
    parser.add_argument('--domain-id', type=int, default=179, help='An unused, isolated ROS domain')
    parser.add_argument('--execute', action='store_true', help='R1.5: execute the complete cycle on exclusively mock hardware')
    args = parser.parse_args()
    if not 0 < args.timeout <= 900 or not 0 <= args.domain_id <= 232:
        parser.error('timeout must be in (0,900]; domain-id in [0,232]')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1')
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    root = Path(__file__).resolve().parents[1]
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
                 scene_share=get_package_share_directory('ur5_2f_test'), executor=str(executor_path),
                 mode='R1.5 execution' if args.execute else 'R1.4 plan-only', domain_id=args.domain_id,
                 source_commit=subprocess.check_output(['git','rev-parse','HEAD'], cwd=root, text=True).strip(),
                 source_dirty=bool(subprocess.check_output(['git','status','--porcelain'], cwd=root, text=True)),
                 executor_hashes={})
    deadline = time.monotonic() + args.timeout
    def spin_until(predicate):
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError('launch/planning deadline exceeded')
            if launch.poll() is not None:
                raise RuntimeError('scene launch exited unexpectedly')
            rclpy.spin_once(node, timeout_sec=.1)
    try:
        for name in ['perceived_object_grasp_execute.py', 'perceived_object_grasp_plan.py', 'runtime_pick_inputs.py']:
            if (installed_scripts/name).read_bytes() != (root/'scripts'/name).read_bytes():
                raise RuntimeError(f'installed executor differs from this checkout: {name}')
            audit['executor_hashes'][name] = hashlib.sha256((installed_scripts/name).read_bytes()).hexdigest()
        # Do not accidentally accept a different installed/dirty scene overlay.
        audit['scene_hashes'] = {}
        for relative in ['environment.yaml', 'cell_definition.yaml', 'config/moveit_collision_objects.yaml']:
            installed = (Path(audit['scene_share'])/relative).read_bytes()
            if installed != (root/'scenes/ur5_2f_test'/relative).read_bytes():
                raise RuntimeError(f'installed scene differs from this checkout: {relative}')
            audit['scene_hashes'][relative] = hashlib.sha256(installed).hexdigest()
        with (out/'launch.log').open('w') as log, (out/'executor.log').open('w') as elog:
            launch = subprocess.Popen(['ros2', 'launch', 'ur5_2f_test', 'demo.launch.py',
                'use_fake_hardware:=true', 'allow_trajectory_execution:='+str(args.execute).lower(), 'launch_rviz:=false'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            spin_until(lambda: 'authored collision objects; MoveIt is planning truth' in (out/'launch.log').read_text())
            executor = subprocess.Popen([sys.executable, str(executor_path),
                '--scene-package', 'ur5_2f_test', '--task-request', str(root/'config/runtime/r1_4b_task.yaml'),
                '--detections', str(root/'config/runtime/r1_4_replay.yaml'), '--replay',
                '--timeout', str(min(300, max(1, deadline-time.monotonic()-5))), '--summary-output', str(summary_path)]
                + (['--start'] if args.execute else []),
                stdout=elog, stderr=subprocess.STDOUT, start_new_session=True)
            spin_until(lambda: executor.poll() is not None)
            # Drain status messages published immediately before the executor exited.
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=.1)
            result = json.loads(summary_path.read_text())
            if args.execute:
                check_execution_result(result, statuses)
            else:
                check_result(result, goals)
            if executor.returncode:
                raise RuntimeError(f'executor exited {executor.returncode}')
            audit.update(result)
            audit.update(result='PASS', selected_object_id=result['selected_object_id'],
                         selected_grasp_index=result['selected_grasp_index'], planned_stages=STAGES,
                         full_cycle_prevalidated=True, execution_attempted=args.execute)
    except (Exception, KeyboardInterrupt) as exc:
        audit['failure'] = str(exc)
    finally:
        executor_clean = stop(executor)
        launch_clean = stop(launch)
        log_text = (out/'launch.log').read_text() if (out/'launch.log').exists() else ''
        crashes = process_failures(log_text)
        audit.update(shutdown_clean=executor_clean and launch_clean and not crashes,
                     shutdown_crashes=crashes, execution_action_goals=sorted(goals),
                     action_terminal_statuses=[dict(topic=t, goal_id=g, status=s) for (t,g),s in sorted(statuses.items())],
                     remaining_owned_process_groups=[p.pid for p in (executor,launch) if p and group_alive(p.pid)])
        if not audit['shutdown_clean'] or (goals and not args.execute):
            audit['result'] = 'FAIL'
        node.destroy_node()
        rclpy.try_shutdown()
        (out/'acceptance.json').write_text(json.dumps(audit, indent=2)+'\n')
        print(json.dumps(audit, indent=2))
    return 0 if audit['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
