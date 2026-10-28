#!/usr/bin/env python3
"""Engineering viewers for a running Workcell Studio runtime: RViz2 or the Gazebo Fortress GUI client.

A viewer only attaches. It never starts MoveIt, controllers or a simulator, never authorizes motion and
never changes the runtime it shows:

  rviz    --domain N | --cycle-output DIR, --rviz-config FILE
          RViz2 in the ROS domain of the runtime's single live move_group. Its robot description,
          semantic description, kinematics and use_sim_time are read from that move_group, and its joint
          states topic is the one move_group subscribes to, so it shows that session and nothing stale.
  gazebo  --cycle-output DIR, --gui-config FILE
          ``ign gazebo -g`` in the partition of the cycle's live, receipt-bound Fortress server (no second
          world: the server is identified by its receipt pid and start time). The GUI configuration is
          read-only (scene, view control, statistics): no play/pause, step, transform or entity editing.

The viewer runs in its own POSIX session/process group owned by this process. It is closed (SIGINT, then
SIGTERM, then SIGKILL, until the group is gone) when this process is asked to stop or when the runtime it
shows ends; closing it never signals the runtime. One JSON status line is printed per transition:
BLOCKED (nothing attachable, exit 2), OPEN, then CLOSED with the reason and the viewer's own exit code.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

POLL_S = 0.5
TOPIC_WAIT_S = 10.0
SIGINT_GRACE_S = 5.0
SIGTERM_GRACE_S = 2.0
# Read from the live move_group so RViz loads exactly the robot that session plans for.
MOVEIT_PARAMETER_PREFIXES = ('robot_description', 'robot_description_semantic', 'robot_description_kinematics',
                             'robot_description_planning')
# The runtime's own middleware selection; a viewer with a different one would not see it.
SESSION_ENVIRONMENT = ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION', 'IGN_PARTITION', 'IGN_IP')


class Blocked(RuntimeError):
    """Nothing attachable: the reason says what to start or wait for."""


def emit(**fields):
    print(json.dumps(fields), flush=True)


def process_info(pid):
    from simulator_backend import process_info as info
    return info(pid)


def same_process(pid, start_ticks):
    try:
        return process_info(pid)['start_ticks'] == start_ticks
    except (FileNotFoundError, ProcessLookupError, IndexError):
        return False


def live_receipt(cycle_output):
    """The cycle's one live simulator: its receipt pid still runs as the receipt's Fortress server."""
    live = []
    for receipt in sorted(Path(cycle_output).glob('*/runtime/receipt.json')):
        try:
            r = json.loads(receipt.read_text())
            p = process_info(r['pid'])
        except (OSError, ValueError, KeyError, IndexError):
            continue
        if (p['start_ticks'] == r.get('start_ticks') and 'gazebo -s -r ' in p['command'] and
                p['environ'].get('IGN_PARTITION') == r.get('partition') and p['environ'].get('ROS_DOMAIN_ID') == r.get('domain')):
            live.append((receipt, r, p))
    if not live:
        raise Blocked('NO_ACTIVE_SIMULATION: no running qualified simulator in this cycle yet; '
                      'open the viewer once the cycle reports simulator launch')
    if len(live) > 1:
        raise Blocked('AMBIGUOUS_SIMULATION: more than one live simulator in ' + str(cycle_output))
    return live[0]


def find_move_group(domain):
    """The single move_group process of a ROS domain (read-only /proc scan of this user's processes)."""
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv0 = (entry / 'cmdline').read_bytes().split(b'\0', 1)[0].decode()
            if Path(argv0).name != 'move_group':
                continue
            info = process_info(int(entry.name))
        except (OSError, IndexError, UnicodeDecodeError):
            continue
        if info['environ'].get('ROS_DOMAIN_ID', '0') == str(domain):
            found.append((int(entry.name), info))
    if not found:
        raise Blocked(f'NO_ACTIVE_MOVEIT: no move_group runs in ROS domain {domain}; start Plan Saved Task or a '
                      'physics cycle and open RViz while it runs')
    if len(found) > 1:
        raise Blocked(f'AMBIGUOUS_MOVEIT: {len(found)} move_group processes in ROS domain {domain}')
    return found[0]


def nested(flat):
    """Dotted ROS parameter names as the nested mapping a --params-file expects."""
    tree = {}
    for name, value in flat.items():
        node = tree
        *parents, leaf = name.split('.')
        for part in parents:
            node = node.setdefault(part, {})
        node[leaf] = value
    return tree


def move_group_topics(node, message_type, endpoints):
    """Topics of a type that move_group's own nodes use. Its planning scene and state monitors run on private
    nodes in the move_group process (move_group_private_*), so the owning node name is matched by prefix."""
    return sorted(topic for topic, types in node.get_topic_names_and_types() if message_type in types
                  and any(e.node_name == 'move_group' or e.node_name.startswith('move_group_private')
                          for e in endpoints(topic)))


def moveit_session(timeout=20.0):
    """Parameters and joint-state topic of the live move_group (the environment already selects its domain)."""
    import rclpy
    from rcl_interfaces.srv import GetParameters, ListParameters
    from rclpy.parameter import parameter_value_to_python
    rclpy.init()
    node = rclpy.create_node('workcell_viewer_probe')
    try:
        # A freshly started move_group answers services before every peer has discovered it: wait (bounded)
        # until it is in this participant's graph, so the topic lookups below see it.
        deadline = time.monotonic() + timeout
        while ('move_group', '/') not in node.get_node_names_and_namespaces():
            if time.monotonic() > deadline:
                raise Blocked('MOVEIT_NOT_READY: move_group is not discoverable yet; open RViz again in a moment')
            rclpy.spin_once(node, timeout_sec=0.2)
        def call(service, kind, request):
            client = node.create_client(kind, service)
            if not client.wait_for_service(timeout_sec=timeout):
                raise Blocked(f'MOVEIT_NOT_READY: {service} unavailable; open RViz once the session is planning')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if future.result() is None:
                raise Blocked(f'MOVEIT_NOT_READY: {service} did not answer')
            return future.result()
        names = list(call('/move_group/list_parameters', ListParameters,
                          ListParameters.Request(prefixes=list(MOVEIT_PARAMETER_PREFIXES), depth=0)).result.names)
        names.append('use_sim_time')
        values = call('/move_group/get_parameters', GetParameters, GetParameters.Request(names=names)).values
        parameters = {name: parameter_value_to_python(value) for name, value in zip(names, values)}
        if not parameters.get('robot_description') or not parameters.get('robot_description_semantic'):
            raise Blocked('MOVEIT_NOT_READY: move_group has no robot description yet')
        # move_group creates its joint-state subscription and monitored-scene publisher as its planning scene
        # monitor starts; wait (bounded) for them so RViz is wired to this session's own topics.
        deadline = time.monotonic() + TOPIC_WAIT_S
        while True:
            joint_states = move_group_topics(node, 'sensor_msgs/msg/JointState', node.get_subscriptions_info_by_topic)
            scenes = move_group_topics(node, 'moveit_msgs/msg/PlanningScene', node.get_publishers_info_by_topic)
            if (joint_states and scenes) or time.monotonic() > deadline:
                return parameters, joint_states, scenes
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def session_rviz_config(text, planning_scene_topics):
    """The cell's RViz configuration with its PlanningScene display on the topic this move_group publishes.

    Generated cells name /moveit_cpp/publish_planning_scene, which no Workcell Studio runtime publishes, so the
    robot and scene never appeared. Only that property changes, and only when move_group publishes one scene."""
    import yaml
    config = yaml.safe_load(text)
    if len(planning_scene_topics) != 1:
        return config
    for display in ((config.get('Visualization Manager') or {}).get('Displays') or []):
        if display.get('Class') == 'moveit_rviz_plugin/PlanningScene':
            display['Planning Scene Topic'] = planning_scene_topics[0]
    return config


def rviz_executable():
    from ament_index_python.packages import get_package_prefix
    return str(Path(get_package_prefix('rviz2')) / 'lib/rviz2/rviz2')


def rviz_command(config, params_file, joint_states, executable='rviz2'):
    # The executable itself (not `ros2 run`), so its own exit code or signal is what is reported.
    command = [executable, '-d', str(config), '--ros-args', '-r', '__node:=workcell_viewer_rviz',
               '--params-file', str(params_file)]
    if len(joint_states) == 1:
        command += ['-r', f'joint_states:={joint_states[0]}', '-r', f'/joint_states:={joint_states[0]}']
    return command


def gazebo_command(gui_config):
    return ['ign', 'gazebo', '-g', '--gui-config', str(gui_config)]


def group_alive(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def stop_group(child):
    """SIGINT, SIGTERM, SIGKILL to the viewer's own group only, until it is gone."""
    for sig, grace in ((signal.SIGINT, SIGINT_GRACE_S), (signal.SIGTERM, SIGTERM_GRACE_S), (signal.SIGKILL, 2.0)):
        if not group_alive(child.pid):
            break
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline and group_alive(child.pid):
            child.poll()
            time.sleep(0.05)
    child.poll()
    if child.returncode is None:
        child.wait()
    return child.returncode


def supervise(viewer, command, env, target_alive, poll_s=POLL_S, **opened):
    """Run the viewer in an owned group until it closes, the target ends or this process is asked to stop."""
    stop = {'requested': False}

    def request(_signum, _frame):
        stop['requested'] = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request)
    child = subprocess.Popen(command, env=env, start_new_session=True)
    emit(viewer=viewer, status='OPEN', pid=child.pid, **opened)
    reason = None
    while reason is None:
        if child.poll() is not None:
            reason = 'VIEWER_CLOSED'
        elif stop['requested']:
            reason = 'STOP_REQUESTED'
        elif not target_alive():
            reason = 'RUNTIME_ENDED'
        else:
            time.sleep(poll_s)
    code = stop_group(child) if reason != 'VIEWER_CLOSED' or group_alive(child.pid) else child.returncode
    # A viewer closed by its user exits 0; one stopped here may also report the signal it was stopped with.
    # Anything else (a crash on the way out included) is reported as unclean.
    clean = code == 0 or (reason != 'VIEWER_CLOSED' and code in (-signal.SIGINT, -signal.SIGTERM))
    emit(viewer=viewer, status='CLOSED', reason=reason, viewer_exit_code=code, clean=clean)
    return 0 if clean else 1


def session_env(environ, domain=None):
    env = dict(os.environ)
    for key in SESSION_ENVIRONMENT:
        env.pop(key, None)
        if key in environ:
            env[key] = environ[key]
    if domain is not None:
        env['ROS_DOMAIN_ID'] = str(domain)
    return env


def open_rviz(args):
    if args.cycle_output:
        _, receipt, _ = live_receipt(args.cycle_output)
        domain = receipt['domain']
    else:
        domain = args.domain
    pid, info = find_move_group(domain)
    env = session_env(info['environ'], domain)
    os.environ.update({k: v for k, v in env.items() if k in SESSION_ENVIRONMENT})
    parameters, joint_states, scenes = moveit_session()
    import yaml
    state = Path(args.state_dir)
    state.mkdir(parents=True, exist_ok=True)
    params_file, config_file = state / f'rviz_{domain}_{pid}.yaml', state / f'rviz_{domain}_{pid}.rviz'
    params_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': nested(parameters)}}))
    config_file.write_text(yaml.safe_dump(session_rviz_config(Path(args.rviz_config).read_text(), scenes), sort_keys=False))
    start_ticks = info['start_ticks']
    return supervise('rviz', rviz_command(config_file, params_file, joint_states, rviz_executable()), env,
                     lambda: same_process(pid, start_ticks), domain=str(domain), move_group_pid=pid,
                     move_group_executable=info['command'].split(' ', 1)[0], use_sim_time=bool(parameters.get('use_sim_time')),
                     joint_states=joint_states, planning_scene=scenes)


def open_gazebo(args):
    receipt_path, receipt, server = live_receipt(args.cycle_output)
    env = session_env(server['environ'])
    pid, start_ticks = receipt['pid'], receipt['start_ticks']
    return supervise('gazebo', gazebo_command(args.gui_config), env, lambda: same_process(pid, start_ticks),
                     world=receipt.get('world'), partition=receipt['partition'], domain=receipt['domain'],
                     server_pid=pid, receipt=str(receipt_path))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='viewer', required=True)
    rviz = sub.add_parser('rviz')
    target = rviz.add_mutually_exclusive_group(required=True)
    target.add_argument('--domain', type=int)
    target.add_argument('--cycle-output', type=Path)
    rviz.add_argument('--rviz-config', type=Path, required=True)
    rviz.add_argument('--state-dir', type=Path, default=Path(os.environ.get('XDG_RUNTIME_DIR') or '/tmp') / 'workcell_viewer')
    gazebo = sub.add_parser('gazebo')
    gazebo.add_argument('--cycle-output', type=Path, required=True)
    gazebo.add_argument('--gui-config', type=Path, default=SCRIPTS.parent / 'config/simulation/workcell_physics_viewer.config')
    args = parser.parse_args(argv)
    try:
        for path in ([args.rviz_config] if args.viewer == 'rviz' else [args.gui_config]):
            if not Path(path).is_file():
                raise Blocked(f'VIEWER_CONFIG_MISSING: {path}')
        return open_rviz(args) if args.viewer == 'rviz' else open_gazebo(args)
    except Blocked as exc:
        emit(viewer=args.viewer, status='BLOCKED', reason=str(exc))
        return 2
    except Exception as exc:                  # never fail silently: the reason reaches the Studio status line
        emit(viewer=args.viewer, status='BLOCKED', reason=f'VIEWER_FAILED: {type(exc).__name__}: {exc}')
        return 3


if __name__ == '__main__':
    sys.exit(main())
