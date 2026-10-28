"""Engineering viewers attach to a running runtime and own only themselves (real POSIX processes, no ROS)."""
import json
import os
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import workcell_viewer as wv  # noqa: E402

# A "viewer" that leaves a grandchild in its group, like ros2 run -> rviz2.
VIEWER = [sys.executable, '-c', 'import subprocess,time; subprocess.Popen(["sleep","60"]); time.sleep(60)']


def sleeper(*argv, env=None):
    return subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)', *argv], env=env, start_new_session=True)


def alive(pid):
    try:
        os.kill(pid, 0)
        return Path(f'/proc/{pid}/stat').read_text().split()[2] != 'Z'
    except (ProcessLookupError, FileNotFoundError):
        return False


def group_members(pgid):
    return [p for p in Path('/proc').iterdir() if p.name.isdigit() and _pgid(p) == pgid]


def _pgid(p):
    try:
        return int(p.joinpath('stat').read_text().rsplit(')', 1)[1].split()[2])
    except (OSError, IndexError):
        return None


def run_supervisor(target_pid, tmp_path):
    """The helper's supervise() in its own process, attached to a target process (like Studio runs it)."""
    code = (f'import sys,os; sys.path.insert(0,{str(ROOT / "scripts")!r}); import workcell_viewer as wv; '
            f'ticks = wv.process_info({target_pid})["start_ticks"]; '
            f'sys.exit(wv.supervise("rviz", {VIEWER!r}, dict(os.environ), lambda: wv.same_process({target_pid}, ticks), '
            f'poll_s=0.1))')
    return subprocess.Popen([sys.executable, '-c', code], stdout=subprocess.PIPE, text=True)


def lines(proc):
    return [json.loads(line) for line in proc.stdout.read().splitlines() if line.startswith('{')]


def opened_pid(proc):
    return json.loads(proc.stdout.readline())['pid']


def test_closing_the_viewer_stops_only_its_own_group_never_the_runtime(tmp_path):
    runtime = sleeper()
    try:
        helper = run_supervisor(runtime.pid, tmp_path)
        viewer = opened_pid(helper)
        time.sleep(0.5)
        assert len(group_members(viewer)) >= 2                       # the viewer and its own child
        helper.send_signal(signal.SIGTERM)                           # Studio: Close visualization
        assert helper.wait(15) == 0
        closed = lines(helper)[-1]
        assert closed['reason'] == 'STOP_REQUESTED' and closed['clean'] and not group_members(viewer)
        assert alive(runtime.pid)                                    # the runtime is untouched
    finally:
        runtime.kill()


def test_a_viewer_closes_when_the_runtime_it_shows_ends(tmp_path):
    runtime = sleeper()
    helper = run_supervisor(runtime.pid, tmp_path)
    viewer = opened_pid(helper)
    runtime.kill(); runtime.wait()
    assert helper.wait(15) == 0
    closed = lines(helper)[-1]
    assert closed['reason'] == 'RUNTIME_ENDED' and not group_members(viewer)      # nothing stale stays on screen


def test_a_viewer_its_user_closes_is_reported_with_its_exit_code(tmp_path):
    runtime = sleeper()
    try:
        helper = run_supervisor(runtime.pid, tmp_path)
        viewer = opened_pid(helper)
        os.killpg(viewer, signal.SIGKILL)                           # a viewer that dies is not "clean"
        assert helper.wait(15) == 1
        closed = lines(helper)[-1]
        assert closed['reason'] == 'VIEWER_CLOSED' and closed['viewer_exit_code'] == -signal.SIGKILL and not closed['clean']
        assert alive(runtime.pid)
    finally:
        runtime.kill()


def test_gazebo_attaches_only_to_the_receipt_bound_live_server(tmp_path):
    with pytest.raises(wv.Blocked, match='NO_ACTIVE_SIMULATION'):
        wv.live_receipt(tmp_path)
    env = dict(os.environ, IGN_PARTITION='viewer_test', ROS_DOMAIN_ID='231')
    server = sleeper('gazebo', '-s', '-r', 'world.sdf', env=env)
    try:
        time.sleep(0.2)
        receipt = tmp_path / '06-full-cycle/runtime/receipt.json'
        receipt.parent.mkdir(parents=True)
        record = dict(pid=server.pid, start_ticks=wv.process_info(server.pid)['start_ticks'], partition='viewer_test',
                      domain='231', world='a0')
        receipt.write_text(json.dumps(record))
        path, r, _ = wv.live_receipt(tmp_path)
        assert path == receipt and r['world'] == 'a0'
        assert wv.session_env(wv.process_info(server.pid)['environ'])['IGN_PARTITION'] == 'viewer_test'
        receipt.write_text(json.dumps(dict(record, start_ticks='1')))          # a reused pid is not the server
        with pytest.raises(wv.Blocked, match='NO_ACTIVE_SIMULATION'):
            wv.live_receipt(tmp_path)
        receipt.write_text(json.dumps(dict(record, partition='other')))       # another partition's world
        with pytest.raises(wv.Blocked, match='NO_ACTIVE_SIMULATION'):
            wv.live_receipt(tmp_path)
    finally:
        server.kill()


def test_rviz_needs_a_live_move_group_and_never_starts_one():
    with pytest.raises(wv.Blocked, match='NO_ACTIVE_MOVEIT'):
        wv.find_move_group(232)
    assert wv.main(['rviz', '--domain', '232', '--rviz-config', str(ROOT / 'scenes/ur5_2f_test/launch/demo.rviz')]) == 2
    source = (ROOT / 'scripts/workcell_viewer.py').read_text()
    assert 'ros2 launch' not in source and "'launch'" not in source and 'move_group' in source


def test_rviz_loads_the_sessions_description_and_joint_states():
    tree = wv.nested({'robot_description': 'u', 'robot_description_kinematics.manipulator.kinematics_solver': 'k'})
    assert tree == {'robot_description': 'u', 'robot_description_kinematics': {'manipulator': {'kinematics_solver': 'k'}}}
    command = wv.rviz_command('c.rviz', 'p.yaml', ['/physical_joint_states'])
    assert command[:5] == ['ros2', 'run', 'rviz2', 'rviz2', '-d'] and 'joint_states:=/physical_joint_states' in command
    assert 'joint_states:=' not in ' '.join(wv.rviz_command('c.rviz', 'p.yaml', ['/a', '/b']))   # ambiguous: no guess


def test_the_gazebo_viewer_configuration_is_read_only():
    text = (ROOT / 'config/simulation/workcell_physics_viewer.config').read_text()
    root = ET.fromstring('<config>' + text.split('?>', 1)[1] + '</config>')
    assert {p.get('filename') for p in root.iter('plugin')} == {'MinimalScene', 'GzSceneManager', 'InteractiveViewControl', 'WorldStats'}
    menus = root.find('window/menus')
    assert menus.find('drawer').get('visible') == 'false' and menus.find('plugins').get('visible') == 'false'
    assert wv.gazebo_command('v.config') == ['ign', 'gazebo', '-g', '--gui-config', 'v.config']
