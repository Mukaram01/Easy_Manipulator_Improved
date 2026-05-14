#!/usr/bin/env python3
from __future__ import annotations
import json
from datetime import datetime, timezone
from pathlib import Path

DENYLIST = [
    'use_fake_hardware:=false',
    'real_hardware:=true',
    'runtime_execution_enabled:=true',
    'execute:=true',
    'command_robot:=true',
    'send_motion:=true',
]

def detect_workspace_root(scene_dir: Path | None = None) -> str:
    cwd = Path.cwd()
    if (cwd / 'src').is_dir():
        return str(cwd)
    if scene_dir:
        for parent in [scene_dir, *scene_dir.parents]:
            if (parent / 'src').is_dir():
                return str(parent)
    return ''

def build_preview_commands(scene_name: str, workspace_root: str = '') -> dict[str, str]:
    ws = workspace_root or '<workspace_root>'
    return {
        'build': f'cd {ws} && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select {scene_name}',
        'source': f'cd {ws} && source install/setup.bash',
        'launch': f'cd {ws} && source install/setup.bash && ros2 launch {scene_name} demo.launch.py use_fake_hardware:=true launch_rviz:=true launch_task_preview:=false',
    }

def command_is_safe(command: str) -> tuple[bool, list[str]]:
    blockers = []
    if 'use_fake_hardware:=true' not in command:
        blockers.append('Missing required use_fake_hardware:=true')
    for token in DENYLIST:
        if token in command:
            blockers.append(f'Unsafe launch argument detected: {token}')
    return (len(blockers) == 0, blockers)

def can_run_preview(status: str, is_preview_only: bool) -> tuple[bool, list[str]]:
    blockers = []
    if status not in {'PASS', 'READY', 'WARNINGS'}:
        blockers.append('acceptance must be PASS or WARNINGS')
    if is_preview_only:
        blockers.append('PREVIEW_ONLY placeholder scenes cannot be launched')
    return (len(blockers) == 0, blockers)

def write_preview_launch_artifacts(scene_dir: Path, scene_name: str, command: str, run: bool, exit_code: int | None = None, event: str='copy_only') -> None:
    out = scene_dir / 'preview_launch'
    out.mkdir(parents=True, exist_ok=True)
    now = datetime.now(timezone.utc).isoformat()
    payload = {
        'scene_name': scene_name,
        'command': command,
        'started_at': now if run else None,
        'finished_at': now,
        'exit_code': exit_code,
        'status': event,
        'safety_flags': {'fake_hardware_only': True, 'runtime_execution_enabled': False},
        'no_robot_motion_commanded': True,
        'copied_only': not run,
    }
    is_build = 'colcon build' in command
    (out / ('build_session.json' if is_build else 'preview_launch_session.json')).write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')
    (out / ('build_summary.txt' if is_build else 'preview_launch_summary.txt')).write_text(
        f"scene_name={scene_name}\ncommand={command}\nstatus={event}\nno robot motion commanded\nno_robot_motion_commanded=true\ncopied_only={str(not run).lower()}\n",
        encoding='utf-8',
    )
    (out / 'latest_console.log').touch()

def emit_preview_json(scene_name: str, scene_dir: Path) -> str:
    ws = detect_workspace_root(scene_dir)
    commands = build_preview_commands(scene_name, ws)
    safe, blockers = command_is_safe(commands['launch'])
    return json.dumps({'workspace_root': ws, 'commands': commands, 'safe': safe, 'blockers': blockers})
