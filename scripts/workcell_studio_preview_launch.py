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

def build_preview_commands(scene_name:str)->dict[str,str]:
    return {
        'build': f'cd ~/workcell_ws\ncolcon build --symlink-install --packages-select {scene_name}',
        'source': 'source install/setup.bash',
        'launch': f'ros2 launch {scene_name} demo.launch.py use_fake_hardware:=true launch_rviz:=true launch_task_preview:=false',
    }

def command_is_safe(command:str)->tuple[bool,list[str]]:
    blockers=[]
    if 'use_fake_hardware:=true' not in command:
        blockers.append('Missing required use_fake_hardware:=true')
    for token in DENYLIST:
        if token in command:
            blockers.append(f'Unsafe launch argument detected: {token}')
    return (len(blockers)==0, blockers)

def can_run_preview(status:str, is_preview_only:bool)->tuple[bool,list[str]]:
    blockers=[]
    if status not in {'PASS','READY','WARNINGS'}:
        blockers.append('acceptance must be PASS or WARNINGS')
    if is_preview_only:
        blockers.append('PREVIEW_ONLY placeholder scenes cannot be launched')
    return (len(blockers)==0, blockers)

def write_preview_launch_artifacts(scene_dir:Path, scene_name:str, command:str, run:bool, exit_code:int|None=None)->None:
    out = scene_dir / 'preview_launch'
    out.mkdir(parents=True, exist_ok=True)
    now=datetime.now(timezone.utc).isoformat()
    payload={
        'scene_name': scene_name,
        'command': command,
        'started_at': now if run else None,
        'finished_at': now if run else None,
        'exit_code': exit_code,
        'safety_flags': {
            'fake_hardware_first': True,
            'runtime_execution_enabled': False,
            'motion_command_sent': False,
        },
        'no robot motion commanded': True,
        'copied_only': not run,
    }
    (out / 'preview_launch_session.json').write_text(json.dumps(payload, indent=2)+'\n', encoding='utf-8')
    (out / 'preview_launch_summary.txt').write_text(
        f"scene_name={scene_name}\ncommand={command}\nno robot motion commanded\ncopied_only={str(not run).lower()}\n",
        encoding='utf-8'
    )
