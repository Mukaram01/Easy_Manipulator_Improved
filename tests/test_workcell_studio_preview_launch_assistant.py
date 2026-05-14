from pathlib import Path
from scripts.workcell_studio_preview_launch import build_preview_commands, write_preview_launch_artifacts

def test_launch_command_includes_fake_hardware():
    cmd = build_preview_commands('demo_scene')['launch']
    assert 'use_fake_hardware:=true' in cmd

def test_artifact_files_created(tmp_path: Path):
    write_preview_launch_artifacts(tmp_path, 'demo_scene', 'ros2 launch demo_scene demo.launch.py use_fake_hardware:=true', run=False)
    session = tmp_path / 'preview_launch' / 'preview_launch_session.json'
    summary = tmp_path / 'preview_launch' / 'preview_launch_summary.txt'
    assert session.is_file()
    assert summary.is_file()
    assert 'no robot motion commanded' in summary.read_text(encoding='utf-8')
