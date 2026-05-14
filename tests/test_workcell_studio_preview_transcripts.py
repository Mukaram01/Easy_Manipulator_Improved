from pathlib import Path
from scripts.workcell_studio_preview_launch import build_preview_commands, write_preview_launch_artifacts, command_is_safe

def test_transcript_filenames_in_helper():
    text = Path('scripts/workcell_studio_preview_launch.py').read_text(encoding='utf-8')
    for token in ['build_session.json', 'build_summary.txt', 'preview_launch_session.json', 'preview_launch_summary.txt', 'latest_console.log']:
        assert token in text

def test_build_and_launch_commands_have_required_sources(tmp_path: Path):
    cmds = build_preview_commands('demo_scene', '/tmp/ws')
    assert 'source /opt/ros/humble/setup.bash' in cmds['build']
    assert 'source install/setup.bash' in cmds['launch']
    assert 'use_fake_hardware:=true' in cmds['launch']
    assert not command_is_safe(cmds['launch'].replace('use_fake_hardware:=true', 'use_fake_hardware:=false'))[0]
    write_preview_launch_artifacts(tmp_path, 'demo_scene', cmds['build'], run=True, exit_code=0, event='build_passed')
    assert (tmp_path / 'preview_launch' / 'build_session.json').is_file()
