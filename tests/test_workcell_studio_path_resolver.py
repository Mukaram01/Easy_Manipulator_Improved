from pathlib import Path
from scripts.workcell_studio_path_resolver import *

def test_resolver_repo_root_from_scripts():
    repo = resolve_repo_root(start=Path(__file__).resolve().parents[1] / 'scripts')
    assert (repo / 'scripts').exists()

def test_infer_workspace_root_custom_name(tmp_path):
    repo = tmp_path / 'custom_robot_ws' / 'src' / 'easy_manipulation_deployment'
    repo.mkdir(parents=True)
    ws = resolve_workspace_root(repo)
    assert ws.name == 'custom_robot_ws'

def test_missing_executable_reports_searched_paths(tmp_path, monkeypatch):
    ws = tmp_path / 'robot_ws'; ws.mkdir()
    monkeypatch.delenv('WORKCELL_BUILDER_EXECUTABLE', raising=False)
    monkeypatch.setattr('scripts.workcell_studio_path_resolver.shutil.which', lambda name: None)

    exe = resolve_workcell_builder_executable(ws)

    assert exe is None
    assert describe_resolution().get('searched_executable_paths')

def test_invalid_explicit_executable_reports_only_explicit_path(tmp_path):
    ws = tmp_path / 'robot_ws'
    ws.mkdir()
    missing = tmp_path / 'missing_workcell_builder'

    assert resolve_workcell_builder_executable(ws, explicit_executable=missing) is None
    assert describe_resolution().get('searched_executable_paths') == [str(missing.resolve())]

    not_executable = tmp_path / 'workcell_builder'
    not_executable.write_text('#!/bin/sh\nexit 0\n', encoding='utf-8')
    not_executable.chmod(0o644)

    assert resolve_workcell_builder_executable(ws, explicit_executable=not_executable) is None
    assert describe_resolution().get('searched_executable_paths') == [str(not_executable.resolve())]

def test_workcell_builder_candidate_order_prefers_env_then_path_then_install_paths(tmp_path, monkeypatch):
    ws = tmp_path / 'robot_ws'
    env_exe = tmp_path / 'custom_workcell_builder'
    path_exe = tmp_path / 'path_workcell_builder'
    monkeypatch.setenv('WORKCELL_BUILDER_EXECUTABLE', str(env_exe))

    def fake_which(name):
        if name == 'workcell_builder':
            return str(path_exe)
        return None

    monkeypatch.setattr('scripts.workcell_studio_path_resolver.shutil.which', fake_which)

    candidates = workcell_builder_executable_candidates(ws)

    assert candidates[:7] == [
        env_exe,
        path_exe,
        ws / 'install' / 'workcell_builder' / 'bin' / 'workcell_builder',
        ws / 'install' / 'workcell_builder' / 'lib' / 'workcell_builder' / 'workcell_builder',
        ws / 'install' / 'bin' / 'workcell_builder',
        Path('/home/user/workcell_ws/install/workcell_builder/bin/workcell_builder'),
        Path('/home/user/workcell_ws/install/workcell_builder/lib/workcell_builder/workcell_builder'),
    ]


def test_workcell_builder_candidates_include_ros2_prefix_paths(tmp_path, monkeypatch):
    ws = tmp_path / 'robot_ws'
    prefix = tmp_path / 'install' / 'workcell_builder'

    def fake_which(name):
        if name == 'workcell_builder':
            return None
        if name == 'ros2':
            return '/usr/bin/ros2'
        return None

    class Result:
        returncode = 0
        stdout = f'{prefix}\n'

    monkeypatch.delenv('WORKCELL_BUILDER_EXECUTABLE', raising=False)
    monkeypatch.setattr('scripts.workcell_studio_path_resolver.shutil.which', fake_which)
    monkeypatch.setattr('scripts.workcell_studio_path_resolver.subprocess.run', lambda *args, **kwargs: Result())

    candidates = workcell_builder_executable_candidates(ws)

    assert candidates[-3:] == [
        prefix / 'lib' / 'workcell_builder' / 'workcell_builder',
        prefix / 'bin' / 'workcell_builder',
        prefix / '..' / 'bin' / 'workcell_builder',
    ]


def test_resolve_records_candidate_metadata_for_env_override(tmp_path, monkeypatch):
    ws = tmp_path / 'robot_ws'
    env_exe = tmp_path / 'custom_workcell_builder'
    env_exe.write_text('#!/bin/sh\nexit 0\n', encoding='utf-8')
    env_exe.chmod(0o755)
    monkeypatch.setenv('WORKCELL_BUILDER_EXECUTABLE', str(env_exe))
    monkeypatch.setattr('scripts.workcell_studio_path_resolver.shutil.which', lambda name: None)

    assert resolve_workcell_builder_executable(ws) == env_exe

    evidence = describe_resolution().get('workcell_builder_executable_candidates')
    assert evidence[0] == {
        'path': str(env_exe),
        'exists': True,
        'is_file': True,
        'executable': True,
        'selected': True,
    }
    assert len(evidence) == 6
