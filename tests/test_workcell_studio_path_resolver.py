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

def test_missing_executable_reports_searched_paths(tmp_path):
    ws = tmp_path / 'robot_ws'; ws.mkdir()
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
