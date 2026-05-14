from scripts.workcell_studio_preview_launch import command_is_safe, can_run_preview

def test_unsafe_args_rejected():
    for token in ['use_fake_hardware:=false', 'runtime_execution_enabled:=true', 'execute:=true']:
        ok, blockers = command_is_safe(f'ros2 launch pkg demo.launch.py {token}')
        assert not ok
        assert blockers

def test_preview_only_cannot_launch():
    ok, blockers = can_run_preview('WARNINGS', True)
    assert not ok
    assert any('PREVIEW_ONLY' in b for b in blockers)

def test_blocked_cannot_launch():
    ok, _ = can_run_preview('BLOCKED', False)
    assert not ok

def test_pass_or_warnings_can_launch():
    assert can_run_preview('PASS', False)[0]
    assert can_run_preview('WARNINGS', False)[0]
