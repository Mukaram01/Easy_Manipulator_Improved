from pathlib import Path


def _read(path: str) -> str:
    return Path(path).read_text(encoding='utf-8')


def test_theme_helpers_and_roles_exist():
    hpp = _read('workcell_builder/workcell_builder/include/workcell_builder_ui_utils.hpp')
    cpp = _read('workcell_builder/workcell_builder/gui/workcell_builder_ui_utils.cpp')
    for token in [
        'workcellStudioStyleSheet',
        'applyWorkcellStudioTheme',
        'applyStatusBadgeStyle',
        'applyButtonRoleStyle',
        'primary_action', 'secondary_action', 'preview_action', 'destructive_action'
    ]:
        assert token in hpp or token in cpp


def test_fake_hardware_is_info_not_error():
    cpp = _read('workcell_builder/workcell_builder/gui/workcell_builder_ui_utils.cpp')
    assert 'FakeHardware' in cpp
    assert '#2769b3' in cpp


def test_major_dialogs_apply_compact_defaults():
    files = [
        'workcell_builder/workcell_builder/gui/mainwindow.cpp',
        'workcell_builder/workcell_builder/gui/scene_select.cpp',
        'workcell_builder/workcell_builder/gui/addscene.cpp',
        'workcell_builder/workcell_builder/gui/addrobot.cpp',
        'workcell_builder/workcell_builder/gui/addendeffector.cpp',
        'workcell_builder/workcell_builder/gui/addobject.cpp',
    ]
    for f in files:
        assert 'applyCompactDialogDefaults(this)' in _read(f)


def test_validation_tokens_and_badges_present():
    cpp = _read('workcell_builder/workcell_builder/gui/scene_select.cpp')
    for token in ['READY', 'WARNINGS', 'BLOCKED', 'SCAFFOLD_ONLY', 'fake_hardware_first: true', 'runtime_execution_enabled: false', 'motion_command_sent: false']:
        assert token in cpp
