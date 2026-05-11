from pathlib import Path


def test_layout_preview_and_readiness_strings_exist():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
    assert 'Workcell Studio Readiness' in cpp
    for status in ['READY_TO_GENERATE', 'WARNINGS', 'BLOCKED', 'SCAFFOLD_ONLY']:
        assert status in cpp
    for blocker in [
        'missing robot',
        'missing robot MoveIt config package',
        'missing STL path',
        'placeholder unknown/none/null values',
    ]:
        assert blocker in cpp
    for warning in [
        'external absolute STL path',
        'conveyor_placeholder is visual/metadata only',
        'real hardware mode requires explicit validation',
    ]:
        assert warning in cpp


def test_preview_and_summary_artifact_paths_and_labels_exist():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
    assert 'Refresh Preview' in cpp
    assert 'Export Preview' in cpp
    assert 'preview/workcell_preview.svg' in cpp
    assert 'preview/workcell_preview.html' in cpp
    assert 'Workcell Studio Preview' in cpp
    assert 'Offline/fake-hardware layout preview only' in cpp
    assert 'workcell_studio_summary.json' in cpp
    assert 'workcell_studio_summary.md' in cpp
    assert 'colcon build --symlink-install --packages-select' in cpp
    assert 'ros2 launch ' in cpp
    assert 'use_fake_hardware:=true' in cpp
    assert 'use_fake_hardware:=false' in cpp
    assert 'unknown_description' not in cpp
    assert 'unknown_moveit_config' not in cpp


def test_validate_scene_is_offline_only_text():
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
    assert 'Validate Scene completed (offline only, no launch/motion/runtime execution).' in cpp
