from pathlib import Path

MANUALS = Path('docs/manuals')


def test_new_manuals_exist():
    for name in [
        'WORKCELL_STUDIO_OVERVIEW.md',
        'WORKCELL_STUDIO_QUICK_START.md',
        'WORKCELL_STUDIO_NEW_CELL_FROM_SCRATCH.md',
        'WORKCELL_STUDIO_CANVAS_EDITOR.md',
        'WORKCELL_STUDIO_PLAN_SIMULATE.md',
        'WORKCELL_STUDIO_ACCEPTANCE_GATE.md',
        'WORKCELL_STUDIO_TROUBLESHOOTING.md',
        'WORKCELL_STUDIO_SCENE_BUNDLES.md',
        'ENVIRONMENT_LAYOUT_V1.md',
    ]:
        assert (MANUALS / name).is_file()


def test_manuals_readme_links_all_new_docs():
    text = (MANUALS / 'README.md').read_text(encoding='utf-8')
    for token in [
        'WORKCELL_STUDIO_OVERVIEW.md','WORKCELL_STUDIO_QUICK_START.md','WORKCELL_STUDIO_NEW_CELL_FROM_SCRATCH.md',
        'WORKCELL_STUDIO_CANVAS_EDITOR.md','WORKCELL_STUDIO_PLAN_SIMULATE.md','WORKCELL_STUDIO_ACCEPTANCE_GATE.md',
        'WORKCELL_STUDIO_SCENE_BUNDLES.md','WORKCELL_STUDIO_TROUBLESHOOTING.md','ENVIRONMENT_LAYOUT_V1.md'
    ]:
        assert token in text


def test_no_preview_launch_primary_wording_in_manuals():
    new_docs = ['WORKCELL_STUDIO_OVERVIEW.md','WORKCELL_STUDIO_QUICK_START.md','WORKCELL_STUDIO_NEW_CELL_FROM_SCRATCH.md','WORKCELL_STUDIO_CANVAS_EDITOR.md','WORKCELL_STUDIO_PLAN_SIMULATE.md','WORKCELL_STUDIO_ACCEPTANCE_GATE.md','WORKCELL_STUDIO_TROUBLESHOOTING.md','WORKCELL_STUDIO_SCENE_BUNDLES.md']
    corpus = '\n'.join((MANUALS / n).read_text(encoding='utf-8') for n in new_docs)
    assert 'Preview Launch' not in corpus


def test_plan_and_simulate_has_fake_hardware_launch_command():
    text = (MANUALS / 'WORKCELL_STUDIO_PLAN_SIMULATE.md').read_text(encoding='utf-8')
    assert 'ros2 launch <scene_name> demo.launch.py use_fake_hardware:=true launch_rviz:=true' in text


def test_acceptance_gate_doc_mentions_master_script():
    text = (MANUALS / 'WORKCELL_STUDIO_ACCEPTANCE_GATE.md').read_text(encoding='utf-8')
    assert 'run_workcell_studio_acceptance_gate.py' in text


def test_new_cell_expected_files_present():
    text = (MANUALS / 'WORKCELL_STUDIO_NEW_CELL_FROM_SCRATCH.md').read_text(encoding='utf-8')
    for token in ['environment_layout.yaml','config/workcell_builder_task_intent.yaml','package.xml','CMakeLists.txt','launch/demo.launch.py']:
        assert token in text


def test_canvas_editor_includes_required_features():
    text = (MANUALS / 'WORKCELL_STUDIO_CANVAS_EDITOR.md').read_text(encoding='utf-8')
    for token in ['Add to Canvas', 'Save Layout', 'Snap Grid', 'Minimap', 'Overlays']:
        assert token in text


def test_troubleshooting_error_codes_present():
    text = (MANUALS / 'WORKCELL_STUDIO_TROUBLESHOOTING.md').read_text(encoding='utf-8')
    for token in ['MISSING_WORKSPACE', 'MISSING_COLCON', 'MISSING_DEMO_LAUNCH']:
        assert token in text


def test_old_docs_removed_or_redirected():
    for name in ['WORKCELL_STUDIO_NEW_CELL_BUTTON_AUDIT.md','WORKCELL_STUDIO_NEW_CELL_FLOW.md','WORKCELL_STUDIO_UI_LAYOUT.md','WORKCELL_STUDIO_PLAN_SIMULATE_MODE.md']:
        p = MANUALS / name
        if p.exists():
            assert 'Moved to' in p.read_text(encoding='utf-8')
