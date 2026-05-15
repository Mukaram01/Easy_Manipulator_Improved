from pathlib import Path

MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
HEADER = Path('workcell_builder/workcell_builder/gui/mainwindow.h').read_text(encoding='utf-8')
SCENE = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
AUDIT_DOC = Path('docs/manuals/WORKCELL_STUDIO_NEW_CELL_FROM_SCRATCH.md').read_text(encoding='utf-8')


def test_required_labels_present_in_source_or_audit_doc():
    labels = [
        'Choose Workspace', 'New Cell', 'New Scene', 'Use Recommended Layout', 'Add to Canvas', 'Save Layout',
        'Remove Selected Layout Item', 'Validate Layout', 'Generate/Update Task Intent', 'Open Task File',
        'Copy Task Summary', 'Generate Scene Package', 'Refresh Existing Scenes', 'Run Offline Validation',
        'Generate Readiness Pack', 'Open Readiness Dashboard', 'Open Plan & Simulate', 'Open RViz2 / MoveIt',
        'Run Fake-Hardware Simulation', 'Stop Simulation', 'Copy Launch Command',
    ]
    corpus = MAIN + SCENE + AUDIT_DOC
    for label in labels:
        assert label in corpus


def test_required_handlers_connected_and_named():
    for token in [
        'on_add_scene_clicked',
        'on_use_recommended_layout_clicked',
        'add_asset_to_canvas_from_catalog',
        '&MainWindow::save_layout_changes',
        '&MainWindow::delete_selected_item',
        '&MainWindow::run_layout_validation_only',
        '&MainWindow::generate_or_update_task_intent_for_selected_scene',
        '&MainWindow::open_selected_task_file',
        '&MainWindow::copy_selected_task_summary',
        'run_layout_merge_for_selected_scene(true)',
        'on_refresh_scenes_button_clicked',
        '&MainWindow::run_offline_validation',
        '&MainWindow::generate_readiness_pack',
        '&MainWindow::open_readiness_dashboard',
        '&MainWindow::run_preview_build',
        '&MainWindow::run_fake_hardware_preview',
        '&MainWindow::stop_preview_process',
        'selected_scene_launch_command()',
    ]:
        assert token in (MAIN + HEADER + SCENE)


def test_no_new_cell_workflow_not_wired_message_usage():
    banned = [
        'show_not_wired_message("New Cell")',
        'show_not_wired_message("New Scene")',
        'show_not_wired_message("Use Recommended Layout")',
        'show_not_wired_message("Add to Canvas")',
        'show_not_wired_message("Save Layout")',
        'show_not_wired_message("Generate Scene Package")',
        'show_not_wired_message("Run Offline Validation")',
        'show_not_wired_message("Open RViz2 / MoveIt")',
    ]
    for token in banned:
        assert token not in MAIN


def test_plan_and_simulate_wording_and_fake_hardware_contract():
    assert 'Plan & Simulate' in MAIN
    assert 'Preview Launch' not in MAIN.split('const QStringList action_labels =')[1].split(';', 1)[0]
    assert 'Generate Scene Package' in MAIN
    assert 'Copy Launch Command' in MAIN
    assert 'use_fake_hardware:=true' in MAIN
    assert 'use_fake_hardware:=false execution button' not in MAIN


def test_audit_doc_includes_each_action_owner_columns():
    assert 'Button and action map' in AUDIT_DOC
    for action in ['Generate Scene Package', 'Run Offline Validation', 'Open RViz2 / MoveIt', 'Run Fake-Hardware Simulation']:
        assert action in AUDIT_DOC
