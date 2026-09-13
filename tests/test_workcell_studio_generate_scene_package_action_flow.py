from pathlib import Path

MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_generate_scene_package_action_runs_merge_then_generation():
    assert 'Generate Scene Package: requested for scene' in MAIN
    assert 'run_layout_merge_for_selected_scene(true);' in MAIN
    assert 'generate_scene_package_for_selected_scene();' in MAIN


def generation_body():
    return MAIN.split('void MainWindow::generate_scene_package_for_selected_scene() {', 1)[1].split('void MainWindow::validate_generated_scene_for_selected_scene()', 1)[0]


def test_post_generation_parity_waits_for_fresh_matching_payload():
    body = generation_body()
    assert body.index('const int exit_code = process.exitCode()') < body.index('invalidate_workcell_studio_scene_metadata_snapshot')
    assert body.index('invalidate_workcell_studio_scene_metadata_snapshot') < body.index('request_post_save_product_view_refresh()')
    completion = body.index('&ScenePreviewWidget::post_save_product_view_refresh_finished')
    assert body.index('CanvasGeneratedParityMode::PostGeneration') > completion
    assert body.index('serial != generated_refresh_serial_') < body.index('CanvasGeneratedParityMode::PostGeneration')
    assert 'revision != completed_revision' in body
    assert 'generation != completed_generation' in body
    assert body.index('++generated_refresh_serial_') < body.index('process.start(')


def test_refresh_failure_never_sets_launch_ready_or_runs_parity():
    body = generation_body()
    failure = body.split('if (!success) {', 1)[1].split('} else {', 1)[0]
    assert 'final generated/Product View parity could not be established' in failure
    assert 'PostGeneration' not in failure
    assert 'launch_artifacts_ready_ = false;' in body.split('if (!success) {', 1)[0]
    assert 'launch_artifacts_ready_ = ran && !post_blocked;' in body
