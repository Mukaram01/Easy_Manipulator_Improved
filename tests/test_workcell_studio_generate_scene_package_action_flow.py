from pathlib import Path

MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_generate_scene_package_action_runs_merge_then_generation():
    assert 'Generate Scene Package: requested for scene' in MAIN
    assert 'run_layout_merge_for_selected_scene(true);' in MAIN
    assert 'generate_scene_package_for_selected_scene();' in MAIN
