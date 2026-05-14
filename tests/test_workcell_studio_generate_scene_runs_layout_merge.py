from pathlib import Path

MAIN_CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_generate_scene_calls_layout_merge():
    assert 'if (label == "Generate Scene") { run_layout_merge_for_selected_scene(true); return; }' in MAIN_CPP
    assert 'Layout has unsaved edits. Save Layout first.' in MAIN_CPP
