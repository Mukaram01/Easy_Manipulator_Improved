from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
HDR = Path('workcell_builder/workcell_builder/gui/mainwindow.h').read_text(encoding='utf-8')


def test_selected_scene_state_helpers_are_declared_and_used():
    assert 'QString selected_scene_path() const;' in HDR
    assert 'bool has_selected_scene() const;' in HDR
    assert 'void refresh_scene_builder_selected_scene_ui();' in HDR
    assert 'refresh_scene_builder_selected_scene_ui();' in CPP
    assert 'rebuild_digital_twin_canvas();' in CPP


def test_open_scene_builder_refreshes_scene_state_and_canvas():
    assert "open_scene_builder_for_selected_scene" in CPP
    assert "opened Scene Builder for '%1' at %2" in CPP
    assert 'refresh_task_intent_panel();' in CPP


def test_no_scene_selected_message_is_only_empty_selection_state():
    assert 'if (!has_selected_scene()) {' in CPP
    assert 'canvas_header_label_->setText("No scene selected")' in CPP


def test_preview_empty_state_message_exists_for_missing_metadata():
    assert 'Scene selected but no previewable layout metadata found. Run Generate Preview/Readiness Pack or add layout items.' in CPP
