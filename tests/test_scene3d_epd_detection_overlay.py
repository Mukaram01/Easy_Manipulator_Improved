from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
PREVIEW_H = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.h').read_text(encoding='utf-8')
MODEL_H = (ROOT / 'workcell_builder/workcell_builder/include/workcell_studio_canvas_model.hpp').read_text(encoding='utf-8')


def test_detection_overlay_optional_fields_exist_in_preview_and_model_contracts():
    for token in ['camera_id', 'frame_id', 'detection_label', 'confidence', 'tracking_id', 'snapshot_source_file', 'alignment_warning']:
        assert token in PREVIEW_H
        assert token in MODEL_H


def test_detection_overlay_items_are_forced_preview_overlay_and_read_only():
    assert 'source_layer = QStringLiteral("overlay")' in MAIN_CPP
    assert 'active_visual_source = QStringLiteral("overlay")' in MAIN_CPP
    assert 'p.editable = false;' in MAIN_CPP
    assert 'p.selectable = true;' in MAIN_CPP
