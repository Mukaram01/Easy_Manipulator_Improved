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


def test_scene3d_epd_detection_loader_known_file_order_tokens_present():
    for tok in [
        'generated" / "perception_bridge_preview_report.json',
        'generated" / "emd_bridge_payload_preview.json',
        'generated" / "epd_detection_snapshot.json',
        'workcell_studio_detection_snapshot/v1',
    ]:
        assert tok in MAIN_CPP


def test_scene3d_epd_detection_loader_missing_snapshot_warns_only_when_required():
    for tok in [
        'required detection snapshot missing: generated/perception_bridge_preview_report.json -> generated/emd_bridge_payload_preview.json -> generated/epd_detection_snapshot.json -> workcell_studio_detection_snapshot/v1',
        'const bool perception_snapshot_required',
        'if (missing_optional_snapshot) continue;',
        'Scene3D detection snapshot warning:',
        'scene_preview_widget_->set_epd_detection_overlays(snapshot_preview.detections);',
    ]:
        assert tok in MAIN_CPP


def test_scene3d_epd_detection_loader_malformed_snapshot_warns_without_throw_tokens_present():
    for tok in [
        'malformed snapshot:',
        'schema mismatch',
        'model.warnings << snapshot_warning;',
    ]:
        assert tok in MAIN_CPP
