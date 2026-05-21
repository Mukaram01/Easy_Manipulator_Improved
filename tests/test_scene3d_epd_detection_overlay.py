from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_scene3d_epd_detection_loader_known_file_order_tokens_present():
    for tok in [
        'generated" / "perception_bridge_preview_report.json',
        'generated" / "emd_bridge_payload_preview.json',
        'generated" / "epd_detection_snapshot.json',
        'workcell_studio_detection_snapshot/v1',
    ]:
        assert tok in MAIN_CPP


def test_scene3d_epd_detection_loader_missing_snapshot_warns_without_throw_tokens_present():
    for tok in [
        'detection snapshot missing: generated/perception_bridge_preview_report.json -> generated/emd_bridge_payload_preview.json -> generated/epd_detection_snapshot.json -> workcell_studio_detection_snapshot/v1',
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
