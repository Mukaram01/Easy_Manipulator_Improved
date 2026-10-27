from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


PREVIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
CANVAS_MODEL_CPP = (ROOT / 'workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp').read_text(encoding='utf-8')


def test_malformed_metadata_warns_not_crash_contract_present():
    for token in [
        'Malformed layout/workcell_studio_layout.yaml; falling back safely',
        'failed to parse generated/scene_visual_mesh_index.json',
        'YAML parse exception in preview loader',
        'std exception in preview loader',
    ]:
        assert token in CANVAS_MODEL_CPP or token in MAIN_CPP
