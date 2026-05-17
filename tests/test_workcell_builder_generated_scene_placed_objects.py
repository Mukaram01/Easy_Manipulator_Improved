from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_generated_scene_tokens_for_placed_objects_present():
    model = (ROOT / 'workcell_builder/workcell_builder/src_object_placement_model.cpp').read_text(encoding='utf-8')
    preview = (ROOT / 'workcell_builder/workcell_builder/gui/placed_object_preview_writer.cpp').read_text(encoding='utf-8')
    assert 'placed_objects:' in model
    assert 'xyz:' in model
    assert 'rpy:' in model
    assert 'fixed' in preview or 'fixed joint' in preview
    assert 'mesh' in preview or 'mesh filename' in preview


def test_safety_words_not_introduced_for_real_hardware_execution():
    docs = (ROOT / 'docs/manuals/SCENE_CREATION_WORKFLOW.md').read_text(encoding='utf-8')
    assert 'fake hardware' in docs.lower()
    assert 'no real hardware' in docs.lower() or 'does not' in docs.lower()
