from pathlib import Path

MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_generated_fallback_uses_locked_urdf_layer_and_primitive_source():
    assert 'p.source_layer = QStringLiteral("locked_generated_urdf_visual");' in MAIN
    assert 'p.active_visual_source = QStringLiteral("primitive_fallback");' in MAIN
    assert 'xacro-expanded visual kept as generated primitive fallback' in MAIN
