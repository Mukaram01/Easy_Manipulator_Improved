from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / 'workcell_builder/workcell_builder/src_workcell_studio_layout_editor.cpp').read_text()


def test_validation_warning_tokens_exist():
    for token in [
        'outside robot reach',
        'camera coverage warning',
        'overlap warning',
        'missing pick zone',
        'missing place zone',
    ]:
        assert token in CPP
