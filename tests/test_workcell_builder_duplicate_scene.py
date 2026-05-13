from pathlib import Path

ADD_CPP = Path('workcell_builder/workcell_builder/gui/addscene.cpp').read_text(encoding='utf-8')
GUI_CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')


def test_duplicate_save_as_guardrails_exist_in_editor_flow():
    assert 'Please choose a different scene name or delete the existing scene first.' in ADD_CPP
    assert 'Delete previous scene folder' in GUI_CPP
    assert 'Generate new folder' in GUI_CPP
