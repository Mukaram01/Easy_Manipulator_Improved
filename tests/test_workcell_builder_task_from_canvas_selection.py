from pathlib import Path
UI = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')

def test_canvas_to_task_tokens():
    for t in ['Use Selected Item as Pick Source','Use Selected Item as Place Target','Use Selected Zone as Pick Zone','Use Selected Zone as Place Zone','Unsaved Task Edits: visible']:
        assert t in UI or t in CPP
