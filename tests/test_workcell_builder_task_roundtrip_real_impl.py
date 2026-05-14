from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()

def test_roundtrip_hooks_present():
    for tok in ['task_editor_state_.unsaved_task_edits = false','workcell_builder_task_intent.yaml']:
        assert tok in CPP
