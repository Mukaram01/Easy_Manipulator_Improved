from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
HPP = Path('workcell_builder/workcell_builder/gui/scene_select.h').read_text()

def test_task_model_exists():
    assert 'struct TaskGraspEditorState' in HPP
    for tok in ['unsaved_task_edits','warnings','blockers','class_routing']:
        assert tok in HPP

def test_ui_model_bindings_exist():
    for tok in ['initialize_task_grasp_editor','sync_task_model_from_editor','rerun_task_validation','connect(ui->task_type_combo']:
        assert tok in CPP
