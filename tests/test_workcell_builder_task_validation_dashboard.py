from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/src_validation_dashboard_model.cpp').read_text(encoding='utf-8')

def test_task_grasp_rows_exist():
    for t in ['Pick source selected','Place target selected','Pick source exists in scene/canvas','Place target exists in scene/canvas','Grasp strategy compatible with selected tool','Approach/retreat distances valid','Release strategy compatible with tool','Task intent YAML generated','Preview-only task blocked from runtime claim']:
        assert t in CPP
