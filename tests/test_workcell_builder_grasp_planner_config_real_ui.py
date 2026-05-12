from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
UI = ROOT / 'workcell_builder/workcell_builder/gui/conveyor_sorting_run_console.ui'

def test_ui_static_required_sections_and_actions():
    text = UI.read_text(encoding='utf-8')
    for token in [
        'EMD Grasp Planner Config','EPD / Perception','Camera','Point Cloud Filtering','End Effector','Grasp Ranking','Visualization / Debug',
        'Validate Planner Config','Generate EMD Files','Copy Planner Launch Command','Copy Execution Launch Command'
    ]:
        assert token in text
