from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT/'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_generated_fallback_dimension_inference_tokens_present():
    for token in ['lower_name.contains("base")','upper_arm','forearm','wrist','tool0','gripper','camera','workbench']:
        assert token in CPP
