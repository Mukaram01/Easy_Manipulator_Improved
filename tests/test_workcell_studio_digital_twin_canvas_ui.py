from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()

def test_ui_tokens():
    for t in ['Digital Twin Canvas','Fit Cell','Reset View','Toggle Grid','Toggle Labels','Toggle Warnings','Export Canvas Snapshot','Pick Source','Grasp Strategy','Place Target','Release','Camera FOV','Robot Reach','No Robot Motion']:
        assert t in MAIN
