from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()

def test_snapshot_token_and_no_interactive_launches():
    assert 'workcell_studio_canvas_snapshot.svg' in MAIN
    for bad in ['rviz', 'gazebo', 'isaac', 'moveit service']:
        assert bad not in MAIN.lower()
