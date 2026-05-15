from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_canvas_polish_tokens_present():
    tokens = [
        'digital_twin_canvas_',
        'Toggle Grid',
        'robot reach circle/arc',
        'camera FOV wedge/cone',
        'pick',
        'place',
        'on_canvas_selection_changed',
        'Fit Cell',
        'Reset View',
        'Zoom In',
        'Zoom Out',
        'Toggle Labels',
        'Toggle Warnings',
        'Export Canvas Snapshot',
        'workcell_studio_canvas_snapshot.png',
        'preview',
        'missing environment.yaml',
        'missing package.xml',
        'missing launch/demo.launch.py',
        'missing task intent',
        'missing robot/gripper metadata',
        'Fake Hardware',
        'No Robot Motion',
        'Preview Only',
    ]
    for token in tokens:
        assert token in CPP


def test_canvas_controls_do_not_use_not_wired_message():
    forbidden = [
        'show_not_wired_message("Fit Cell")',
        'show_not_wired_message("Reset View")',
        'show_not_wired_message("Zoom In")',
        'show_not_wired_message("Zoom Out")',
        'show_not_wired_message("Toggle Grid")',
        'show_not_wired_message("Toggle Labels")',
        'show_not_wired_message("Toggle Warnings")',
        'show_not_wired_message("Export Canvas Snapshot")',
    ]
    for token in forbidden:
        assert token not in CPP
