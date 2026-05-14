from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()


def test_required_interactive_ui_tokens_exist():
    tokens = [
        'Snap to Grid', 'Fine Move Mode', 'Undo', 'Redo',
        'Duplicate Selected', 'Delete Selected', 'Save Layout', 'Revert Layout',
        'Unsaved Layout Edits', 'Unlock Robot Base'
    ]
    for token in tokens:
        assert token in MAIN


def test_delete_robot_guard_token_exists():
    assert 'Delete robot is blocked/guarded' in MAIN
