from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_premium_canvas_tokens():
    tokens = [
        'CanvasInteractionMode::Select',
        'CanvasInteractionMode::Place',
        'CanvasInteractionMode::Move',
        'CanvasInteractionMode::Inspect',
        'Snap Grid',
        'snap_step_m_',
        'ghost placement preview committed',
        'digital_twin_minimap',
        'Overlays',
        'Show Reach',
        'Show Camera FOV',
        'Show Pick/Place',
        'Show Trajectory',
        'robot reach circle/arc',
        'camera FOV wedge/cone',
        'pick',
        'place',
        'Qt::Key_Delete',
        'QKeySequence::Save',
        'Qt::Key_Escape',
        'Qt::Key_F',
        'environment_layout.yaml',
    ]
    for token in tokens:
        assert token in CPP


def test_no_real_hardware_runtime_added():
    forbidden = [
        'hardware_mode:=real',
    ]
    for token in forbidden:
        assert token not in CPP
