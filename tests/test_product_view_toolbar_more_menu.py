from __future__ import annotations

from pathlib import Path

CPP = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text()


def _between(start: str, end: str) -> str:
    return CPP.split(start, 1)[1].split(end, 1)[0]


def test_product_view_primary_toolbar_keeps_only_core_controls_visible():
    primary = _between(
        'auto make_primary_button = [scene_builder](const QString & text) {',
        'scene_builder_camera_view_button_ = new QToolButton(scene_builder);',
    )

    for text in ["Select", "Place Asset", "Move", "Save Layout"]:
        assert text in primary
    assert 'scene_builder_secondary_overflow_button_->setText("Panels & Tools")' in CPP
    assert 'primary_controls->addWidget(scene_builder_secondary_overflow_button_)' in CPP

    assert 'primary_controls->addWidget(place_mode_persistent_box_)' not in primary
    assert 'primary_controls->addWidget(inspect_mode_button)' not in primary
    assert 'controls->addWidget(camera_view)' not in CPP
    assert '  controls->addWidget(scene_builder_secondary_overflow_button_);' not in CPP


def test_secondary_actions_are_reused_in_more_menu_with_groups_and_state():
    more = _between(
        'scene_builder_secondary_overflow_menu_ = new QMenu(scene_builder_secondary_overflow_button_);',
        'scene_builder_secondary_overflow_button_->setMenu(scene_builder_secondary_overflow_menu_);',
    )

    for token in [
        'addAction(inspect_action)',
        'addAction(keep_placing_action)',
        'addAction(snap_menu_action)',
        'addAction(gizmo_menu_action)',
        'addAction(undo_action)',
        'addAction(redo_action)',
        'addAction(fit_button)',
        'addAction(fit_robot_button)',
    ]:
        assert token in more

    assert more.count('scene_builder_secondary_overflow_menu_->addSeparator();') >= 3
    assert 'setDefaultWidget(place_mode_persistent_box_)' in more
    assert 'canvas_more_menu->menuAction()' in more
    assert 'visual_modes_menu->menuAction()' in more
    assert 'scene_builder_action("layout.undo")' in more
    assert 'scene_builder_action("layout.redo")' in more
    assert 'setShortcut(QKeySequence::Undo)' in more
    assert 'setShortcut(QKeySequence::Redo)' in more
    assert 'QObject::connect(inspect_action, &QAction::triggered, inspect_mode_button, &QPushButton::click)' in more

    assert 'place_mode_persistent_box_->setChecked(false);' in CPP
    assert 'snap_action->setCheckable(true)' in CPP
    assert 'snap_action->setChecked(true)' in CPP
    assert 'fine_move_action->setCheckable(true)' in CPP
