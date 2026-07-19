from pathlib import Path

MAIN_CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
QSS = Path('workcell_builder/workcell_builder/gui/resources/workcell_studio_dark.qss').read_text(encoding='utf-8')


def test_scene_builder_workspace_tokens_present():
    for token in [
        'Scene Hierarchy',
        'Asset Catalog',
        'Task Intent',
        'Pick-Place Configuration',
        'Grasp Strategy',
        'Approach & Retreat',
        'Validation',
        'Fake Hardware',
        'No Robot Motion',
        'digital_twin_canvas_',
    ]:
        assert token in MAIN_CPP


def test_scene_builder_uses_left_center_right_layout_structure():
    for token in [
        'left_panel',
        'center_panel',
        'right_panel',
        'scene_top->addWidget(left_panel)',
        'scene_top->addWidget(center_panel, 1)',
        'scene_top->addWidget(right_panel)',
        'sceneBuilderBottomStatusBar',
        'sceneBuilderWorkspace',
    ]:
        assert token in MAIN_CPP


def test_compact_bottom_logs_regression_tokens_present():
    for token in [
        'log_card->setVisible(false);',
        'scene_builder_log_panel_->setVisible(show);',
        'studio_log_->setVisible(show);',
        'QApplication::clipboard()->setText(studio_log_->toPlainText());',
        'if (studio_log_) studio_log_->clear();',
        'scene_builder_status_message_label_->setText(message);',
        'scene_builder_issue_count_label_->setText(QString("Warnings: %1 | Errors: %2")',
        'scene_builder_selection_summary_label_->setText(QString("Selection: %1 (%2)")',
    ]:
        assert token in MAIN_CPP
    assert 'bottom_cards' not in MAIN_CPP
    assert 'Simulation Log' not in MAIN_CPP
    assert 'Cycle/Timing Summary' not in MAIN_CPP
    assert MAIN_CPP.count('sceneBuilderSelectionSummary') == 1
    assert MAIN_CPP.count('new QTextEdit(log_card)') == 1
    assert 'new QPlainTextEdit(logs_tab)' not in MAIN_CPP


def test_studio_card_and_panel_styling_tokens_present():
    assert 'studioCard' in MAIN_CPP
    assert 'QFrame#studioCard' in QSS
    assert 'QFrame#studioPanel' in QSS
