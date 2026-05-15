from pathlib import Path

CPP = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")


def test_left_navigation_is_hidden_and_not_added_to_layout():
    assert "studio_nav_->hide();" in CPP
    assert "body->addWidget(studio_nav_);" not in CPP


def test_top_toolbar_primary_actions_and_full_screen_remain():
    for token in [
        'const QStringList action_labels = {"New Cell", "Open Scene", "Validate", "Demo Mode", "Plan & Simulate", "Generate Scene Package", "Export"};',
        'full_screen_button_ = new QPushButton("Full Screen", this);',
    ]:
        assert token in CPP
